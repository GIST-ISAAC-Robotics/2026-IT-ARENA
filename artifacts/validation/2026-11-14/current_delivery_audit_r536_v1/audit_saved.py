"""Windows 저장 로그/현재 소스 감사. ROS·WSL·네트워크를 실행하지 않는다."""
import ast
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
RAW = A.parent/'jetson_drain_budget_r523_v1/remote_result/result'
PRIOR = A.parent/'fresh_current_delivery_r526_v1/saved_timing.json'
FILES = [
    'g1_g2_node.py', 'runtime_node_backend.py', 'runtime_dual_delivery.py',
    'runtime_dual_state.py', 'runtime_dual_process.py', 'completion_notice.py',
    'wall_follow.py', 'lidar_motion_ros.py', 'follow_trace.py', 'lidar_safety.py',
]


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    prior = json.loads(PRIOR.read_bytes())
    raw = {name: (RAW/name).read_bytes() for name in prior['source_sha256']}
    actual = {name: sha(data) for name, data in raw.items()}
    assert actual == prior['source_sha256'], 'r526 evidence source changed'
    commands = [json.loads(line) for line in gzip.decompress(raw['commands.jsonl.gz']).splitlines()]
    drive = [r for r in commands if r['topic']=='/drive']
    positive = [r for r in drive if r['speed']>0 and 7.62<=r['received_s']<27.52]
    controls = json.loads(raw['controller_controls.json'])
    receipts = json.loads(raw['calculation_receipts.json'])
    currents = [r for r in receipts if r['stage']=='current']
    by_ref = {round(r['source_s']*1e9): r for r in positive}
    missing, matched = [], []
    for row in currents:
        owners = [r for r in controls if r['begin_monotonic_ns']/1e9 <= row['submitted_wall_s']
                  <= r['end_monotonic_ns']/1e9]
        assert len(owners)==1
        ref = owners[0]['control_reference_s']
        command = by_ref.get(round(ref*1e9))
        if command is not None:
            matched.append(row)
            continue
        nearby_plans = [r for r in receipts if r['stage']=='plan' and
                        abs(r['received_wall_s']-row['received_wall_s'])<.12]
        missing.append(dict(sequence=row['sequence'], generation=row['generation'],
            original_current_reference_s=ref,
            current_receipt=row, matching_any_drive=[r for r in drive if abs(r['source_s']-ref)<1e-8],
            nearby_plan_receipts=nearby_plans,
            classification=('explicit_new_plan_superseded' if row.get('discard_reason') ==
                            'new_plan_superseded_current_check' else 'unresolved_no_ready_transition_evidence')))
    assert len(currents)==182 and len(matched)==176 and len(missing)==6
    source = {}
    method_lines = {}
    for name in FILES:
        path = ROOT/'src/arena_autonomy/arena_autonomy'/name
        data = path.read_bytes()
        source[str(path.relative_to(ROOT)).replace('\\','/')] = sha(data)
        tree = ast.parse(data.decode('utf8'))
        method_lines[name] = {node.name: node.lineno for node in ast.walk(tree)
                              if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))}
    result = dict(scope='Windows local saved-log and current-source audit; no execution or performance counterfactual',
        positive_drive_count=len(positive), current_receipt_count=len(currents), matched_positive_count=len(matched),
        current_unpublished_classification=dict(Counter(r['classification'] for r in missing)),
        missing_current=missing, saved_r526_summary={k:prior[k] for k in (
            'current_collection_to_command_observer_ms','matched_ready_before_100ms_after_previous_observer',
            'angle_change_fits_20ms','source_age_at_observer_ms','remaining_command_ttl_at_observer_ms')},
        not_observable_from_saved_logs=[
            'Full ready output and every prepared/application_reason result',
            'Actual current previous angle, publication dt and queue state at each historical poll',
            'Exact /drive publisher entry/return and guard command callback entry',
            'Post-publication status/bookkeeping latency separated from subscriber observation',
            'Outcomes or timing after r526+compact are integrated',
        ])
    manifest = dict(source_sha256=source,source_method_lines=method_lines,
        evidence_sha256={**actual,'r526_saved_timing.json':sha(PRIOR.read_bytes())},
        no_network=True,no_ros=True,no_wsl=True,common_source_edits=False)
    for name,data in [('saved_audit.json',result),('manifest.json',manifest)]:
        (A/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    assert all(sha((RAW/name).read_bytes())==expected for name,expected in actual.items())
    assert all(sha((ROOT/name).read_bytes())==expected for name,expected in source.items())
    print(json.dumps(dict(saved_source_hashes_match=True,current_sources=len(source),
        current_receipts=len(currents),positive_matches=len(matched),
        unissued=result['current_unpublished_classification'],source_unchanged=True)))


if __name__=='__main__':
    main()
