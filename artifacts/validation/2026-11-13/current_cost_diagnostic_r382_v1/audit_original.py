"""r380 회수본 전체 10행의 원 기대값 차이만 읽는다. 제어기 import/실행 없음."""
import hashlib
import json
from pathlib import Path

from diff_utils import comparison, recorded_result

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
SOURCE = A.parent/'current_cost_r380_v1/remote_result/extracted'
TRACE = A.parent/'relative_body_cad_drive_r371_v1/run1/follow_decisions.jsonl'


def main():
    target = A/'original_diff.json'
    assert not target.exists(), 'original audit is immutable'
    expected = []
    with TRACE.open(encoding='utf8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'call' and 'current_rebase' in event['data']:
                expected.append(event['data']['current_rebase']['output'])
    rows = [json.loads(line) for line in (SOURCE/'jetson/timing_rows.jsonl').read_text(encoding='utf8').splitlines()]
    audits = []
    for row in rows:
        reference = expected[row['index']]
        restored, cap_check = recorded_result(row['before'], reference)
        audits.append(dict(index=row['index'], reference_s=row['reference_s'], mode=row['selector_mode'],
            applied=row['applied'], x86_original_vs_arm_baseline=comparison(reference, restored),
            parent_cap_precondition=cap_check, same_arm_baseline_vs_candidate=comparison(row['before'], row['after'])))
    files = [TRACE, SOURCE/'jetson/report.json', SOURCE/'jetson/timing_rows.jsonl', SOURCE/'execution.json',
             SOURCE/'import_preflight.json', SOURCE/'native_build.log', SOURCE/'import_preflight.log', SOURCE/'jetson_run.log']
    report = dict(schema='r382_original_r380_audit_v1', source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        original_failure_preserved=True, original_report=json.loads((SOURCE/'jetson/report.json').read_bytes()),
        checked_rows=len(rows), original_comparator='numeric math.isclose(rel_tol=1e-10, abs_tol=1e-10); structure/boolean exact',
        exact_mismatch_indices=[r['index'] for r in audits if not r['x86_original_vs_arm_baseline']['exact_tree_equal']],
        original_comparator_mismatch_indices=[r['index'] for r in audits if not r['x86_original_vs_arm_baseline']['existing_comparator_equal']],
        same_arm_changed_indices=[r['index'] for r in audits if not r['same_arm_baseline_vs_candidate']['exact_tree_equal']],
        rows=audits,
        cause_status='unconfirmed: exact-coordinate deduplication and strict geometric boundaries can amplify tiny floating point differences; no matched x86 intermediate query capture exists')
    target.write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k:report[k] for k in ('checked_rows','exact_mismatch_indices','original_comparator_mismatch_indices','same_arm_changed_indices')}))


if __name__ == '__main__': main()
