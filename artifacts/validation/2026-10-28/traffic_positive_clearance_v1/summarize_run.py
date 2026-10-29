"""선택형 재획득의 실제 분기와 기존 판정을 읽기 전용으로 요약한다."""
from collections import Counter
import importlib.util
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
source = ROOT/'artifacts/validation/2026-10-27/g1_traffic_lap_v1/summarize_run.py'
spec = importlib.util.spec_from_file_location('traffic_summary', source)
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)
run, out = BASE/'run1_course', BASE/'run1_diagnostics.json'
if out.exists():
    raise FileExistsError(out)
result = summary.summarize(run)
counts, events, extents = Counter(), [], []
clearance_events=[]
for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
    row = json.loads(line)
    if row['kind'] == 'header':
        header = row['data']
        continue
    if row['kind'] != 'call':
        continue
    data = row['data']
    meta = data['output'][2]
    partial = meta.get('partial_observation') or {}
    anchor = partial.get('anchor_diagnostic') or {}
    clearance = partial.get('clearance_diagnostic') or {}
    corridor = meta.get('corridor_candidate') or {}
    if anchor:
        counts['anchor:'+str(anchor.get('reason'))] += 1
    if clearance:
        counts['clearance:'+str(clearance.get('reason'))] += 1
        if clearance.get('released'):
            counts['clearance_releases'] += 1
            clearance_events.append(dict(t_s=data['input']['reference_s'],clearance=clearance,
                command=data['output'][:2],reason=meta.get('reason')))
    if meta.get('corridor_selected'):
        counts['selected_role:'+str(corridor.get('role_selection'))] += 1
        counts['selected_role_positive:'+str(corridor.get('role_selection'))] += int(data['output'][0] > 0.)
    if meta.get('suspended_passage_history'):
        counts['suspended_history_calls'] += 1
        counts['suspended_history_positive'] += int(data['output'][0] > 0.)
    if meta.get('restored_passage_history'):
        counts['restored_history_calls'] += 1
        events.append(dict(t_s=data['input']['reference_s'],
            restored=meta['restored_passage_history'],command=data['output'][:2],
            reason=meta.get('reason'),selector_reason=meta.get('selector_reason'),
            retained_passage=meta.get('retained_passage'),corridor=corridor))
    if meta.get('retained_passage'):
        counts['retained_passage_calls'] += 1
    if meta.get('selector_bounded_rear_fringe_risk_accepted'):
        counts['rear_fringe_accepted'] += 1
        counts['rear_fringe_positive'] += int(data['output'][0] > 0.)
        fringe = meta.get('selector_rear_fringe') or (meta.get('selector_guard_candidate') or {}).get('rear_fringe') or {}
        extents.append(fringe.get('exterior_max_m'))
    if meta.get('selector_same_path_gap_risk_accepted'):
        counts['same_path_accepted'] += 1
result.update(runtime_traffic_recovery_enabled=header['follow_config'].get('traffic_recovery_trial'),
    runtime_traffic_path_recovery_enabled=header['follow_config'].get('traffic_path_recovery_trial'),
    recovery_counts=dict(counts),restored_history_events=events,clearance_events=clearance_events,
    rear_fringe_maximum_exterior_m=max((v for v in extents if v is not None),default=None))
out.write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
print(json.dumps({k: result[k] for k in ('report_passed','run_valid','failed_gates','progress',
    'recovery_counts','rear_fringe_maximum_exterior_m','stationary_spans_at_least_2p5s')},ensure_ascii=False))
