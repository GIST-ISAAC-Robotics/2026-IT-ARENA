"""새 실행의 정체·종료와 후반 분류 사용을 회수한다. 원래 판정은 유지한다."""
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
    if anchor:
        counts['anchor:'+str(anchor.get('reason'))] += 1
    if anchor.get('released'):
        events.append(dict(t_s=data['input']['reference_s'], kind='anchor_release', command=data['output'][:2]))
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
              recovery_counts=dict(counts), recovery_events=events,
              rear_fringe_maximum_exterior_m=max((v for v in extents if v is not None), default=None))
out.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
print(json.dumps({k: result[k] for k in ('report_passed','run_valid','failed_gates','progress',
                                       'recovery_counts','rear_fringe_maximum_exterior_m',
                                       'stationary_spans_at_least_2p5s')}, ensure_ascii=False))
