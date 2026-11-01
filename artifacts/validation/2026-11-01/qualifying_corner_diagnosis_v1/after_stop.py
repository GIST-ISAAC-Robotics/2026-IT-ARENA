"""경계 상실 전후 판단과 정지 뒤 재획득 시도를 원시 판단에서 요약한다."""
import collections
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'qualifying_profile_v1/run1'
report = json.loads((RUN / 'report.json').read_bytes())
for k in ('passed', 'error', 'failure', 'failure_reason', 'stop_reason', 'completed'):
    if k in report:
        print(k, str(report[k])[:900])
print('report keys', sorted(report)[:80])
EVIDENCE = ('corner_distance_evidence', 'fragmented_boundary_evidence', 'corner_boundary_evidence')
seen = collections.Counter()
last = None
with (RUN / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] != 'call':
            continue
        d = r['data']
        i = d['input']
        t = i['reference_s']
        if t < 45.6:
            continue
        target, steering, det = d['output']
        summary = {k: det.get(k) for k in (
            'reason', 'underlying_reason', 'boundary_selection', 'boundary_candidate_reason', 'boundary_candidate_ahead_m',
            'fresh_boundary_accepted', 'bilateral_alternative_confirmed', 'corner_distance_agrees_memory',
            'fragmented_agrees_memory', 'boundary_protrusions_removed', 'boundary_support_points')}
        for name in EVIDENCE:
            if name in det:
                e = det[name]
                summary[name] = {k: (v if not isinstance(v, (list, dict)) else f'{type(v).__name__}[{len(v)}]') for k, v in e.items()}
        if det.get('boundary_selection') is not None or 'corner_distance_evidence' in det:
            key = (summary['reason'], summary['boundary_selection'], summary['boundary_candidate_reason'],
                   (summary.get('corner_distance_evidence') or {}).get('reason'),
                   (summary.get('fragmented_boundary_evidence') or {}).get('reason'))
            seen[key] += 1
            if t < 46.5 or seen[key] <= 2:
                print(f'{t:.3f} meas={i["speed_mps"]:.2f} side={i["side"]} tgt={target:.2f}', json.dumps(summary, ensure_ascii=False)[:900])
print('--- counts after 45.6')
for k, v in seen.most_common():
    print(v, k)
