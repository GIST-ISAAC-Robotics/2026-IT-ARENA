"""코너 대체 경로(corridor) 판단 필드를 성공·실패 실행에서 비교한다."""
import collections
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNS = {'rise_only': (ROOT / 'race_acceleration_v1/run1', 46.9, 47.32),
        'combined_failed': (ROOT / 'qualifying_profile_v1/run1', 45.7, 46.6)}


def brief(v, depth=0):
    if isinstance(v, dict):
        if depth >= 3:
            return f'dict[{len(v)}]'
        return {k: brief(x, depth + 1) for k, x in v.items()
                if k not in ('path_points', 'path', 'source_indices', 'points', 'wall_points')}
    if isinstance(v, list):
        return v if len(v) <= 4 and all(not isinstance(x, (list, dict)) for x in v) else f'list[{len(v)}]'
    if isinstance(v, float):
        return round(v, 3)
    return v


for name, (folder, lo, hi) in RUNS.items():
    print('#', name)
    reasons = collections.Counter()
    with (folder / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            if r['kind'] != 'call':
                continue
            t = r['data']['input']['reference_s']
            if t < lo:
                continue
            det = r['data']['output'][2]
            cc = det.get('corridor_candidate')
            key = (det.get('reason'), (cc or {}).get('reason') if isinstance(cc, dict) else cc,
                   det.get('corridor_selected'))
            reasons[key] += 1
            if t <= hi and det.get('boundary_selection') is not None:
                po = det.get('partial_observation')
                print(f'{t:.2f}', det.get('reason'), 'sel=', det.get('boundary_selection'),
                      'cs=', det.get('corridor_selected'), 'cc=', json.dumps(brief(cc), ensure_ascii=False)[:700],
                      'po=', json.dumps(brief(po), ensure_ascii=False)[:300])
    print('counts', reasons.most_common(10))
