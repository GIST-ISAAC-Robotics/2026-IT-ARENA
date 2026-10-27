"""이미 끝난 두 주행의 겹치는 진행 구간 비교. 지도는 평가에만 사용한다."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

OUT=Path(__file__).resolve().parent
paths=[OUT.parent/'partial_observation_v1/run10_lap/trajectory.jsonl',OUT/'run1_lap/trajectory.jsonl']
rows=[[json.loads(line) for line in p.open(encoding='utf-8')] for p in paths]
active=[[r for r in run if r['autonomy'].get('started') and r['progress_m']<=43.] for run in rows]
matched=[]
for row in active[1]:
    other=min(active[0],key=lambda x:abs(x['progress_m']-row['progress_m']))
    if abs(other['progress_m']-row['progress_m'])<=.025:
        matched.append(dict(progress_m=row['progress_m'],progress_difference_m=other['progress_m']-row['progress_m'],
            position_difference_m=math.hypot(row['x']-other['x'],row['y']-other['y'])))
result=dict(scope='sampled_overlapping_progress_prefix_not_causal_identity_or_new_drive',
    progress_limit_m=43.,pair_progress_tolerance_m=.025,matched_samples=len(matched),
    max_position_difference_m=max(r['position_difference_m'] for r in matched),
    rows=[dict(file=str(p.relative_to(OUT.parent)),samples=len(run),
        side_counts=dict(Counter(r['autonomy']['side'] for r in run)),
        path_wall_counts=dict(Counter(r['autonomy'].get('path_wall') for r in run)),
        fallback_count=sum(bool(r['autonomy'].get('wall_fallback')) for r in run),
        observed_marker_ids=sorted({i for r in run for i in r['autonomy'].get('marker_ids',[])}))
        for p,run in zip(paths,active)],
    source_sha256={str(p.relative_to(OUT.parent)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
result['source_sha256'][str(Path(__file__).relative_to(OUT.parent))]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
with (OUT/'run1_prefix_comparison.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
print(json.dumps(result))
