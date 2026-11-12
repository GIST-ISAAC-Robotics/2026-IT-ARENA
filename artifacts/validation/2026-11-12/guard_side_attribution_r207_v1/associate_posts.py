"""이미 추출한 반환점의 시설 근접 대조. 제어기 재실행/입력은 없다."""
from pathlib import Path
import json,math
from collections import Counter
import numpy as np
from shapely.geometry import Point
from build_experimental_track import rectangle
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
run=A.parent/'released_mount_runtime_r203_v1/two_actor_run1/run1'
data=json.loads((A/'diagnosis.json').read_bytes())
ego=np.asarray([json.loads(x) for x in (run/'ego_poses.jsonl').open()]);ego[:,3]=np.unwrap(ego[:,3])
meta=json.loads((ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1/marker_posts_provisional.json').read_bytes())
posts={p['name']:rectangle(*p['center'],p['yaw'],p['length'],p['width']) for p in meta['links'] if p['part']=='post'}
signal=json.loads((run/'signal_posts.json').read_bytes())
for p in signal['links']:
    if p['name'].startswith('tl_post'):
        posts[p['name']]=rectangle(*p['center_xy_m'],signal['beam_yaw_rad'],p['along_m'],p['across_m'])
rows=[]
for v in data['views']:
    x,y,yaw=[float(np.interp(v['reference_s'],ego[:,0],ego[:,i])) for i in (1,2,3)]
    c,s=math.cos(yaw),math.sin(yaw)
    for side,points in v['sides'].items():
        for p in points:
            px,py=p['xy'];q=Point(x+c*px-s*py,y+s*px+c*py)
            distance,name=min((q.distance(box),name) for name,box in posts.items())
            rows.append(dict(seq=v['seq'],side=side,source_index=p['source_index'],nearest_post=name,distance_m=distance))
summary=dict(points=len(rows),nearest_post_counts=dict(Counter(r['nearest_post'] for r in rows)),
    distance_m=dict(min=min(r['distance_m'] for r in rows),median=float(np.median([r['distance_m'] for r in rows])),max=max(r['distance_m'] for r in rows)),
    scope='evaluation association, not a controller pole classifier')
with (A/'post_association.json').open('x') as f:json.dump(dict(summary=summary,points=rows),f,indent=2);f.write('\n')
print(json.dumps(summary))
