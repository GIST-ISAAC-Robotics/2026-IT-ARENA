"""run9 마지막 실제 스캔의 정지 자세 대조. 과거 전체 주행 재생이 아니다."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'tests')]
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_local_follow import history,scan

path=OUT/'run9_lap/last_scan.json'
raw=json.loads(path.read_text())
ranges=[float('nan') if x is None else x for x in raw['ranges']]
h=history()
s=scan(.5,ranges)
for field in ('angle_min','angle_increment','range_min','range_max','scan_time','time_increment'):
    setattr(s,field,raw[field])
points,_,_=h.project(s.ranges,s.angle_min,s.angle_increment,s.range_min,s.range_max,
                    .5,s.time_increment,.6,'both',True)
rows=[]
for forward in (False,True):
    p=PartialObservationPolicy(allow_narrow_forward=forward)
    p._remember(points,.5,.4,True)
    result=p.evaluate(s,points,h,.6,1.,.3674,.3674)
    rows.append(dict(policy='bounded_local_v2' if forward else 'bounded_side_v1',result=result))
unknown=[i for i,x in enumerate(ranges) if not np.isfinite(x) and abs(s.angle_min+i*s.angle_increment)<np.radians(100)]
result=dict(scope='stationary_saved_scan_not_dynamic_replay',rows=rows,
    unknown_angles_deg=[float(np.degrees(s.angle_min+i*s.angle_increment)) for i in unknown],
    neighbors_m=[[ranges[i-1],ranges[i+1]] for i in unknown],
    source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in
        (path,Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py')})
assert rows[0]['result']['reason']=='central_missing'
assert rows[1]['result']['allowed'] and rows[1]['result']['speed_cap_mps']==.2
assert not rows[1]['result']['unknown_filled']
with (OUT/'run10_forward_probe.json').open('x') as stream:json.dump(result,stream,indent=2)
print(json.dumps(result))
