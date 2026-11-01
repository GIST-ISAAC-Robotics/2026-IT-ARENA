"""경기 출발칸의 횡위치를 포함해 상대 경로의 배치 가능성만 비교한다."""
import json
import math
import prepare as plan
import validate_g1_traffic as ev
from prelap_signal_eval import FIXTURE

cfg,_=plan.expected();line,scene=ev.load_centerline(FIXTURE)
s=ev.actor_start_s(cfg,scene,0,{'actor_start_center_gap_m':0.})
x,y,yaw=line.pose(s)
first=json.loads(plan.REFERENCE.read_bytes())[0]
ego_lateral=-(first['x']-x)*math.sin(yaw)+(first['y']-y)*math.cos(yaw)
out=dict(centerline_pose=[x,y,yaw],ego_pose=[first[k] for k in ('x','y','yaw')],
         ego_lateral_m=ego_lateral,candidates=[])
for offset in (-.25,-.30,-.35,-.40):
    plan.LATERAL=offset
    cfg,overlay=plan.expected()
    try:row=dict(offset_m=offset,valid=True,geometry=plan.geometry(cfg,overlay))
    except ValueError as error:row=dict(offset_m=offset,valid=False,error=str(error))
    out['candidates'].append(row)
plan.write(plan.HERE/'placement_probe.json',out)
print(json.dumps(out,ensure_ascii=False))
