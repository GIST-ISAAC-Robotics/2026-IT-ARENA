"""최고 명령이 난 실제 스캔 한 장에서 더 먼 '실제 반환'이 있는지 확인한다. 주행 변경 아님."""
import json
import math
from pathlib import Path

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig,pursue_path
from arena_autonomy.local_path import local_path


base=Path(__file__).resolve().parent
diag=json.loads((base/'ten_kmh_diagnostics.json').read_text())['highest_request']
cap=10/3.6
with (base/'solo_2p0/follow_decisions.jsonl').open() as f:
    head=json.loads(next(f))['data']; cfg=FollowConfig(**head['follow_config'])
    motion=MotionConfig(**head['motion_config'])
    raw=next(row['data']['input'] for row in map(json.loads,f) if row['kind']=='call' and
             abs(row['data']['input']['reference_s']-diag['reference_s'])<1e-7)
arg,scan,history=restore_call(raw,motion)
scene=motion_scene(scan,history,arg['reference_s'],timing_verified=arg['timing_verified'],
                   epoch=arg['epoch'],rear_half_angle_deg=arg['rear_half_angle_deg'])
rows=[]
for extra in (0.,.4,.8):
    span=diag['observed_search_horizon_m']+extra
    p,meta=local_path(scene.occupied_endpoints_xy_m,diag['path_wall'],cfg.wall_offset_m,horizon=span)
    row=dict(search_budget_m=span,fit_status=meta['reason'],measured_boundary_ahead_m=meta.get('boundary_observed_ahead_m'))
    if p is not None:
        speed,angle,length=pursue_path(p,arg['speed_mps'],cfg,scene.t_ref_s-scene.source_first_s,speed_cap=cap)
        row.update(path_length_m=length,path_cap_mps=speed,desired_angle_deg=math.degrees(angle))
    rows.append(row)
result=dict(reference_s=diag['reference_s'],scope='같은 직선 스캔의 경로 적합만 비교. 실제 주행 효과/장애물 통과 허가 아님.',rows=rows)
with (base/'straight_range_probe.json').open('x') as f:json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps(result,ensure_ascii=False))
