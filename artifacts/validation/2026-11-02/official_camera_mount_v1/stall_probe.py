"""선두 정지 입력의 조향·현재 반환 대조. 저장 자세 이동/주행 성공을 합성하지 않는다."""
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.local_path import swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding

header=call=None
for line in (HERE/'run1/follow_decisions.jsonl').open():
    event=json.loads(line)
    if event['kind']=='header': header=event['data']
    if event['kind']=='call' and event['data']['output'][2].get('reason')=='follow_gap_hold':
        call=event['data']
        if event['data']['input']['reference_s']>20: break
cfg=FollowConfig(**header['follow_config'])
data,scan,history=restore_call(call['input'],MotionConfig(**header['motion_config']))
scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                   epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
points=np.asarray(scene.occupied_endpoints_xy_m)
padding=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
path=np.asarray(call['output'][2]['path_points'])
age=scene.t_ref_s-scene.source_first_s
cap,desired,remaining=pursue_path(path,data['speed_mps'],cfg,age)
rows=[]
for angle in (0.,-.06,-.12,-.18,desired):
    clearance=swept_limit(points,angle,cfg.wheelbase_m,horizon=2.5,margin=padding,length=cfg.length_m,width=cfg.width_m)
    rows.append(dict(steering_rad=angle,clearance_m=clearance))
near=points[(points[:,0]>-.2)&(points[:,0]<.8)&(np.abs(points[:,1])<.3)]
result=dict(reference_s=data['reference_s'],config=asdict(cfg),desired_rad=desired,path_cap_mps=cap,
            path_remaining_m=remaining,constant_steering_sweeps=rows,near_points_m=near.tolist(),
            scope='현재 저장 관측에서 조향별 기하만 대조. 실제 조향/가속/주행/독립 보호 통과 아님.')
(HERE/'stall_probe.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
fig,ax=plt.subplots(figsize=(10,5))
ax.scatter(points[:,0],points[:,1],s=12,label='current returns')
ax.plot(path[:,0],path[:,1],label='accepted path')
ax.add_patch(plt.Rectangle((-.1,-.075),.2,.15,fill=False,color='black',linewidth=2))
for row in rows:
    a=row['steering_rad']; k=math.tan(a)/cfg.wheelbase_m; s=np.linspace(0,.7,100)
    x=s if abs(k)<1e-8 else np.sin(k*s)/k
    y=np.zeros_like(s) if abs(k)<1e-8 else (1-np.cos(k*s))/k
    ax.plot(x,y,label=f'{a:.3f}rad: {row["clearance_m"]:.2f}m')
ax.set(xlim=(-.25,1),ylim=(-.55,.55),xlabel='forward m',ylabel='left m')
ax.set_aspect('equal');ax.legend(fontsize=8);ax.grid();fig.tight_layout()
fig.savefig(HERE/'stall_probe.png',dpi=150)
print(json.dumps({k:v for k,v in result.items() if k not in ('config','near_points_m')},ensure_ascii=False))
