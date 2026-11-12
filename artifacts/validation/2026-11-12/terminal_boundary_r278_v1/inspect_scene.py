"""r274 정체 원 관측의 양측 경로를 시각화. 제어/정답 입력/조건 변경 없음."""
from pathlib import Path
import json,math,sys
import numpy as np
A=Path(__file__).resolve().parent;R=A.parents[3];B=A.parent/'parallel_ros_runtime_r274_v1'
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.fragmented_wall_hypothesis import propose
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

with (B/'two_actor_run1/run1/follow_decisions.jsonl').open() as f:header=json.loads(next(f))['data']
call=json.loads((B/'terminal_boundary_example.json').read_bytes())
cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
d,scan,history=restore_call(call['input'],motion)
scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
points=np.asarray(scene.occupied_endpoints_xy_m)
rays=[r.source_index for r in scene.rays if r.status=='return']
age=scene.t_ref_s-scene.source_first_s;speed=cfg.open_route_speed_mps
horizon=max(1.8,.06+cfg.standstill_gap_m+speed*(age+.10)+speed**2/4)
radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad)
results={};fig,ax=plt.subplots(figsize=(11,7))
ax.scatter(points[:,0],points[:,1],s=8,c='#536b87',label='Original occupied returns')
for side,color in [('left','#c82b32'),('right','#188f42')]:
    path,info=local_path(points,side,cfg.wall_offset_m,horizon=horizon,
        rays=rays if cfg.ray_wall_continuity else None,piecewise_corner=cfg.piecewise_wall_trial,
        minimum_radius=radius,narrow_occluder_bridge=cfg.narrow_occluder_bridge)
    hypothesis,evidence=propose(points,side,cfg.wall_offset_m,radius)
    results[side]=dict(info=info,fragmented=evidence,fragment_path=None if hypothesis is None else hypothesis.tolist())
    if path is not None:ax.plot(path[:,0],path[:,1],color=color,label=side+' observed offset path')
    if hypothesis is not None:ax.plot(hypothesis[:,0],hypothesis[:,1],'--',color=color,label=side+' fragment candidate')
    ax.add_patch(plt.Rectangle((-.35,(cfg.wall_offset_m if side=='left' else -cfg.wall_offset_m)-.08),.7,.16,
                              fill=False,edgecolor=color,linestyle=':'))
ax.add_patch(plt.Rectangle((-.10,-.075),.2,.15,color='black',alpha=.5))
ax.arrow(0,0,.35,0,width=.008,color='black')
ax.set(xlim=(-.7,2.4),ylim=(-1.6,1.6),xlabel='Forward x (m)',ylabel='Left y (m)',
       title=f'r274 original sensor view at {scene.t_ref_s:.3f}s; no ground-truth input')
ax.set_aspect('equal');ax.grid(alpha=.2);ax.legend(loc='upper right');fig.tight_layout()
fig.savefig(A/'terminal_sensor_view.png',dpi=160)
result=dict(reference_s=scene.t_ref_s,speed_mps=d['speed_mps'],horizon_m=horizon,wall_offset_m=cfg.wall_offset_m,
            boundary_min_ahead_m=cfg.boundary_min_ahead_m,points=len(points),sides=results)
(A/'inspection.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
