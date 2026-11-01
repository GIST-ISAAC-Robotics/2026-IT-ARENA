"""오래 감속한 장면 하나: 현재 조향 원호, 당시 관측 경로와 사후 실제 궤적."""
import json
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.occupancy_timing import per_ray_padding
from sensor_turn_probe import rollout

base=Path(__file__).resolve().parent;run=base/'solo_2p0'
stamp=json.loads((base/'preview_counterexamples.json').read_text())['reference_s']
with (run/'follow_decisions.jsonl').open() as f:
    h=json.loads(next(f))['data']
    event=next(e['data'] for e in map(json.loads,f) if e['kind']=='call' and e['data']['input']['reference_s']==stamp)
a,scan,history=restore_call(event['input'],MotionConfig(**h['motion_config']))
sc=motion_scene(scan,history,stamp,timing_verified=a['timing_verified'],epoch=a['epoch'],
                rear_half_angle_deg=a['rear_half_angle_deg'])
v,angle,d=event['output'];arc=d['swept_clearance_m'];span=arc+.20
points=np.asarray(sc.occupied_endpoints_xy_m);path=np.asarray(d['path_points']);cfg=h['follow_config']
with (run/'ego_poses.jsonl').open() as f:
    truth=np.array([q for q in map(json.loads,f) if stamp<=q[0]<=stamp+3.])
xy=truth[:,1:3]-truth[0,1:3];co,si=np.cos(truth[0,3]),np.sin(truth[0,3])
body=xy@np.array([[co,-si],[si,co]]);s=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(body,axis=0),axis=1))]
L=cfg['wheelbase_m']
font_manager.fontManager.addfont('/mnt/c/Windows/Fonts/malgun.ttf')
plt.rcParams.update({'font.family':'Malgun Gothic','axes.unicode_minus':False})
fig,ax=plt.subplots(figsize=(8.2,4.9))
ax.scatter(points[:,0],points[:,1],s=12,c='#405365',label='당시 라이다 반환점',zorder=1)
ax.plot(path[:,0],path[:,1],color='#6a57a7',ls=':',lw=2,label='당시 벽에서 얻은 경로')
for st,label in [(angle,'발행 조향 유지'),(a['previous_steering_rad'],'직전 조향 유지')]:
    # 두 원호가 같다면 둘 중 하나만 보여 준다.
    if label.startswith('직전') and abs(angle-st)<1e-6:continue
    samples=np.arange(0.,arc+.001,.01);k=math.tan(st)/L;yaw=k*samples
    rx=samples if abs(k)<1e-9 else np.sin(yaw)/k
    ry=np.zeros_like(samples) if abs(k)<1e-9 else (1-np.cos(yaw))/k
    ax.plot(rx-L/2+L/2*np.cos(yaw),ry+L/2*np.sin(yaw),'--',lw=2,
            color=('#d66338' if label.startswith('발행') else '#c99c38'),label=label)
pad=per_ray_padding(sc,.025+cfg['measurement_padding_m'],cfg['object_motion_bound_mps'])
touched=np.zeros(len(points),dtype=bool)
for st in (angle,a['previous_steering_rad']):
    k=math.tan(st)/L;distance=arc+.04;theta=k*distance
    rx=(math.sin(theta)/k if abs(k)>1e-8 else distance)-L/2
    ry=(1-math.cos(theta))/k if abs(k)>1e-8 else 0.
    dx,dy=points[:,0]-rx,points[:,1]-ry;c,s=math.cos(theta),math.sin(theta)
    touched|=(abs(c*dx+s*dy-L/2)<=cfg['length_m']/2+pad)&(abs(-s*dx+c*dy)<=cfg['width_m']/2+pad)
if np.any(touched):ax.scatter(*points[touched].T,marker='x',s=44,c='#be441b',label='고정 원호의 여유에 걸린 점')
ax.plot(body[s<=span,0],body[s<=span,1],color='#08735d',lw=3,label='실제로 지나간 차체 중심 (평가용)')
pose=rollout(path,span,angle,a['speed_mps'],cfg,max(v,(span-cfg['standstill_gap_m'])/cfg['headway_s']),
             stamp-sc.source_first_s)
if pose is not None:ax.plot(pose[:,0],pose[:,1],color='#3086ba',lw=1.7,label='당시 관측만으로 예측한 차체 중심')
ax.add_patch(plt.Rectangle((-.10,-.075),.2,.15,color='#405365',alpha=.35))
ax.set(xlabel='차체 기준 전방 (m)',ylabel='왼쪽 (m)',xlim=(-.15,1.6),ylim=(-.65,.95),aspect='equal')
ax.set_title('단독 코너의 한 스캔: 조향을 고정하면 실제 주행과 경로가 달라집니다')
ax.legend(ncol=2,loc='upper center',bbox_to_anchor=(.5,-.20),frameon=False,fontsize=9)
ax.grid(alpha=.20);fig.tight_layout()
output=base/'turn_example_with_hit.png'
if output.exists():raise FileExistsError(output)
fig.savefig(output,dpi=145,bbox_inches='tight')
