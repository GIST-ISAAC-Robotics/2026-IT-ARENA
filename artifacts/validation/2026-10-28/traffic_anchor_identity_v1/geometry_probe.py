"""원래 위협점 주변 실제 반환/짧은 현을 읽기 전용으로 대조한다."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.partial_observation import short_observed_surface
from arena_autonomy.partial_surface_alignment import surface_translation

BASE=Path(__file__).resolve().parent
path=BASE/'target_frames.json';before=hashlib.sha256(path.read_bytes()).hexdigest()
saved=json.loads(path.read_text(encoding='utf-8'));motion=MotionConfig(**saved['header']['motion_config'])
frames=saved['frames']
latched=next(r for r in frames if abs(r['data']['input']['reference_s']-129.88)<1e-8)
e=latched['data']['output'][2]['partial_observation']['latch_evidence']
old=np.asarray(e['correspondence_context']['previous_points_at_reference_xy'])
oldrow=next(r for r in frames if abs(r['data']['input']['reference_s']-e['previous_reference_s'])<1e-8)
olddata,oldraw,oldh=restore_call(oldrow['data']['input'],motion)
data,raw,h=restore_call(latched['data']['input'],motion)
carried=transport_points(old,h,olddata['reference_s'],data['reference_s'])
ret=np.asarray(e['witnesses'][0]['previous_point_now_xy'])
index=int(np.argmin(np.linalg.norm(carried-ret,axis=1)))

def projected(raw,h,t):
    source=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
    p,ids,_=h.project(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,
        source,raw.time_increment,t,mode='both',timing_verified=True)
    return p,ids,source

def neighborhood(point,p,ids):
    k=int(np.argmin(np.linalg.norm(p-point,axis=1)))
    lo=max(0,k-3);hi=min(len(p),k+4)
    candidates=[]
    for i in range(lo,hi-1):
        v=p[i+1]-p[i];length=float(np.linalg.norm(v))
        f=float((point-p[i])@v/max(length**2,1e-12))
        candidates.append(dict(ids=ids[i:i+2].tolist(),length_m=length,fraction=f,
            normal_distance_m=float(np.linalg.norm(point-p[i]-f*v)),adjacent=bool(ids[i+1]-ids[i]==1)))
    return dict(nearest_id=int(ids[k]),nearest_distance_m=float(np.linalg.norm(p[k]-point)),
        ids=ids[lo:hi].tolist(),points=p[lo:hi].tolist(),segments=candidates,
        short_surface=short_observed_surface(point,p,ids))

p,ids,_=projected(oldraw,oldh,olddata['reference_s'])
origin=neighborhood(old[index],p,ids)
rows=[];plots=[];last=None
for row in frames:
    data,raw,h=restore_call(row['data']['input'],motion);t=data['reference_s']
    p,ids,source=projected(raw,h,t)
    if t<129.88-1e-9 or source==last:continue
    last=source;moved=transport_points(old,h,olddata['reference_s'],t);point=moved[index]
    fit=surface_translation(moved,p,ids)
    # 거절된 보정도 진단용으로만 비교한다. 운영 채택하지 않는다.
    shift=np.sum([s['step_m'] for s in fit.get('steps',[])],axis=0) if fit.get('steps') else np.zeros(2)
    rows.append(dict(t_s=t,source_s=source,fit_reason=fit['reason'],fit_accepted=fit['accepted'],
        retained_xy=point.tolist(),uncorrected=neighborhood(point,p,ids),
        proposed_shift_m=np.asarray(shift).tolist(),proposed=neighborhood(point+shift,p,ids),
        input_speed=data.get('speed'),input_keys=sorted(data.keys())))
    if len(plots)<3:plots.append((t,moved,p,point,point+shift))
report=dict(scope='diagnostic only, rejected fits are not used by controller',input_sha256=before,
    input_unchanged=before==hashlib.sha256(path.read_bytes()).hexdigest(),original_point_xy=old[index].tolist(),
    original=origin,rows=rows)
with (BASE/'geometry_comparison.json').open('x',encoding='utf-8') as f:json.dump(freeze(report),f,ensure_ascii=False,indent=2)
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
fig,axes=plt.subplots(1,len(plots),figsize=(15,5),layout='constrained')
for ax,(t,oldnow,current,point,shifted) in zip(axes,plots):
    ax.scatter(oldnow[:,0],oldnow[:,1],s=7,c='gray',alpha=.55,label='old scan transported')
    ax.scatter(current[:,0],current[:,1],s=10,c='#0068aa',label='current returns')
    ax.scatter(*point,s=100,marker='x',c='#dd2200',linewidths=2,label='retained point')
    ax.scatter(*shifted,s=70,marker='+',c='#992299',label='proposed fit (diagnostic)')
    ax.set(xlim=(point[0]-.15,point[0]+.15),ylim=(point[1]-.15,point[1]+.15),
        title=f'{t:.2f} s',xlabel='body x (m)',ylabel='body y (m)');ax.set_aspect('equal');ax.grid(alpha=.2)
axes[0].legend(fontsize=7)
fig.savefig(BASE/'local_correspondence.png',dpi=160)
print(json.dumps(dict(original=origin,rows=[dict(t_s=r['t_s'],fit=r['fit_reason'],
    before=r['uncorrected']['nearest_distance_m'],after=r['proposed']['nearest_distance_m'],
    short=r['proposed']['short_surface'] is not None) for r in rows[:4]])))
