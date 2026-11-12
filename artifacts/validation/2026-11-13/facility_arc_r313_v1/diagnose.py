"""정지 입력의 조향 선택 비교. 기존 점유 여유를 유지한 진단이며 구동에 미연결이다."""
from pathlib import Path
import hashlib,json,math,sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig,per_ray_padding
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_path import swept_limit

def arc(angle,steps,wb,center=False):
    k=math.tan(angle)/wb
    heading=k*steps
    x,y=(steps-wb/2,np.zeros_like(steps)) if abs(k)<1e-5 else (np.sin(heading)/k-wb/2,(1-np.cos(heading))/k)
    if center:x,y=x+wb/2*np.cos(heading),y+wb/2*np.sin(heading)
    return np.column_stack((x,y))

def distance_to_path(points,path):
    a,b=path[:-1],path[1:];v=b-a;delta=points[:,None,:]-a
    t=np.clip(np.sum(delta*v,axis=2)/np.maximum(1e-12,np.sum(v*v,axis=1)),0,1)
    return np.min(np.linalg.norm(delta-t[:,:,None]*v,axis=2),axis=1)

def main():
    trace=R/'artifacts/validation/2026-11-13/occluder_pass_drive_r310_v1/run1/follow_decisions.jsonl'
    with trace.open() as f:
        head=json.loads(next(f))['data'];calls=[r['data'] for line in f if (r:=json.loads(line))['kind']=='call']
    cfg=FollowConfig(**head['follow_config']);motion=MotionConfig(**head['motion_config'])
    report=dict(scope='saved input steering geometry only; no drive authorization',trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        held_padding=True,hold_clearance_m=cfg.hold_clearance_m,resume_clearance_m=cfg.resume_clearance_m,
        boundary_agreement_m=cfg.boundary_agreement_m,rows=[])
    fig,axes=plt.subplots(2,3,figsize=(15,9),constrained_layout=True)
    for t,ax in zip((21.2,21.8,22.,24.,26.,30.),axes.flat):
        call=min(calls,key=lambda c:abs(c['input']['reference_s']-t));d,scan,hist=restore_call(call['input'],motion)
        scene=motion_scene(scan,hist,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        points=np.asarray(scene.occupied_endpoints_xy_m)
        path=np.asarray(call['output'][2]['path_points']);desired=call['output'][1]
        margin=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
        options=[]
        for angle in sorted(set(np.linspace(-cfg.max_steering_rad,cfg.max_steering_rad,61).tolist()+[desired,0.])):
            limit=swept_limit(points,angle,cfg.wheelbase_m,horizon=2.5,margin=margin,length=cfg.length_m,width=cfg.width_m)
            prefix=cfg.resume_clearance_m+.04
            error=distance_to_path(arc(angle,np.linspace(0,prefix,33),cfg.wheelbase_m),path)
            options.append(dict(angle_rad=angle,clearance_m=limit,prefix_path_error_max_m=float(max(error)),
                geometry_candidate=bool(limit>=cfg.resume_clearance_m and max(error)<=cfg.boundary_agreement_m)))
        candidates=[o for o in options if o['geometry_candidate']]
        best=min(candidates,key=lambda x:abs(x['angle_rad']-desired)) if candidates else None
        row=dict(time=d['reference_s'],measured_speed_mps=d['speed_mps'],reason=call['output'][2]['reason'],desired_rad=desired,
            previous_rad=d['previous_steering_rad'],best=best,options=options)
        report['rows'].append(row)
        ax.scatter(points[:,0],points[:,1],s=5,c='#333333',label='Raw occupied returns')
        ax.plot(path[:,0],path[:,1],c='#1565c0',lw=2,label='Observed rear-axle path')
        for ang,color,label in [(desired,'#c62828','Current steering'),*([(best['angle_rad'],'#00897b','Nearby clear steering')] if best else [])]:
            xy=arc(ang,np.linspace(0,.6,61),cfg.wheelbase_m,True);ax.plot(*xy.T,c=color,lw=2,label=label)
        ax.add_patch(plt.Rectangle((-cfg.length_m/2,-cfg.width_m/2),cfg.length_m,cfg.width_m,fill=False,lw=2,color='black'))
        ax.set(xlim=(-.3,1.1),ylim=(-.4,1.1),aspect='equal',xlabel='Forward x (m)',ylabel='Left y (m)',title=f'{t:.1f}s / {len(candidates)} geometric candidates')
        ax.grid(alpha=.2)
    axes[0,0].legend(fontsize=7,loc='upper right');fig.savefig(A/'steering_geometry.png',dpi=150);plt.close(fig)
    (A/'diagnosis.json').write_text(json.dumps(report,indent=2)+'\n')
    for row in report['rows']:print(json.dumps({k:v for k,v in row.items() if k!='options'}))

if __name__=='__main__':main()
