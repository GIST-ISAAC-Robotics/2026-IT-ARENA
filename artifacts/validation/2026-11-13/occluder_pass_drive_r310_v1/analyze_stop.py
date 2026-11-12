"""정지 이유·실제 조향·원 반환 간격 진단. 제어에는 정답을 넣지 않는다."""
from pathlib import Path
import json,sys,math
import numpy as np
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,per_ray_padding
from arena_autonomy.local_path import swept_limit

def hit_points(points,angle,cfg,margin,step):
    k=math.tan(angle)/cfg.wheelbase_m;yaw=k*step;c,s=math.cos(yaw),math.sin(yaw)
    rx,ry=(step-cfg.wheelbase_m/2,0.) if abs(k)<1e-5 else (s/k-cfg.wheelbase_m/2,(1-c)/k)
    dx,dy=points[:,0]-rx,points[:,1]-ry
    xy=np.column_stack((c*dx+s*dy-cfg.wheelbase_m/2,-s*dx+c*dy))
    keep=(np.abs(xy[:,0])<=cfg.length_m/2+margin)&(np.abs(xy[:,1])<=cfg.width_m/2+margin)
    return np.flatnonzero(keep),xy

def main():
    with (A/'run1/follow_decisions.jsonl').open() as f:
        header=json.loads(next(f))['data'];calls=[j['data'] for l in f if (j:=json.loads(l))['kind']=='call']
    cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
    trace=json.loads((A/'run1/trace.json').read_bytes());rows=[]
    for t in (20.9,22.,24.,26.,30.):
        call=min(calls,key=lambda c:abs(c['input']['reference_s']-t))
        d,scan,history=restore_call(call['input'],motion)
        scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        points=np.asarray(scene.occupied_endpoints_xy_m)
        margin=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
        dynamics=min(trace,key=lambda x:abs(x['time']-t))['dynamics']
        row=dict(time=d['reference_s'],input={k:v for k,v in d.items() if isinstance(v,(float,int,str))},
            command=call['output'][:2],reason=call['output'][2].get('reason'),
            details={k:v for k,v in call['output'][2].items() if k in ('swept_clearance_m','target_speed_before_hold_mps','observed_path_ahead_m')},
            actual_steering=[dynamics['left_steering_rad'],dynamics['right_steering_rad']],checks=[])
        for name,angle in [('command',call['output'][1]),('previous',d['previous_steering_rad'])]:
            clearance=swept_limit(points,angle,cfg.wheelbase_m,horizon=3.,margin=margin,length=cfg.length_m,width=cfg.width_m)
            step=0. if clearance==0 else clearance+.04
            ids,xy=hit_points(points,angle,cfg,margin,step)
            row['checks'].append(dict(name=name,angle=angle,clearance=clearance,step=step,
                hits=[dict(point=points[i].tolist(),body_xy=xy[i].tolist(),padding=float(margin[i])) for i in ids]))
        rows.append(row)
    report=json.loads((A/'run1/report.json').read_bytes());fixture=json.loads((A/'fixture_execution.json').read_bytes())
    result=dict(scope='saved sensor / evaluated dynamics only',progress_m=report['progress_m'],peak_ground_speed_mps=report['peak_ground_speed_mps'],
        passed=report['passed'],stop=report['failure_cleanup_stop']['dense_pose_stop_passed'],shutdown_clean=report['shutdown_clean'],
        source_unchanged=not fixture['changed_inputs'],input_count=fixture['input_count'],collision_samples=report['collision_samples'],
        physical_contact_messages=sum(x['messages'] for x in fixture['contacts']),rows=rows)
    (A/'stop_analysis.json').write_text(json.dumps(freeze(result),indent=2)+'\n')
    print(json.dumps(freeze(result)))

if __name__=='__main__':main()
