"""저장 입력의 실제 거부 원호와 반환점을 분리. 다른 조향은 단일 장면 비교이며 구동 허가 아님."""
from pathlib import Path
import hashlib,json,math
import numpy as np
from arena_autonomy.follow_trace import (MotionConfig,FollowConfig,replay_call,freeze,
    comparable_decision,equal_output,restore_call,motion_scene)
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_path import swept_limit,braking_speed
from arena_autonomy.bypass_entry_trial import entry_motion_region
from arena_autonomy.pass_release_trial import lateral_config,region_blocked,SelectorConfig

A=Path(__file__).resolve().parent;trace=A/'run1/follow_decisions.jsonl'
count=0;different=0;target=None
for line in trace.open():
    event=json.loads(line);d=event['data']
    if event['kind']=='header':
        m=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
        f=BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
    elif event['kind']=='reset':f.reset()
    elif event['kind']=='call':
        out=freeze(replay_call(f,d['input'],m));count+=1
        different+=not equal_output(comparable_decision(out),comparable_decision(d['output']))
        if d['input']['reference_s']>=30.:
            target=d;break
assert target and different==0
i,scan,history=restore_call(target['input'],m)
scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
b=target['output'][2]['bypass'];candidate=b['candidates'][0];wall=b['reference']['walls']['right']
points=np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
guide=f.bypass.guide
padding,detail=guide._padding(scene,cfg,'right',wall,0.)
angle=candidate['steering_rad']
clearance=swept_limit(points,angle,cfg.wheelbase_m,horizon=2.5,margin=padding,length=cfg.length_m,width=cfg.width_m)
assert abs(clearance-candidate['command_clearance_m'])<1e-9
rays=[r for r in scene.rays if r.status=='return']
def hits(steering):
    curvature=math.tan(steering)/cfg.wheelbase_m
    for distance in np.arange(0.,2.501,.04):
        yaw=curvature*distance;c=math.cos(yaw);s=math.sin(yaw)
        rx=distance-cfg.wheelbase_m/2 if abs(curvature)<1e-5 else s/curvature-cfg.wheelbase_m/2
        ry=0. if abs(curvature)<1e-5 else (1-c)/curvature
        dx,dy=points[:,0]-rx,points[:,1]-ry
        xx=c*dx+s*dy-cfg.wheelbase_m/2;yy=-s*dx+c*dy
        ids=np.flatnonzero((abs(xx)<=cfg.length_m/2+padding)&(abs(yy)<=cfg.width_m/2+padding))
        if len(ids):
            return dict(first_interference_distance_m=float(distance),points=[dict(index=int(k),ray=int(rays[k].source_index),xy_m=points[k].tolist(),padding_m=float(padding[k]),age_s=float(scene.t_ref_s-rays[k].acquired_at_s),outer_wall_member=rays[k].source_index in wall['source_ray_indices']) for k in ids])
    return dict(first_interference_distance_m=None,points=[])
angles=sorted(set([angle,i['previous_steering_rad'],0.,-.37,-.30,-.20,-.10,.10,.20,.30,.37]))
samples=[]
for a in angles:
    gap=swept_limit(points,a,cfg.wheelbase_m,horizon=2.5,margin=padding,length=cfg.length_m,width=cfg.width_m)
    target_speed=min(.35,braking_speed(gap,scene.t_ref_s-scene.source_first_s))
    region=entry_motion_region(scene,i['speed_mps'],target_speed,a,a,i['dt_s'],cfg,lateral_config(cfg),padding)
    samples.append(dict(angle_rad=a,clearance_m=gap,braking_cap_mps=braking_speed(gap,scene.t_ref_s-scene.source_first_s),aligned_hypothesis_region=region,aligned_hypothesis_blocked=region_blocked(region,SelectorConfig()),**hits(a)))
result=dict(scope='고정 입력 원호 비교. 조향 변화율/이동 영역/경로 목표를 모두 만족한 새 구동 허가나 폐루프 결과가 아님.',trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),matched_calls=count,mismatches=different,reference_s=scene.t_ref_s,current_command=target['output'][:2],requested_candidate={k:candidate[k] for k in ('steering_rad','command_clearance_m','preview_return_progress_m','preview_point_gap_m','reason')},sweeps=samples)
(A/'blocked_geometry.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k!='sweeps'},ensure_ascii=True));print([(r['angle_rad'],r['clearance_m'],len(r['points'])) for r in samples])
