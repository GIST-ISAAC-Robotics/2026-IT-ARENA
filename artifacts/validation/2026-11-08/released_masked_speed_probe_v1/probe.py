"""r37 정체의 같은 측정 속도/조향에서 낮은 목표만 비교. 실제 주행 아님."""
from pathlib import Path
import hashlib,json
import numpy as np
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig,freeze,equal_output
from arena_autonomy.local_follow import pursue_path
from arena_autonomy.pass_release_trial import motion_region,lateral_config,region_blocked,SelectorConfig

A=Path(__file__).resolve().parent
trace=A.parent/'c1_query_side_scope_v1/B_c1_injection/run1/follow_decisions.jsonl'
targets=[33.06,40.02,65.0];rows=[];snapshots=[]
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':header=d;cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
    if e['kind']!='call' or not targets:continue
    now=d['input']['reference_s']
    if now+1e-8<targets[0]:continue
    n=d['output'][2]['normal']
    if n.get('selector_mode')!='RELEASED' or n.get('retained_passage'):continue
    targets.pop(0)
    i,scan,history=restore_call(d['input'],mc)
    scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
    age=now-scene.source_first_s
    cap,desired,remaining=pursue_path(np.asarray(n['path_points']),i['speed_mps'],cfg,age)
    steer=float(np.clip(desired,i['previous_steering_rad']-3*i['dt_s'],i['previous_steering_rad']+3*i['dt_s']))
    target=n['target_speed_before_hold_mps']
    region=motion_region(scene,i['speed_mps'],target,steer,i['previous_steering_rad'],i['dt_s'],cfg,lateral_config(cfg))
    assert equal_output(freeze(region),n['selector_region']),(now,region,n['selector_region'])
    reduced=motion_region(scene,i['speed_mps'],.35,steer,i['previous_steering_rad'],i['dt_s'],cfg,lateral_config(cfg))
    alternatives=[]
    for trial in (.35,.25,.15,.10):
        r=motion_region(scene,i['speed_mps'],trial,steer,i['previous_steering_rad'],i['dt_s'],cfg,lateral_config(cfg))
        alternatives.append(dict(target_mps=trial,region=r,blocked=region_blocked(r,SelectorConfig())))
    rows.append(dict(t=now,measured_mps=i['speed_mps'],target_mps=target,steering_rad=steer,region_exact=True,
        original=region,alternatives=alternatives))
    snapshots.append(dict(header=header,input=d['input'],output=d['output'],baseline_steering_rad=steer,baseline_target_mps=target))
assert len(rows)==3
(A/'result.json').write_text(json.dumps(freeze(dict(scope='exact recorded region then target-only counterfactual; no drive',trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),rows=rows)),indent=2)+'\n',encoding='utf8')
(A/'snapshots.json').write_text(json.dumps(freeze(snapshots))+'\n',encoding='utf8')
print(json.dumps(rows))
