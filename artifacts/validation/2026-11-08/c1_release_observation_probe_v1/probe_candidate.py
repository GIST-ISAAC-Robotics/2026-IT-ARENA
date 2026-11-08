"""r35 고정 입력 4개에서 같은-경로 증거 범위 비교. 주행이 아니다."""
from pathlib import Path
import json,importlib.util
import numpy as np
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig,freeze
from arena_autonomy.pass_release_trial import motion_region,lateral_config
from arena_autonomy.same_path_gap_trial import bounded_same_path_gap
A=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('staged_same_path_gap',A/'same_path_gap_trial.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
rows=[]
for sample in json.loads((A/'snapshots.json').read_bytes()):
    h=sample['header'];cfg=FollowConfig(**h['follow_config'])
    i,scan,history=restore_call(sample['input'],MotionConfig(**h['motion_config']))
    scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
    n=sample['output'][2]['normal'];proof=n['partial_observation'];steer=proof['timed_missing_diagnostic']['checked_steering_rad']
    context={};motion_region(scene,i['speed_mps'],.35,steer,i['previous_steering_rad'],i['dt_s'],cfg,lateral_config(cfg),context=context)
    q=context['queries'][~context['free'] & ~context['masked']]
    command=dict(now=i['reference_s'],speed=i['speed_mps'],target=.35,steering=steer,previous=i['previous_steering_rad'])
    args=(scene,scan,q,context['padding'],cfg.length_m,cfg.width_m,proof,command)
    old=bounded_same_path_gap(*args);new=module.bounded_same_path_gap(*args)
    rows.append(dict(t=i['reference_s'],old=old,new=new,scope='fixed scene local proof, not driven motion'))
(A/'candidate_result.json').write_text(json.dumps(freeze(rows),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print([(r['t'],r['old']['reason'],r['new']) for r in rows])
assert sum(r['new']['accepted'] for r in rows)==3
assert all((r['old']['reason']=='unknown_outside_side_scope' and r['new']['accepted'])
    or (r['old']['reason']=='different_or_larger_motion' and r['new']==r['old']) for r in rows)
