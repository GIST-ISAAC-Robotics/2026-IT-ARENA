import copy
from collections import Counter
import hashlib
import json
from pathlib import Path
from arena_autonomy.follow_trace import restore_call, freeze, equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import guard_observation_policy as baseline
import candidate_policy as candidate

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
trace=ROOT/'artifacts/validation/2026-11-12/traffic_solo_gap_runtime_r225_v1/two_actor_run1/run1/follow_decisions.jsonl'
answers=json.loads((ROOT/'artifacts/validation/2026-11-12/projection_kernel_r257_v1/jetson/baseline/answers.json').read_bytes())
outputs={row[0]:row[1] for row in answers}
same=Counter();changed=Counter();examples={};counts=Counter()
with trace.open() as f:
    header=json.loads(next(f))['data'];motion=MotionConfig(**header['motion_config']);cfg=FollowConfig(**header['follow_config'])
    for e in map(json.loads,f):
        d=e['data']
        if e['kind']!='call' or not d.get('current_rebase'):continue
        out=outputs[e['seq']];old=d['input'];now=d['current_rebase']['input']
        _,scan,history=restore_call(now,motion)
        scene=motion_scene(scan,history,now['reference_s'],timing_verified=now['timing_verified'],
                           epoch=now['epoch'],rear_half_angle_deg=now['rear_half_angle_deg'])
        a=baseline.rebase(out,old,now,scene,history,cfg);b=candidate.rebase(out,old,now,scene,history,cfg)
        same['calls']+=1;same['full_differences']+=not equal_output(freeze(a),freeze(b))
        if same['calls']%8:continue
        for delta in (-.04,.04):
            current=copy.deepcopy(now);current['previous_steering_rad']+=delta
            original=baseline.rebase(out,old,current,scene,history,cfg)
            result=candidate.rebase(out,old,current,scene,history,cfg)
            counts['changed_requests']+=1
            changed[result['reason']]+=1
            counts['positive_current_candidates']+=result['speed_mps']>0
            assert original['speed_mps']==0
            assert result['path_source_s']==out[2]['normal']['path_source_s']
            assert abs(result['steering_rad']-current['previous_steering_rad'])<=3*current['dt_s']+1e-12
            assert result['speed_mps']<=out[0]
            if result['reason'] not in examples:
                examples[result['reason']]=dict(seq=e['seq'],delta=delta,speed_mps=result['speed_mps'],
                                               steering_rad=result['steering_rad'],scope=candidate.common.scope(out))
report=dict(unchanged_request=dict(same),sensitivity=dict(counts),reasons=dict(changed),examples=examples,
            trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
            limitation='stored-input changed steering sensitivity; not overlapping runtime or closed-loop driving')
(A/'replay_report.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
assert same['full_differences']==0
