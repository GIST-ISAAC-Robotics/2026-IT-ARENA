"""원 r181 실제 현재 입력/반환을 공통 선택 함수로 전부 재현한다."""
import json
from pathlib import Path
from arena_autonomy import guard_observation_policy as policy
from arena_autonomy.follow_trace import freeze, equal_output, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
A=Path(__file__).resolve().parent
results={}
for name in ('guard_approach_runtime_r182_v1','expiry_route_runtime_r187_v1'):
    trace=A.parent/name/'two_actor_run1/run1/follow_decisions.jsonl'
    calls=0;mismatches=[]
    with trace.open() as stream:
        header=json.loads(next(stream))['data']
        cfg=FollowConfig(**{**header['follow_config'],'guard_observation_policy':'selected_r181'})
        motion=MotionConfig(**header['motion_config'])
        for event in map(json.loads,stream):
            if event['kind']!='call' or 'current_rebase' not in event['data']:continue
            d=event['data'];now=d['current_rebase']['input']
            _,scan,history=restore_call(now,motion)
            scene=motion_scene(scan,history,now['reference_s'],timing_verified=now['timing_verified'],
                               epoch=now['epoch'],rear_half_angle_deg=now['rear_half_angle_deg'])
            result=policy.rebase(d['output'],d['input'],now,scene,history,cfg);calls+=1
            if not equal_output(freeze(result),d['current_rebase']['output']):mismatches.append(event['seq'])
    results[name]=dict(calls=calls,mismatches=mismatches)
with (A/'rebase_records.json').open('x') as stream:json.dump(results,stream,indent=2);stream.write('\n')
print(json.dumps(results));assert all(not value['mismatches'] for value in results.values())
