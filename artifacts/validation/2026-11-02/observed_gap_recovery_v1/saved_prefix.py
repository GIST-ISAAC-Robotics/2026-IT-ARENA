"""현재 운영 시험과 이미 검증된 교통 옵션의 저장 입력 비교. 최초 제어 차이까지만 해석."""
import json
from pathlib import Path
import sys
from dataclasses import replace

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig

with (HERE.parent/'official_race_distance_v2/run1/follow_decisions.jsonl').open() as f:
    traffic=json.loads(next(f))['data']
results={}
for mode in ('current_recovery',):
    calls=0; first=None; exact=True
    for line in (HERE.parent/'standing_steering_v1/run1/follow_decisions.jsonl').open():
        event=json.loads(line);data=event['data']
        if event['kind']=='header':
            cfg=FollowConfig(**data['follow_config']);motion=MotionConfig(**data['motion_config'])
            if mode=='selected_traffic':
                cfg=replace(cfg,released_rear_sweep_trial=True,static_side_release_trial=True)
            controller=(BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping(traffic['execution_config']),allow_simulation_trial=True)
                        if mode=='selected_traffic' else LocalFollow(cfg))
        elif event['kind']=='reset':controller.reset()
        elif event['kind']=='call':
            actual=freeze(replay_call(controller,data['input'],motion));expected=data['output'];calls+=1
            exact &= equal_output(comparable_decision(actual),comparable_decision(expected))
            if not equal_output(actual[:2],expected[:2]):
                first=dict(call=calls,reference_s=data['input']['reference_s'],expected=expected,actual=actual)
                break
    results[mode]=dict(calls_before_or_at_difference=calls,full_detail_match=exact,first_control_difference=first)
(HERE/'saved_prefix.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({mode:dict(calls=row['calls_before_or_at_difference'],exact=row['full_detail_match'],
    first=None if row['first_control_difference'] is None else
        {k:row['first_control_difference'][k] for k in ('call','reference_s')},
    controls=None if row['first_control_difference'] is None else
        dict(expected=row['first_control_difference']['expected'][:2],actual=row['first_control_difference']['actual'][:2]))
    for mode,row in results.items()},ensure_ascii=False))
