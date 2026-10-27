"""원본 판단 재생과 선택형 G1 비교. 새 경로를 따라 움직인 결과가 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
from arena_autonomy.follow_trace import replay_events,replay_call
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig

OUT=Path(__file__).resolve().parent
ROOT=OUT.parents[3]
SOURCE=ROOT/'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl'
def events():
    with SOURCE.open(encoding='utf-8') as stream:
        for line in stream:yield json.loads(line)
digest=hashlib.sha256(SOURCE.read_bytes()).hexdigest()
baseline=replay_events(events())
baseline.pop('outcomes')
with (OUT/'run4_strict_replay.json').open('x',encoding='utf-8') as stream:json.dump(baseline,stream,indent=2)
if not baseline['passed']:raise RuntimeError('original strict replay differs')
controller=motion=None;calls=0;changes=[];reasons=Counter();costs=[]
for event in events():
    data=event['data']
    if event['kind']=='header':
        assert data['schema']==1 and not data['follow_config'].get('recent_observation_enabled',False)
        cfg={**data['follow_config'],'partial_observation_policy':'bounded_local_v2'}
        controller=LocalFollow(FollowConfig(**cfg));motion=MotionConfig(**data['motion_config'])
    elif event['kind']=='reset':controller.reset()
    elif event['kind']=='call':
        started=time.perf_counter();actual=replay_call(controller,data['input'],motion)
        costs.append((time.perf_counter()-started)*1000);calls+=1
        reasons[actual[2]['reason']]+=1
        old=data['output']
        if abs(actual[0]-old[0])>1e-9 or abs(actual[1]-old[1])>1e-9:
            obs=actual[2].get('partial_observation') or {}
            changes.append(dict(seq=event['seq'],reference_s=data['input']['reference_s'],
                previous_speed_mps=old[0],speed_mps=actual[0],previous_reason=old[2]['reason'],
                reason=actual[2]['reason'],partial_reason=obs.get('reason'),
                observation_allowed=obs.get('allowed'),unknown_filled=obs.get('unknown_filled')))
result=dict(scope='saved_G1_sensor_decisions_not_closed_loop',strict_replay=baseline['passed'],
    calls=calls,changed_commands=len(changes),new_positive_calls=sum(r['previous_speed_mps']==0 and r['speed_mps']>0 for r in changes),
    reasons=dict(reasons),maximum_ms=max(costs),p95_ms=sorted(costs)[int(.95*len(costs))],
    input_sha256=digest,input_unchanged=digest==hashlib.sha256(SOURCE.read_bytes()).hexdigest(),changes=changes)
with (OUT/'run4_candidate_replay.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='changes'}))
