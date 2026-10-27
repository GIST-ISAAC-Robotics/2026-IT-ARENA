"""실패 입력에서 기존 v2와 표면 대응 v3 대조. 제어 연결/새 주행 없음."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import time
from arena_autonomy.follow_trace import replay_events, replay_call
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.partial_surface_alignment import surface_translation

OUT=Path(__file__).resolve().parent
SOURCE=OUT/'run8_lap/follow_decisions.jsonl'
TARGET=OUT/'run9_metric_probe.json'
if TARGET.exists():raise FileExistsError(TARGET)
def events():
    with SOURCE.open(encoding='utf-8') as stream:
        for line in stream:yield json.loads(line)

baseline=replay_events(events());baseline.pop('outcomes')
if not baseline['passed']:
    with TARGET.open('x',encoding='utf-8') as stream:json.dump({'baseline':baseline},stream,indent=2)
    raise RuntimeError('original G1 replay differs')
controller=motion=None;calls=0;changes=[];reasons=Counter();costs=[];fits={};first_latch=None
for event in events():
    data=event['data']
    if event['kind']=='header':
        controller=LocalFollow(FollowConfig(**data['follow_config']))
        controller.partial_policy=PartialObservationPolicy(allow_narrow_forward=True,surface_translation_enabled=True,lateral_metric_enabled=True)
        motion=MotionConfig(**data['motion_config'])
    elif event['kind']=='reset':controller.reset()
    elif event['kind']=='call':
        start=time.perf_counter();actual=replay_call(controller,data['input'],motion)
        costs.append((time.perf_counter()-start)*1000);calls+=1
        reasons[actual[2]['reason']]+=1
        obs=actual[2].get('partial_observation') or {}
        fit=obs.get('surface_correspondence')
        if fit and not fit.get('cached'):
            fits[str(event['seq'])]={'reference_s':data['input']['reference_s'],**fit}
        if obs.get('latch_time_s') is not None and first_latch is None:
            first_latch={k:obs.get(k) for k in ('latch_time_s','latch_reason','latch_evidence')}
        old=data['output']
        if abs(actual[0]-old[0])>1e-9 or abs(actual[1]-old[1])>1e-9:
            changes.append(dict(seq=event['seq'],reference_s=data['input']['reference_s'],
                previous_speed_mps=old[0],speed_mps=actual[0],previous_reason=old[2]['reason'],
                reason=actual[2]['reason'],partial_reason=obs.get('reason')))
result=dict(scope='saved_decisions_not_closed_loop_v4_offline_only',baseline=baseline,calls=calls,
    changed_commands=len(changes),new_positive_calls=sum(r['previous_speed_mps']==0 and r['speed_mps']>0 for r in changes),
    reasons=dict(reasons),maximum_ms=max(costs),p95_ms=sorted(costs)[int(.95*len(costs))],
    source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (SOURCE,Path(__file__))},
    fits=fits,changes=changes,first_latch=first_latch)
with TARGET.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
print(json.dumps({k:result[k] for k in ('calls','changed_commands','new_positive_calls','reasons','maximum_ms','p95_ms')}))
print(json.dumps({'fits':len(fits),'accepted_fits':sum(f['accepted'] for f in fits.values())}))
