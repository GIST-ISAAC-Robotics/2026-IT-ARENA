"""원 실행의 명령 시각/보호층 정지를 분리한다. 재실행/판정 변경 없음."""
import bisect
from collections import Counter
import json
from pathlib import Path
A=Path(__file__).resolve().parent;D=A/'two_actor_run1'
def load(name):
    with (D/name).open() as f:return [json.loads(s) for s in f]
statuses=load('full_status.jsonl');times=[s['sim_time_s'] for s in statuses]
safety=load('full_safety.jsonl');events=load('run1/follow_decisions.jsonl')
calls=[e['data'] for e in events if e['kind']=='call']
groups=Counter();firsts={}
for row in safety:
    if not row.get('command_stale'):continue
    i=bisect.bisect_right(times,row['reference_s'])-1
    if i<0:continue
    status=statuses[i]
    key='positive' if status.get('speed_command_mps',0)>0 else 'zero'
    groups[key]+=1
    if key not in firsts:
        firsts[key]=dict(guard_time_s=row['reference_s'],last_status_s=status['sim_time_s'],
            requested_speed_mps=status['speed_command_mps'],delivery=status.get('decision_delivery'),
            current=status.get('current_rebase'),guard=row)
zero=Counter();nonzero=0
for c in calls:
    output=c.get('current_rebase',{}).get('output')
    if output is not None:
        speed=output['speed_mps'];reason=output['reason']
    else:
        speed=c['output'][0];reason=c['output'][2].get('reason','unknown')
    if speed<=0:zero[reason]+=1
    else:nonzero+=1
contacts=load('run1/actor_contacts.jsonl')
result=dict(stale_samples_by_latest_status_speed=dict(groups),first_stale_examples=firsts,
    final_candidate_zero_reasons=dict(zero),final_positive_candidates=nonzero,
    first_actor_contact=contacts[0] if contacts else None,
    source_note='status is a periodic publication sample; use it to classify latest seen command, not every guard input byte',
    limitation='timing association only; not causal isolation of every low-speed segment or every collision')
(A/'diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('first_stale_examples','first_actor_contact')}))
print(json.dumps(dict(first_positive=firsts.get('positive'),first_contact=result['first_actor_contact'])))
