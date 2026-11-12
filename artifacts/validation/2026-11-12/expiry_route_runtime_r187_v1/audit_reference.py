"""실제 판단 기록에서 최근 참조 유지 세대와 폐기/STOP 경계를 사후 감사한다."""
import json
from collections import Counter
from pathlib import Path
A=Path(__file__).resolve().parent
trace=A/'two_actor_run1/run1/follow_decisions.jsonl'
generations={};latest_reset=None;failures=[];uses=Counter();calls=0
for event in map(json.loads,trace.open()):
    d=event['data']
    if event['kind']=='reset':latest_reset=d.get('reason')
    if event['kind']!='call':continue
    calls+=1
    computation=d.get('compute',{})
    generation=computation.get('generation')
    retained=computation.get('expiry_reference_retained')
    if type(retained) is not bool:
        failures.append(dict(seq=event['seq'],reason='missing_retention_metadata'))
    if generation not in generations:
        generations[generation]=dict(first_seq=event['seq'],reset_reason=latest_reset,retained=retained)
        if retained and latest_reset!='decision_source_expired':
            failures.append(dict(seq=event['seq'],reason='retained_outside_source_expiry',latest_reset=latest_reset))
    elif generations[generation]['retained']!=retained:
        failures.append(dict(seq=event['seq'],reason='retention_flag_changed_inside_generation'))
    normal=d['output'][2].get('normal',{})
    if normal.get('boundary_selection')=='junction_recent_path_agreement':
        uses['current_observation_recent_route_agreement']+=1
    if normal.get('expiry_route_switch_rejected'):
        uses['divergent_wall_rejected']+=1
result=dict(passed=bool(calls) and not failures,calls=calls,generations=generations,
    retained_generations=sum(bool(v['retained']) for v in generations.values()),
    current_observation_uses=dict(uses),failures=failures,
    scope='metadata audit; copied geometric reference is not permission to drive or proof of road compliance')
with (A/'reference_audit.json').open('x') as stream:json.dump(result,stream,indent=2);stream.write('\n')
print(json.dumps({k:v for k,v in result.items() if k!='generations'}))
raise SystemExit(0 if result['passed'] else 1)
