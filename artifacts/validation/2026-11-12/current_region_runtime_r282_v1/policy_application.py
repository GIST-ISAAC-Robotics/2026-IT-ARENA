"""실행 기록의 실제 현재 영역 후보 사용/거절을 집계한다."""
from collections import Counter
import json
from pathlib import Path
A=Path(__file__).resolve().parent
counts=Counter();reasons=Counter();examples={};total=0
with (A/'two_actor_run1/run1/follow_decisions.jsonl').open() as stream:
    for line in stream:
        event=json.loads(line)
        if event['kind']!='call':continue
        c=event['data'];current=c.get('current_rebase')
        if not current:continue
        total+=1;out=current['output'];reasons[out['reason']]+=1
        proof=out.get('current_route_proof') or {};risk=proof.get('bounded_current_risk') or {}
        if risk.get('accepted'):
            counts['current_low_gap_accepted']+=1
            counts['positive_after_low_gap']+=out['speed_mps']>0
            counts['unknown_retained']+=proof['region']['unobserved']>0
        if out['speed_mps']==0 and out['reason'] not in examples:
            examples[out['reason']]=dict(reference_s=current['input']['reference_s'],measured_speed_mps=current['input']['speed_mps'],plan_speed_mps=c['output'][0],output=out)
result=dict(current_checks=total,counts=dict(counts),reasons=dict(reasons),first_zero_by_reason=examples,
    scope='computed current outputs; positive result not sufficient to prove applied drive or successful vehicle motion')
(A/'policy_application.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='first_zero_by_reason'}))
