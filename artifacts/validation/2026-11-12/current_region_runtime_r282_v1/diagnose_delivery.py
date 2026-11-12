"""신규 후보/폐기를 구분하고 주기 상태로 만료·마지막 정체를 분해한다."""
import bisect,json
from collections import Counter
from pathlib import Path
A=Path(__file__).resolve().parent;D=A/'two_actor_run1'
def rows(name):
    with (D/name).open() as f:
        for line in f:yield json.loads(line)
statuses=[dict(t=r['sim_time_s'],speed=r.get('speed_command_mps',0.),state=r.get('state'),
               reason=r.get('reason'),normal=r.get('normal',{})) for r in rows('full_status.jsonl')]
times=[r['t'] for r in statuses];groups=Counter();first={}
for r in rows('full_safety.jsonl'):
    if not r.get('command_stale'):continue
    i=bisect.bisect_right(times,r['reference_s'])-1
    if i<0:continue
    key='positive' if statuses[i]['speed']>0 else 'zero'
    groups[key]+=1
    first.setdefault(key,dict(reference_s=r['reference_s'],status=statuses[i]))
positive=0;zero=Counter();discarded=Counter();terminal=Counter();first_boundary=None;examples=[]
for e in rows('run1/follow_decisions.jsonl'):
    if e['kind']!='call':continue
    c=e['data'];kind=c['delivery']['kind']
    if kind=='discarded':discarded[c['delivery']['reason']]+=1;continue
    current=c.get('current_rebase',{}).get('output')
    speed=current['speed_mps'] if current else c['output'][0]
    reason=current['reason'] if current else c['output'][2].get('reason','unknown')
    if speed>0:positive+=1
    else:zero[reason]+=1
    normal=c['output'][2].get('normal',{})
    if c['input']['reference_s']>54.:
        terminal[normal.get('reason',reason)]+=1
        if first_boundary is None and normal.get('reason')=='follow_boundary_unconfirmed':
            first_boundary=c
    if current and current['speed_mps']<=0 and len(examples)<3:
        examples.append(dict(reference_s=c['input']['reference_s'],reason=reason,normal_reason=normal.get('reason')))
(A/'terminal_boundary_example.json').write_text(json.dumps(first_boundary,indent=2)+'\n')
result=dict(stale_samples_by_latest_periodic_status=dict(groups),first_stale_examples=first,
 applied_positive=positive,applied_zero_reasons=dict(zero),discard_reasons=dict(discarded),
 terminal_plan_reasons_after54s=dict(terminal),current_zero_examples=examples,
 scope='periodic status association; not per-command byte correlation or isolation of every stall cause')
(A/'delivery_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
