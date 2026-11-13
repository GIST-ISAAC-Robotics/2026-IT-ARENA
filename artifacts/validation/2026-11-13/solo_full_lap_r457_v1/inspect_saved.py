"""현재 실행의 JSON 구조만 작게 읽는다. 제어 실행 없음."""
import json
from pathlib import Path
A=Path(__file__).resolve().parent
def small(v):
    return {k:x for k,x in v.items() if x is None or isinstance(x,(str,int,float,bool))}
printed=set()
for line_no,line in enumerate((A/'run1/follow_decisions.jsonl').open(),1):
    r=json.loads(line);d=r['data'];kind=r['kind']
    if kind=='runtime_plan_completed':
        n=d['output'][2].get('normal',{})
        if n.get('selector_reason')=='guard_curve_current_approach_observed_with_mask_budget' and 'plan' not in printed:
            printed.add('plan');print('new plan',line_no,r['seq'],small(d),'output',d['output'][:2],small(n))
            print('candidate',small(n.get('selector_guard_candidate',{})))
            for k,v in n.get('selector_guard_candidate',{}).items():
                if isinstance(v,dict):print('candidate/'+k,small(v))
    if kind=='call' and d.get('current_rebase') and 'current' not in printed:
        printed.add('current');q=d['current_rebase'];print('current',line_no,r['seq'],small(q['input']),small(q['output']),small(q['compute']),small(d['compute']),small(d['delivery']))
        print('current input keys',list(q['input'].keys()));print('current compute keys',list(q['compute'].keys()))
    if len(printed)==2:break
