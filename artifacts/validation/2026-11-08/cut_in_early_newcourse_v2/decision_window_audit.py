"""시험 종료 후의 제어 판단을 실제 9.5초 주행과 분리한다. 등록 파일·원 판정은 보존."""
from pathlib import Path
import collections,json
A=Path(__file__).resolve().parent;run=A/'close_cut_in/run1'
r=json.loads((run/'report.json').read_bytes());lo=r['case_window']['green_sim_s'];hi=r['case_window']['end_sim_s']
inside=[];outside=[]
for line in (run/'follow_decisions.jsonl').open():
    e=json.loads(line)
    if e['kind']!='call':continue
    d=e['data'];t=d['input']['reference_s'];o=d['output']
    row=dict(t=t,green_relative_s=t-lo,command_mps=o[0],state=o[2].get('handoff_state'),reason=o[2].get('reason'))
    (inside if lo-1e-9<=t<=hi+1e-9 else outside).append(row)
assert inside
out=dict(scope='actual case interval only, supplementary decision counters; original evaluation and raw evidence unchanged',window_s=[lo,hi],
         case_calls=len(inside),case_final=inside[-1],case_states=dict(collections.Counter(x['state'] for x in inside)),
         case_bypass_calls=sum(x['state']=='BYPASS' for x in inside),
         excluded_bypass=[x for x in outside if x['state']=='BYPASS'])
(A/'decision_window_audit.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(out))
