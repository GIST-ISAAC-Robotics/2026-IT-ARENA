"""C4 원판단의 정지 분기를 요약한다. 제어/주행 변경 없음."""
import json
from collections import Counter
from pathlib import Path
A=Path(__file__).resolve().parent;run=A/'c_stationary_posts/run4'
calls=[];last=None;transitions=[];counter=Counter()
for line in (run/'follow_decisions.jsonl').open(encoding='utf8'):
    e=json.loads(line)
    if e['kind']!='call':continue
    d=e['data'];t=d['input']['reference_s'];out=d['output'];detail=out[2]
    bypass=detail.get('bypass',{});reason=bypass.get('reason');counter[reason]+=1
    row=dict(t=t,speed=out[0],steer=out[1],reason=reason,phase=bypass.get('guide_phase'),
             state=detail.get('handoff_state'),behavior=detail.get('behavior'))
    calls.append(row)
    if reason!=last:transitions.append(row);last=reason
(A/'c4_last_call.json').write_text(json.dumps(d,indent=1)+'\n')
(A/'c4_reason_summary.json').write_text(json.dumps(dict(reasons=dict(counter),transitions=transitions,calls=calls),indent=1)+'\n')
print(json.dumps(dict(reasons=dict(counter),last_transitions=transitions[-18:],
    last_detail_keys=list(detail),last_bypass_keys=list(bypass))))
