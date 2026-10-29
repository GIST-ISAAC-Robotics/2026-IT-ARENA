"""G1 정합을 확인한 run3의 사전 감속 제안 근거를 집계한다. 새 주행 아님."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import statistics

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
report=json.loads((BASE/'run3/report.json').read_text(encoding='utf-8'))
assert report['counts']['normal_detail_matched']==report['counts']['calls']
rows=[json.loads(line) for line in (BASE/'run3/rows.jsonl').open(encoding='utf-8')]
capped={row['seq']:row for row in rows if row['reason']=='handoff_follow' and row['requested'][0]<row['expected'][0]-1e-8}
spans=[]
for row in capped.values():
    at=row['reference_s']
    if not spans or at-spans[-1][-1]>.031:spans.append([at])
    else:spans[-1].append(at)
counts=Counter();steering=[]
source=ROOT/report['input']
for line in source.open(encoding='utf-8'):
    row=json.loads(line)
    if row['seq'] not in capped:continue
    data=row['data'];detail=data['output'][2]
    counts[(detail.get('reason'),detail.get('selector_mode'))]+=1
    steering.append(abs(data['input']['previous_steering_rad']))
ages=[row['persistence'].get('source_span_s',0.) for row in capped.values()]
result=dict(source=report['input'],source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    rows_sha256=hashlib.sha256((BASE/'run3/rows.jsonl').read_bytes()).hexdigest(),
    total_calls=report['counts']['calls'],normal_detail_matched=report['counts']['normal_detail_matched'],
    capped_calls=len(capped),ready_calls=sum(row['persistence']['ready'] for row in capped.values()),
    not_ready_calls=sum(not row['persistence']['ready'] for row in capped.values()),
    source_span_s=dict(min=min(ages),median=statistics.median(ages),max=max(ages)),
    previous_steering_abs_rad=dict(min=min(steering),median=statistics.median(steering),max=max(steering)),
    capped_with_steering_le_008=sum(x<=.08 for x in steering),
    original_states=[dict(reason=k[0],selector=k[1],calls=v) for k,v in counts.items()],
    capped_intervals=[dict(first_reference_s=s[0],last_reference_s=s[-1],calls=len(s)) for s in spans],
    interpretation='Preparation cap is proposed before low-motion evidence is ready in most cases. Intervals are stored call times, not measured driving delay or new trajectory.')
with (BASE/'cap_diagnosis.json').open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
print(json.dumps({k:v for k,v in result.items() if k not in ('capped_intervals','original_states')},indent=2))
