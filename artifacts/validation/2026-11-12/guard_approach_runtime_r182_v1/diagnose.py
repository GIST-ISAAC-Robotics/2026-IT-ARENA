"""r182의 실제 선택/현재 검사/종료 범위를 요약한다."""
import hashlib
import json
from collections import Counter
from pathlib import Path
A=Path(__file__).resolve().parent;D=A/'two_actor_run1';RUN=D/'run1'
report=json.loads((RUN/'report.json').read_bytes())
end=report['case_window']['end_sim_s']
trace=RUN/'follow_decisions.jsonl'
counts=Counter();rejects=Counter();zero=Counter();resets=[];tail=[]
for event in map(json.loads,trace.open()):
    d=event['data']
    if event['kind']=='reset':resets.append(dict(seq=event['seq'],**d))
    if event['kind']!='call' or d['input']['reference_s']>end:continue
    counts['case_calls']+=1
    normal=d['output'][2].get('normal',{})
    for key in ('selector_adjacent_low_retry','selector_approach_low_retry'):
        if normal.get(key,{}).get('applied'):counts[key]+=1
    current=d.get('current_rebase',{}).get('output')
    if current:
        counts['current_calls']+=1
        counts['current_positive']+=current['speed_mps']>0
        if current['speed_mps']<=0:rejects[current['reason']]+=1
        if current.get('current_approach_low_retry',{}).get('applied'):counts['current_approach_low_retry']+=1
    if d['output'][0]<=0:zero[str((normal.get('selector_mode'),normal.get('reason'),normal.get('selector_reason')))]+=1
    if d['input']['reference_s']>=58:
        tail.append(dict(seq=event['seq'],reference_s=d['input']['reference_s'],
            preferred_wall=d['input']['side'],selected_wall=normal.get('path_wall'),
            selection=normal.get('boundary_selection'),speed=d['output'][0],steering=d['output'][1],
            delivery=d.get('delivery'),generation=d.get('compute',{}).get('generation')))
result=dict(trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),case_end_s=end,
    counts=dict(counts),current_rejections=dict(rejects),planner_zero=dict(zero),resets=resets,
    departure_sequence=tail,abort=report['abort'],raw_stop=report['stop'],
    shutdown_clean=report['shutdown_clean'],remaining=report['remaining_test_pids'],
    integrity=json.loads((D/'extra_integrity.json').read_bytes()))
with (A/'diagnosis.json').open('x') as out:json.dump(result,out,indent=2);out.write('\n')
print(json.dumps(dict(counts=dict(counts),current_rejections=dict(rejects))))
