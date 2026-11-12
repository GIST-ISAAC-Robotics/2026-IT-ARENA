"""r304 실제 입력에서 공통 경로와 연결 후보를 대조. 주행 재실행 아님."""
from pathlib import Path
import json,sys,collections,copy
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(A),str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze,equal_output,comparable_decision
import candidate
with (A/'run1/follow_decisions.jsonl').open() as f:
    header=json.loads(next(f))['data'];events=[json.loads(l) for l in f]
summary={};details={}
for mode in ('recorded_candidate','common_baseline'):
    if mode=='recorded_candidate':candidate.install()
    else:candidate.uninstall()
    follow,motion=make_controller(copy.deepcopy(header));changed=[];counts=collections.Counter();rows=[];n=0
    for e in events:
        if e['kind']=='reset':replay_reset(follow,e['data'])
        if e['kind']!='call':continue
        c=e['data'];actual=freeze(replay_call(follow,c['input'],motion));n+=1
        if not equal_output(comparable_decision(actual),comparable_decision(c['output'])):
            changed.append(c['input']['reference_s'])
            rows.append(dict(reference_s=c['input']['reference_s'],before=c['output'],after=actual))
        counts['positive' if actual[0]>0 else 'zero']+=1;counts[actual[2].get('reason')]+=1
    if mode=='recorded_candidate':assert not changed
    summary[mode]=dict(calls=n,different=len(changed),first_difference=changed[0] if changed else None,counts=dict(counts))
    details[mode]=rows
(A/'replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
(A/'replay_changes.json').write_text(json.dumps(details,separators=(',',':'))+'\n')
print(json.dumps(summary))
for row in details['common_baseline'][:3]:
    print(json.dumps({k:([v[0],v[1],v[2].get('reason'),v[2].get('boundary_selection'),v[2].get('wall_model')] if k in ('before','after') else v) for k,v in row.items()}))
