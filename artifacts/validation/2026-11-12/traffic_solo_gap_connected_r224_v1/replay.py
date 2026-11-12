"""새 공통 옵션 OFF의 원 전체 출력과 ON의 r223 명령/관측 대조."""
from pathlib import Path
import argparse,gzip,hashlib,json
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.expiry_reference import reset_after_source_expiry
A=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['on','off']);args=parser.parse_args()
reports=[]
for tag,folder in [('r220','expiry_reference_runtime_r220_v1'),('r217','expiry_null_runtime_r217_v1')]:
    trace=A.parent/folder/'two_actor_run1/run1/follow_decisions.jsonl'
    events=[json.loads(line) for line in trace.open()]
    header=events[0]['data'];header['follow_config']['expiry_route_reference_trial']=True
    header['follow_config']['oblique_gap_trial']=args.mode=='on'
    owner,motion=make_controller(header)
    candidate=json.loads((A.parent/f'traffic_solo_gap_r223_v1/candidate_{tag}.json').read_text())
    rows={r['seq']:r for r in candidate['rows']}
    changed={r['seq']:r for r in candidate['summary']['changes']}
    errors=[];outputs=[];count=0;leaked=[]
    for event in events[1:]:
        data=event['data'];seq=event['seq']
        if event['kind']=='reset':
            if data.get('reason')=='decision_source_expired':reset_after_source_expiry(owner)
            else:owner.reset()
        if event['kind']!='call':continue
        answer=freeze(replay_call(owner,data['input'],motion));count+=1;outputs.append([seq,answer])
        if owner.normal.partial_policy.oblique_metric_enabled:leaked.append(seq)
        if args.mode=='off':
            matches=equal_output(comparable_decision(answer),comparable_decision(data['output']))
        else:
            normal=answer[2].get('normal')or{};row=rows[seq]
            expected=changed[seq]['after'] if seq in changed else data['output'][:2]
            matches=equal_output(answer[:2],expected) and equal_output(normal.get('partial_observation'),row['partial'])
            matches=matches and normal.get('reason')==row['reason'] and normal.get('selector_mode')==row['mode']
        if not matches:errors.append(seq)
    with gzip.open(A/f'{args.mode}_{tag}_outputs.json.gz','xt') as out:json.dump(outputs,out,separators=(',',':'))
    reports.append(dict(trace=tag,calls=count,errors=errors,policy_state_leaks=leaked,
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest()))
with (A/f'{args.mode}.json').open('x') as out:json.dump(reports,out,indent=2);out.write('\n')
print(json.dumps(reports),flush=True)
assert not any(r['errors'] or r['policy_state_leaks'] for r in reports)
