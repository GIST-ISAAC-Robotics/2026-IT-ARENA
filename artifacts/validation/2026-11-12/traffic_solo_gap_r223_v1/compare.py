"""r220 후반 공백과 기존 단독 저속 정책의 저장 입력 비교. 주행 연결 없음."""
from pathlib import Path
import argparse, hashlib, json
from collections import Counter
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output
from arena_autonomy.expiry_reference import reset_after_source_expiry
from arena_autonomy.local_follow import LocalFollow

A=Path(__file__).resolve().parent
TRACES={
    'r220':A.parent/'expiry_reference_runtime_r220_v1/two_actor_run1/run1/follow_decisions.jsonl',
    'r217':A.parent/'expiry_null_runtime_r217_v1/two_actor_run1/run1/follow_decisions.jsonl'}
parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['baseline','candidate']);args=parser.parse_args()
old=LocalFollow._partial_command
applications=[]
current_seq=None

def partial(owner,*values,**kwargs):
    policy=owner.partial_policy
    selector=owner.pass_selector
    eligible=(selector is not None and selector.mode=='SOLO' and
              all(not s['seen'] and not s['present'] for s in selector.sides.values()))
    was=policy.oblique_metric_enabled
    if eligible:policy.oblique_metric_enabled=True
    try:
        result=old(owner,*values,**kwargs)
        if eligible and (result[2].get('partial_observation') or {}).get('oblique_trial',False):
            applications.append(current_seq)
        return result
    finally:policy.oblique_metric_enabled=was

if args.mode=='candidate':LocalFollow._partial_command=partial
summaries=[]
for tag,trace in TRACES.items():
    events=[json.loads(line) for line in trace.open()]
    header=events[0]['data']
    header['follow_config']['expiry_route_reference_trial']=True
    owner,motion=make_controller(header)
    mismatch=[];changes=[];rows=[]
    for event in events[1:]:
        current_seq=event['seq'];data=event['data']
        if event['kind']=='reset':
            if data.get('reason')=='decision_source_expired':reset_after_source_expiry(owner)
            else:owner.reset()
        if event['kind']!='call':continue
        answer=freeze(replay_call(owner,data['input'],motion))
        expected=data['output'];normal=answer[2].get('normal') or {}
        if not equal_output(comparable_decision(answer),comparable_decision(expected)):mismatch.append(current_seq)
        if not equal_output(answer[:2],expected[:2]):
            changes.append(dict(seq=current_seq,reference_s=data['input']['reference_s'],
                before=expected[:2],after=answer[:2],reason=normal.get('reason'),
                mode=normal.get('selector_mode'),choice=normal.get('selector_choice'),
                partial=normal.get('partial_observation'),sides=normal.get('selector_sides')))
        rows.append(dict(seq=current_seq,reference_s=data['input']['reference_s'],speed=answer[0],
            reason=normal.get('reason'),mode=normal.get('selector_mode'),
            partial=normal.get('partial_observation')))
    summary=dict(trace=tag,calls=len(rows),mismatches=mismatch,changes=changes,
        stop_to_positive=sum(c['before'][0]<=0<c['after'][0] for c in changes),
        positive_to_stop=sum(c['before'][0]>0>=c['after'][0] for c in changes),
        changed_positive_non_solo=[c['seq'] for c in changes if c['after'][0]>0 and c['mode']!='SOLO'],
        reasons=Counter(r['reason'] for r in rows),trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest())
    with (A/f'{args.mode}_{tag}.json').open('x') as out:json.dump(dict(summary=summary,rows=rows),out,indent=2);out.write('\n')
    summaries.append({k:v for k,v in summary.items() if k not in ('changes','mismatches')})
    summaries[-1]['mismatch_count']=len(mismatch)
    if args.mode=='baseline':assert not mismatch, mismatch[:10]
print(json.dumps(summaries),flush=True)
