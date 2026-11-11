"""r126 32~44초 경계 재획득의 내부 상태를 저장 재생에서만 관찰한다."""
from collections import Counter
import hashlib
import inspect
import json
from pathlib import Path
import sys

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.local_follow import LocalFollow

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
TRACE=A.parent/'follow_rebase_r126_v1/two_actor_run1/run1/follow_decisions.jsonl'


def state(owner):
    return {key:freeze(getattr(owner,key)) for key in (
        'path_source','path_ref','pending_count','pending_ref','held','release_count','hold_reason')}


def main():
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    digest=sha(TRACE)
    events=[json.loads(line) for line in TRACE.open(encoding='utf8')]
    follow,motion=make_controller(events[0]['data'])
    source=Path(inspect.getsourcefile(LocalFollow));lines,start=inspect.getsourcelines(LocalFollow._baseline_command)
    target=[start+i for i,line in enumerate(lines) if line.rstrip()=='                if eligible:']
    assert len(target)==1
    records=[];calls=0;mismatches=[];captures=[]
    def local(frame,event,arg):
        if event=='line' and frame.f_lineno==target[0]:
            fields=frame.f_locals
            captures.append(dict(now=fields['now'],eligible=bool(fields['eligible']),
                candidate_wall=fields['candidate_wall'],info=freeze(fields['info']),
                candidate=freeze(fields['candidate']),carried=freeze(fields['carried']),
                pending=freeze(fields['self'].pending),before_acceptance=state(fields['self']),
                selection=fields['detail'].get('boundary_selection')))
        return local
    def enter(frame,event,arg):
        if event=='call' and frame.f_code is LocalFollow._baseline_command.__code__:return local
        return None
    for row in events[1:]:
        if row['kind']=='reset':follow.reset();continue
        if row['kind']!='call':continue
        data=row['data'];t=data['input']['reference_s'];selected=32<=t<=44
        captures=[];before=state(follow.normal)
        if selected:sys.settrace(enter)
        try:actual=freeze(replay_call(follow,data['input'],motion))
        finally:sys.settrace(None)
        calls+=1
        if not equal_output(comparable_decision(actual),comparable_decision(data['output'])):
            mismatches.append(row['seq'])
        if selected:
            normal=actual[2].get('normal') or {}
            records.append(dict(seq=row['seq'],reference_s=t,output=actual[:2],
                reason=normal.get('reason'),fresh=normal.get('fresh_boundary_accepted'),
                selection=normal.get('boundary_selection'),delivery=data['delivery']['kind'],
                before=before,after=state(follow.normal),candidates=captures))
    result=dict(replayed_calls=calls,mismatch_sequences=mismatches,window_calls=len(records),
        reasons=dict(Counter(r['reason'] for r in records)),
        eligible_candidates=sum(c['eligible'] for r in records for c in r['candidates']),
        ineligible_candidates=sum(not c['eligible'] for r in records for c in r['candidates']),
        trace_sha256=digest,trace_unchanged=sha(TRACE)==digest,
        implementation_sha256=sha(source),observed_source_line=target[0],
        scope='saved diagnostic observation only; no geometry, thresholds or runtime changes')
    with (A/'details_v2.jsonl').open('x') as stream:
        for row in records:stream.write(json.dumps(row)+'\n')
    with (A/'result.json').open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result));return 0 if not mismatches and result['trace_unchanged'] else 1


if __name__=='__main__':raise SystemExit(main())
