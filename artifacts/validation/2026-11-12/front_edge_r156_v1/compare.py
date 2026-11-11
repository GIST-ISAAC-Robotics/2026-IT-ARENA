"""보존 원본 함수의 동일 재생 뒤 경계 이웃 수정만 대조한다."""
from collections import Counter
import hashlib
import importlib.util
import json
from pathlib import Path
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.partial_observation import PartialObservationPolicy

A=Path(__file__).resolve().parent
TRACE=A.parent/'adjacent_live_r152_v1/two_actor_run2/run1/follow_decisions.jsonl'

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module);return module

def main():
    events=[json.loads(s) for s in TRACE.open()];digest=hashlib.sha256(TRACE.read_bytes()).hexdigest()
    load('r156_planner',A.parent/'adjacent_live_r152_v1/candidate.py').install()
    before=load('r156_before',A/'partial_observation.before.py').PartialObservationPolicy.evaluate
    production=PartialObservationPolicy.evaluate
    after=load('r156_candidate',A/'partial_observation.candidate.py').PartialObservationPolicy.evaluate
    outputs={};mismatches=[];changes=[];counts=Counter()
    try:
        for name,method in [('baseline',before),('candidate',after)]:
            PartialObservationPolicy.evaluate=method
            owner,motion=make_controller(events[0]['data']);rows=[]
            for event in events[1:-1]:
                if event['kind']=='reset':owner.reset()
                elif event['kind']=='call':
                    out=freeze(replay_call(owner,event['data']['input'],motion));rows.append(dict(seq=event['seq'],output=out))
                    old=event['data']['output']
                    same=equal_output(comparable_decision(out),comparable_decision(old))
                    if name=='baseline' and not same:mismatches.append(event['seq'])
                    if name=='candidate':
                        if not same:
                            normal=out[2].get('normal',out[2]);partial=normal.get('partial_observation',{})
                            changes.append(dict(seq=event['seq'],reference_s=event['data']['input']['reference_s'],
                                before=old[:2],after=out[:2],reason=normal.get('reason'),
                                partial_reason=partial.get('reason'),baseline_reason=normal.get('baseline_reason')))
                        counts['stop_to_go']+=old[0]<=0<out[0]
                        counts['go_to_stop']+=out[0]<=0<old[0]
                        counts['positive_changed']+=old[0]>0 and out[0]>0 and old[:2]!=out[:2]
            outputs[name]=rows
    finally:PartialObservationPolicy.evaluate=production
    summary=dict(calls=len(outputs['baseline']),baseline_mismatches=mismatches,changed_outputs=len(changes),
        changes=changes,counts=dict(counts),trace_sha256=digest,
        trace_unchanged=hashlib.sha256(TRACE.read_bytes()).hexdigest()==digest,
        source_before_sha256=hashlib.sha256((A/'partial_observation.before.py').read_bytes()).hexdigest(),
        source_after_sha256=hashlib.sha256((A/'partial_observation.candidate.py').read_bytes()).hexdigest())
    (A/'comparison.json').write_text(json.dumps(summary,indent=2))
    (A/'expected.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in outputs['candidate']))
    print(json.dumps({k:v for k,v in summary.items() if k!='changes'}))
    assert not mismatches and summary['trace_unchanged']

if __name__=='__main__':main()
