"""동일 저장 입력에서 새 복귀 선택만 끈 독립 상태와 비교. 실제 주행 증거는 아니다."""
from collections import Counter
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.bypass_observed_return import ObservedEntryReturn

def main():
    current_only='--current-only' in sys.argv
    out=A/('replay_v2' if current_only else 'replay');out.mkdir(exist_ok=False)
    verified=json.loads((A/'replay/summary.json').read_bytes()) if current_only else None
    paths=dict(
        restart='2026-11-10/restart_recessed_facility_v1/moving_stop_restart/run1',
        stationary='2026-11-10/facility_traffic_stability_v1/stationary_recessed_r60/run1',
        moving='2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1',
        combined='2026-11-10/combined_recessed_facility_v1/two_actor_run1/run1')
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    original=ObservedEntryReturn.evaluate;results={}
    @contextmanager
    def version(current):
        ObservedEntryReturn.evaluate=original if current else lambda self,g,s,v,p,dt,base:base
        try:yield
        finally:ObservedEntryReturn.evaluate=original
    for name,rel in paths.items():
        path=ROOT/'artifacts/validation'/rel/'follow_decisions.jsonl';before_sha=sha(path)
        if current_only:
            assert verified['runs'][name]['input_sha256']==before_sha
            assert verified['runs'][name]['counts']['before_recorded_mismatch']==0
        counts=Counter();ctls={};examples=[];applications=[]
        with (out/(name+'_changes.jsonl')).open('x') as stream:
            for event in read_events(path):
                data=event['data']
                if event['kind']=='header':
                    for key in (('current',) if current_only else ('before','current')):
                        ctls[key],motion=build_controller(data)
                elif event['kind']=='reset':
                    for ctl in ctls.values():ctl.reset()
                elif event['kind']=='call':
                    outputs={}
                    if current_only:outputs['before']=data['output']
                    for key in (('current',) if current_only else ('before','current')):
                        with version(key=='current'):outputs[key]=freeze(replay_call(ctls[key],data['input'],motion))
                        counts[key+'_zeros']+=outputs[key][0]<=1e-6
                    counts['calls']+=1
                    if current_only:counts['before_zeros']+=outputs['before'][0]<=1e-6
                    counts['before_recorded_mismatch']+=not equal_output(outputs['before'][:2],data['output'][:2])
                    detail=outputs['current'][2].get('bypass',{}).get('observed_entry_return',{})
                    if detail.get('applied'):applications.append(dict(t=data['input']['reference_s'],detail=detail))
                    if not equal_output(outputs['before'][:2],outputs['current'][:2]):
                        counts['command_changes']+=1
                        row=dict(t=data['input']['reference_s'],before=outputs['before'][:2],current=outputs['current'][:2],
                            detail=outputs['current'][2])
                        stream.write(json.dumps(row)+'\n')
                        if len(examples)<4:examples.append({k:v for k,v in row.items() if k!='detail'})
        results[name]=dict(counts=dict(counts),input_sha256=before_sha,input_unchanged=sha(path)==before_sha,
            applications=applications,examples=examples)
        print(json.dumps(dict(case=name,counts=counts,applications=[a['t'] for a in applications])),flush=True)
    result=dict(runs=results,sources=sources,source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    assert result['source_unchanged'] and all(r['input_unchanged'] and r['counts']['before_recorded_mismatch']==0 for r in results.values())

if __name__=='__main__':main()
