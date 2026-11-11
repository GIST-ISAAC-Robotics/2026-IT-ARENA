"""연결 후 저장 입력 비교. 이미 재현한 대조군의 중복 실행은 생략한다."""
from collections import Counter
from contextlib import contextmanager
import hashlib,json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
import arena_autonomy.released_straight_recovery as recovery


def main():
    out=A/'replay';out.mkdir(exist_ok=False)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    old=json.loads((A.parent/'observed_entry_return_r63_v1/replay_v2/summary.json').read_bytes())
    corner=json.loads((A/'probe/summary.json').read_bytes())
    paths=dict(corner='2026-11-10/corner_parallel_continuity_v1/run1',
        restart='2026-11-10/observed_entry_return_r63_v1/moving_stop_restart/run1',
        moving='2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1',
        combined='2026-11-10/combined_recessed_facility_v1/two_actor_run1/run1')
    original=recovery.compare_released_straight;results={}
    @contextmanager
    def version(current):
        recovery.compare_released_straight=original if current else lambda *args:None
        try:yield
        finally:recovery.compare_released_straight=original
    for name,rel in paths.items():
        trace=ROOT/'artifacts/validation'/rel/'follow_decisions.jsonl';digest=sha(trace)
        if name=='corner':
            assert corner['source_unchanged'] and corner['counts']['command_mismatch']==0
            assert corner['sources'][trace.relative_to(ROOT).as_posix()]==digest
        elif name!='restart':
            verified=old['runs'][name]
            assert old['source_unchanged'] and verified['input_sha256']==digest
            assert verified['counts']['before_recorded_mismatch']==verified['counts'].get('command_changes',0)==0
        variants=('before','current') if name=='restart' else ('current',)
        counts=Counter();ctls={};examples=[];accepted=Counter()
        with (out/(name+'_changes.jsonl')).open('x') as stream:
            for event in read_events(trace):
                data=event['data']
                if event['kind']=='header':
                    for key in variants:ctls[key],motion=build_controller(data)
                elif event['kind']=='reset':
                    for ctl in ctls.values():ctl.reset()
                elif event['kind']=='call':
                    outputs={'before':data['output']}
                    for key in variants:
                        with version(key=='current'):outputs[key]=freeze(replay_call(ctls[key],data['input'],motion))
                    counts['calls']+=1
                    counts['before_recorded_mismatch']+=not equal_output(outputs['before'][:2],data['output'][:2])
                    for key in ('before','current'):counts[key+'_zeros']+=outputs[key][0]<=1e-6
                    detail=outputs['current'][2]
                    proposal=detail.get('released_straight_recovery',detail.get('normal',{}).get('released_straight_recovery',{}))
                    if proposal.get('accepted'):accepted[proposal['reason']]+=1
                    if not equal_output(outputs['before'][:2],outputs['current'][:2]):
                        counts['command_changes']+=1
                        row=dict(t=data['input']['reference_s'],before=outputs['before'][:2],current=outputs['current'][:2],detail=detail)
                        stream.write(json.dumps(row)+'\n')
                        if len(examples)<4:examples.append({k:v for k,v in row.items() if k!='detail'})
        results[name]=dict(counts=dict(counts),input_sha256=digest,input_unchanged=sha(trace)==digest,
            accepted=dict(accepted),examples=examples,
            before_verified='independent controller' if name=='restart' else 'previous source-preserving replay')
        print(json.dumps(dict(case=name,counts=counts,accepted=accepted)),flush=True)
    result=dict(runs=results,sources=sources,source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    assert result['source_unchanged'] and all(r['input_unchanged'] and r['counts']['before_recorded_mismatch']==0 for r in results.values())


if __name__=='__main__':main()
