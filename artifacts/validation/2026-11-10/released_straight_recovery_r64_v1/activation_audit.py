"""r64 실제 입력의 후보 진입 조건만 기록한다. 명령·조건은 변경하지 않는다."""
from collections import Counter
import hashlib,json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.observation_support import source_valid
from arena_autonomy.pass_release_trial import NORMAL_BASELINE
import arena_autonomy.released_straight_recovery as candidate


def main():
    trace=A/'run1/follow_decisions.jsonl'
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    original=candidate.compare_released_straight
    counts=Counter();rows=[];seen=set();digest=sha(trace)
    def wrapped(owner,scene,history,baseline,selected,speed,previous,dt):
        result=original(owner,scene,history,baseline,selected,speed,previous,dt)
        counts['helper_calls']+=1
        if selected[0]<=0 and owner.mode=='RELEASED' and scene.source_first_s not in seen:
            seen.add(scene.source_first_s)
            d=baseline[2];p=d.get('partial_observation') or {}
            gates=dict(no_side=not any(s['present'] for s in owner.sides.values()),
                reason=selected[2].get('selector_reason')=='large_motion_region_unobserved',
                baseline_normal=d.get('reason') in NORMAL_BASELINE and baseline[0]>0,
                current=d.get('boundary_from_memory') is False and d.get('path_source_s')==scene.source_first_s,
                fully_observed=p.get('allowed') is True and p.get('reason')=='fully_observed',
                valid=source_valid(scene))
            rows.append(dict(t=scene.t_ref_s,source=scene.source_first_s,speed=speed,previous=previous,
                baseline=list(baseline[:2]),baseline_reason=d.get('reason'),partial_reason=p.get('reason'),
                selector_reason=selected[2].get('selector_reason'),gates=gates,candidate=freeze(result)))
        return result
    candidate.compare_released_straight=wrapped
    try:
        for event in read_events(trace):
            d=event['data']
            if event['kind']=='header':ctl,motion=build_controller(d)
            elif event['kind']=='reset':ctl.reset()
            elif event['kind']=='call':
                result=freeze(replay_call(ctl,d['input'],motion));counts['calls']+=1
                counts['mismatch']+=not equal_output(result[:2],d['output'][:2])
    finally:candidate.compare_released_straight=original
    summary=dict(counts=dict(counts),rows=rows,input_sha256=digest,input_unchanged=sha(trace)==digest,
        sources=sources,source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (A/'activation_audit.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(counts=counts,rows=rows,unchanged=summary['source_unchanged'])))
    assert counts['mismatch']==0 and summary['input_unchanged'] and summary['source_unchanged']

if __name__=='__main__':main()
