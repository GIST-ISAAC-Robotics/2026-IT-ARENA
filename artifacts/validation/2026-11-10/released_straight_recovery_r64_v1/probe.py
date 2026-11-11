"""직전 주행을 바꾸지 않는 직진/제자리 조향 비교. 실제 명령은 원선택 그대로다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.pass_release_trial import PassReleaseSelector
from arena_autonomy.released_straight_recovery import compare_released_straight


def main():
    out=A/'probe';out.mkdir(exist_ok=False)
    trace=ROOT/'artifacts/validation/2026-11-10/corner_parallel_continuity_v1/run1/follow_decisions.jsonl'
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    files=[Path(__file__),trace,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in files}
    with zipfile.ZipFile(out/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files:
            if p!=trace:z.write(p,p.relative_to(ROOT))
    original=PassReleaseSelector.select;forward=LocalFollow._forward_command
    active_history=[];rows=[];counts=Counter();seen=set()
    def capture_history(self,scene,history,*args,**kw):
        active_history.append(history)
        try:return forward(self,scene,history,*args,**kw)
        finally:active_history.pop()
    def select(self,scene,baseline,parallel,speed,previous,dt,**kw):
        result=original(self,scene,baseline,parallel,speed,previous,dt,**kw)
        if scene.source_first_s in seen:return result
        candidate=compare_released_straight(self,scene,active_history[-1],baseline,result,speed,previous,dt)
        if candidate is None:return result
        seen.add(scene.source_first_s)
        row=dict(t=scene.t_ref_s,source=scene.source_first_s,speed=speed,previous=previous,
                 candidate=candidate)
        if candidate.get('accepted') and candidate['speed_mps']==0:
            # 실제 센서/궤적에 적용하지 않은 정렬 완료 가정의 별도 비교.
            row['after_alignment_hypothesis']=compare_released_straight(
                self,scene,active_history[-1],baseline,result,speed,0.,dt)
        rows.append(freeze(row));return result
    LocalFollow._forward_command=capture_history;PassReleaseSelector.select=select
    try:
        for event in read_events(trace):
            data=event['data']
            if event['kind']=='header':controller,motion=build_controller(data)
            elif event['kind']=='reset':controller.reset()
            elif event['kind']=='call':
                result=freeze(replay_call(controller,data['input'],motion));counts['calls']+=1
                counts['command_mismatch']+=not equal_output(result[:2],data['output'][:2])
    finally:
        LocalFollow._forward_command=forward;PassReleaseSelector.select=original
    result=dict(scope='read-only candidate; hypotheses do not alter recorded commands',
        counts=dict(counts),rows=rows,sources=sources,
        source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    brief=[dict(t=r['t'],speed=r['speed'],reason=r['candidate']['reason'],
        quiet=r['candidate'].get('motion_quiet'),selected=r['candidate']['accepted'],
        hypothetical=r.get('after_alignment_hypothesis',{}).get('reason')) for r in rows]
    print(json.dumps(dict(counts=counts,rows=brief,source_unchanged=result['source_unchanged'])))
    assert counts['command_mismatch']==0 and result['source_unchanged']


if __name__=='__main__':main()
