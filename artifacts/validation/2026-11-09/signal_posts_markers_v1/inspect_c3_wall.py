"""C3 복귀 단계의 벽 지지 구간을 원판단과 동일 재생하며 추출한다."""
import json
from dataclasses import asdict
from pathlib import Path
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.bypass_entry_trial import EntryGuide
A=Path(__file__).resolve().parent;old=EntryGuide._outer_wall_fit;captures=[]
def wrapped(self,scene,remembered):
    fit,wall=old(self,scene,remembered)
    if any(abs(scene.t_ref_s-t)<1e-6 for t in (15.1,15.4,16.2)):
        captures.append(dict(t=scene.t_ref_s,phase=self.phase,side=self.side,fit=fit,wall=wall,
            remembered=remembered.tolist(),config=asdict(self.config),
            rays=[dict(index=r.source_index,status=r.status,point=r.hit_xy_m,age=scene.t_ref_s-r.acquired_at_s) for r in scene.rays]))
    return fit,wall
EntryGuide._outer_wall_fit=wrapped
n=mismatches=0
for e in read_events(A/'c_stationary_posts/run3/follow_decisions.jsonl'):
    d=e['data']
    if e['kind']=='header':c,mc=build_controller(d)
    elif e['kind']=='reset':c.reset()
    elif e['kind']=='call':
        out=freeze(replay_call(c,d['input'],mc));n+=1
        mismatches+=not equal_output(comparable_decision(out),comparable_decision(d['output']))
        if d['input']['reference_s']>16.21:break
(A/'c3_wall_diagnosis.json').write_text(json.dumps(dict(calls=n,mismatches=mismatches,captures=captures),indent=1)+'\n')
print(json.dumps(dict(calls=n,mismatches=mismatches,captures=[dict(t=x['t'],phase=x['phase'],side=x['side'],fit=x['fit'],reason=x['wall'].get('reason'),rear_witness=x['wall'].get('rear_geometry_witness')) for x in captures])),flush=True)
