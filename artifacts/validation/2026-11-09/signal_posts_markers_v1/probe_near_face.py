"""C 실제 실패 기록에서 별도 근접 면 관찰 사본만 비교. 런타임 소스는 변경하지 않는다."""
import json
import hashlib
from pathlib import Path
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
from near_face_candidate import NearbyFace
A=Path(__file__).resolve().parent
source=A/'c_stationary_posts/run1/follow_decisions.jsonl'
original=ForecastGuide._padding

def compared(self,scene,config,side,wall,growth_s=0.):
    padding,evidence=original(self,scene,config,side,wall,growth_s)
    if not getattr(self,'near_face_probe',False):return padding,evidence
    if not hasattr(self,'near_observers'):
        self.near_observers={s:NearbyFace(self.corridor_policy) for s in ('left','right')}
    padding,extra=self.near_observers[side].apply(scene,config,side,wall,self.motion_history,padding,growth_s)
    return padding,dict(evidence,nearby_face=extra)
ForecastGuide._padding=compared
rows=[]
before=hashlib.sha256(source.read_bytes()).hexdigest()
for e in read_events(source):
    d=e['data']
    if e['kind']=='header':
        baseline,mc=build_controller(d);candidate,_=build_controller(d)
        candidate.bypass.guide.near_face_probe=True
    elif e['kind']=='reset':
        baseline.reset();candidate.reset()
        candidate.bypass.guide.near_face_probe=True
        candidate.bypass.guide.near_observers={}
        del candidate.bypass.guide.near_observers
    elif e['kind']=='call':
        a=freeze(replay_call(baseline,d['input'],mc));b=freeze(replay_call(candidate,d['input'],mc))
        near=b[2].get('bypass',{}).get('outer_boundary_padding',{}).get('nearby_face',{})
        rows.append(dict(t=d['input']['reference_s'],before=a[0],after=b[0],
            same=equal_output(a[:2],b[:2]),baseline_matches=equal_output(comparable_decision(a),comparable_decision(d['output'])),
            nearby=near,behavior=b[2].get('behavior')))
(A/'near_face_rows.json').write_text(json.dumps(rows,indent=1)+'\n')
changes=[r for r in rows if not r['same']]
summary=dict(scope='open-loop C failure inputs, separately observed adjacent face; no new simulation',
    calls=len(rows),baseline_mismatches=sum(not r['baseline_matches'] for r in rows),
    zero_before=sum(r['before']<=1e-6 for r in rows),zero_after=sum(r['after']<=1e-6 for r in rows),
    changes=len(changes),first_changes=changes[:2],last=rows[-1],
    source_unchanged=hashlib.sha256(source.read_bytes()).hexdigest()==before)
(A/'near_face_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=1))
