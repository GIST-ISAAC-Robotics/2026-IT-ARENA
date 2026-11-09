"""C4 정체 구간의 실제 벽 지지와 지주 반환을 원출력 재생으로 조사한다."""
import json
import numpy as np
from pathlib import Path
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.bypass_entry_trial import EntryGuide
A=Path(__file__).resolve().parent;old=EntryGuide._outer_wall_fit;captures=[]

def wrapped(self,scene,remembered):
    fit,wall=old(self,scene,remembered)
    if any(abs(scene.t_ref_s-t)<1e-6 for t in (21.5,23.42,24.52,27.0)):
        captures.append(dict(t=scene.t_ref_s,phase=self.phase,side=self.side,fit=fit,wall=wall,
            remembered=remembered.tolist(),path=self.path.tolist(),
            rays=[dict(index=r.source_index,status=r.status,point=r.hit_xy_m,age=scene.t_ref_s-r.acquired_at_s) for r in scene.rays]))
    return fit,wall
EntryGuide._outer_wall_fit=wrapped
n=mismatches=0
for e in read_events(A/'c_stationary_posts/run4/follow_decisions.jsonl'):
    d=e['data']
    if e['kind']=='header':c,mc=build_controller(d)
    elif e['kind']=='reset':c.reset()
    elif e['kind']=='call':
        out=freeze(replay_call(c,d['input'],mc));n+=1
        mismatches+=not equal_output(comparable_decision(out),comparable_decision(d['output']))
        if d['input']['reference_s']>27.01:break
(A/'c4_wall_diagnosis.json').write_text(json.dumps(dict(calls=n,mismatches=mismatches,captures=captures),indent=1)+'\n')
brief=[]
for x in captures:
    r=[r for r in x['rays'] if r['status']=='return'];pts=np.asarray([v['point'] for v in r]);ids=np.asarray([v['index'] for v in r])
    limit=.1+.035*np.minimum(np.linalg.norm(pts[:-1],axis=1),np.linalg.norm(pts[1:],axis=1))
    splits=np.flatnonzero((np.linalg.norm(np.diff(pts,axis=0),axis=1)>limit)|(np.diff(ids)!=1))+1
    groups=[]
    for group in np.split(np.arange(len(pts)),splits):
        p=pts[group]
        if np.min(np.linalg.norm(p,axis=1))<.65:
            groups.append(dict(ids=[int(ids[group[0]]),int(ids[group[-1]])],n=len(p),
                span=np.ptp(p,axis=0).tolist(),lo=p.min(axis=0).tolist(),hi=p.max(axis=0).tolist()))
    brief.append(dict(t=x['t'],phase=x['phase'],side=x['side'],fit=x['fit'],
        wall=x['wall'],groups=groups))
(A/'c4_wall_compact.json').write_text(json.dumps(brief,indent=1)+'\n')
print(json.dumps(dict(calls=n,mismatches=mismatches,captures=[dict(t=x['t'],phase=x['phase'],side=x['side'],fit=x['fit'],
    wall_reason=x['wall'].get('reason'),bridged=x['wall'].get('narrow_occluders_bridged'),groups=x['groups']) for x in brief])),flush=True)
