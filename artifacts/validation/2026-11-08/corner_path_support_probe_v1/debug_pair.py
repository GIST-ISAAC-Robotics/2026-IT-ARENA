from pathlib import Path
import json,sys,math
import numpy as np
import candidate
from arena_autonomy.local_follow import FollowConfig,path_agrees_arc
from arena_autonomy.piecewise_wall import corner_path
from arena_autonomy.local_path import _origin_error,_path_ahead
A=Path(__file__).resolve().parent
r=next(x for x in json.loads((A/'snapshots.json').read_bytes())['captures'] if abs(x['t']-34.62)<.001)
values={}
def trace(frame,event,arg):
    if frame.f_code is candidate.propose.__code__ and event=='return':values.update(frame.f_locals)
    return trace
sys.settrace(trace)
candidate.propose(r['points'],r['carried'],r['old_wall'],FollowConfig(**r['config']),r['t']-r['old_path_source'])
sys.settrace(None)
p=values['p'];cfg=values['config'];ref=values['reference'];rows=[]
for ai,a,t0 in values['near']:
 for bi,b,t1 in values['faces']:
  if np.intersect1d(ai,bi).size:continue
  angle=math.atan2(float(np.cross(t0,t1)),float(t0@t1))
  if not math.radians(40)<=abs(angle)<=math.radians(135):continue
  u,v=np.linalg.solve(np.column_stack((t0,-t1)),b-a);corner=a+u*t0
  ap=(p[ai]-corner)@t0;bp=(p[bi]-corner)@t1
  guide=np.vstack((corner+np.linspace(ap.min(),0,32)[:,None]*t0,corner+np.linspace(0,min(bp.max(),1.8),32)[1:,None]*t1))
  path=corner_path(guide,r['old_wall'],cfg.wall_offset_m,cfg.wheelbase_m/math.tan(cfg.max_steering_rad))
  rows.append(dict(angle=angle,near_count=len(ai),far_count=len(bi),corner=corner.tolist(),gap=-float(ap.max()),bmin=float(bp.min()),bmax=float(bp.max()),
   path=path is not None,anchor=None if path is None else _origin_error(path),ahead=None if path is None else _path_ahead(path),
   agrees=False if path is None else path_agrees_arc(path,ref,.08,hi=1.5)))
near_candidates=[]
for group in values['groups']:
 pts=p[group]; mask=(abs(pts[:,0])<=.35)&(-pts[:,1]>.1)&(abs(pts[:,1]+cfg.wall_offset_m)<=.12); an=pts[mask]
 if len(an):near_candidates.append(dict(n=len(an),span=float(np.ptp(an[:,0])),xy=an.tolist()))
result=dict(near=len(values['near']),faces=len(values['faces']),rows=rows,near_candidates=near_candidates)
(A/'pair_diagnostic.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
print(json.dumps(result,indent=2))
