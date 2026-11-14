from pathlib import Path
import json,hashlib,sys
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.follow_trace import freeze,comparable_decision,equal_output
import arena_autonomy.bypass_entry_trial as entry
source=A.parent/'rear_overtake_current_geometry_r499_v1/run1/follow_decisions.jsonl'
raw=source.read_bytes(); header=None; plans=[];expected={}
for line in raw.splitlines():
 row=json.loads(line);k,d=row['kind'],row['data']
 if k=='runtime_evaluator_header':header=d['header']
 elif k=='runtime_plan_submitted':plans.append(d)
 elif k=='runtime_plan_completed':expected[(d['generation'],d['sequence'])]=d['output']
e=Evaluator('plan',header)
original=entry._compare_entries; captured=[]; enabled=False; key=None
angles=[]
from functools import partial
from arena_autonomy.adaptive_entry_preview import shorter_pursuit
def inspect(*args,**kw):
 out=original(*args,**kw)
 if not enabled:return out
 scene=args[0];cfg=kw['config'];guide=kw.get('guide'); points=np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
 pad=entry.per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
 pdetail={}
 if guide and guide.get('stationary_outer_boundary'):
  provider=kw.get('padding_provider') or entry.current_outer_wall_padding
  pad,pdetail=provider(scene,cfg,guide['outer_side'],guide['reference']['walls'][guide['outer_side']],0.)
 grid=[]
 for angle in angles:
  clearance=entry.swept_limit(points,angle,cfg.wheelbase_m,horizon=2.5,margin=pad,length=cfg.length_m,width=cfg.width_m)
  grid.append(dict(angle=angle,clearance=clearance))
 detail={k:v for k,v in out.items() if k not in ('candidates','reference')}
 detail['candidates']=[{k:v for k,v in c.items() if k not in ('path_points','motion_region')} for c in out.get('candidates',[])]
 alternatives=[]
 if guide and not kw.get('follow_headway',False):
  for lookahead in (.24,.18,.12):
   alt=original(*args,**dict(kw,pursuit_provider=partial(shorter_pursuit,lookahead_m=lookahead)))
   alternatives.append(dict(lookahead=lookahead,selected=alt.get('selected'),rows=[{k:v for k,v in c.items() if k not in ('path_points','motion_region')} for c in alt.get('candidates',[])]))
 captured.append(dict(alternatives=alternatives,key=key,follow_headway=kw.get('follow_headway',False),stationary=guide.get('stationary_outer_boundary') if guide else None,
  reference=out.get('reference'),padding=pdetail,angle_clearances=grid,result=detail))
 return out
entry._compare_entries=inspect
generation=None;rows=[]
try:
 for i,item in enumerate(plans):
  key=(item['generation'],item['sequence'])
  if key[0]!=generation:e.reset(item['reset_reason']);generation=key[0]
  enabled=(key[1] in (96,97,98,99,100) or i>=len(plans)-2)
  out=e(item['input']); match=key in expected and equal_output(comparable_decision(out),comparable_decision(expected[key]))
  rows.append(dict(key=key,reference_s=item['input']['reference_s'],speed=out[0],steer=out[1],reason=out[2].get('reason'),original_match=match))
finally:
 entry._compare_entries=original;e.close()
assert source.read_bytes()==raw
(A/'candidate_replay.json').write_text(json.dumps(freeze(dict(scope='r502 same-path shorter pursuit candidate full sequence replay; saved input only, not closed loop',trace_sha256=hashlib.sha256(raw).hexdigest(),rows=rows,comparisons=captured)),indent=2),encoding='utf-8')
print(json.dumps(dict(calls=len(rows),matched=sum(r['original_match'] for r in rows),captured=len(captured),zeros=sum(r['speed']==0 for r in rows))))
