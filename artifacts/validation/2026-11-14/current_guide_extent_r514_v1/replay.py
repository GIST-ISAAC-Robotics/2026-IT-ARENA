from pathlib import Path
import json,hashlib,sys
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.follow_trace import freeze,comparable_decision,equal_output
import arena_autonomy.bypass_entry_trial as entry
source=A.parent/'continuous_pass_preview_drive_r513_v1/run1/follow_decisions.jsonl'
raw=source.read_bytes(); header=None; plans=[];expected={}
for line in raw.splitlines():
 row=json.loads(line);k,d=row['kind'],row['data']
 if k=='runtime_evaluator_header':header=d['header']
 elif k=='runtime_plan_submitted':plans.append(d)
 elif k=='runtime_plan_completed':expected[(d['generation'],d['sequence'])]=d['output']
e=Evaluator('plan',header)
from arena_autonomy.moving_pass import MovingPassGuide
original=MovingPassGuide._wall_covers_guide;captured=[];key=None

def inspect(self,fit,wall):
 out=original(self,fit,wall)
 captured.append(dict(key=key,phase=self.phase,fit=fit,wall=wall,accepted=out,wheelbase_m=self.config.wheelbase_m,length_m=self.config.length_m))
 return out
MovingPassGuide._wall_covers_guide=inspect;generation=None;rows=[]
try:
 for d in plans:
  key=(d['generation'],d['sequence'])
  if key[0]!=generation:e.reset(d['reset_reason']);generation=key[0]
  out=e(d['input']);rows.append(dict(key=key,reference_s=d['input']['reference_s'],speed=out[0],steer=out[1],matched=key in expected and equal_output(comparable_decision(out),comparable_decision(expected[key]))))
finally:MovingPassGuide._wall_covers_guide=original;e.close()
assert source.read_bytes()==raw
(A/'candidate_replay.json').write_text(json.dumps(freeze(dict(rows=rows,comparisons=captured)),indent=2))
print(json.dumps(dict(calls=len(rows),matched=sum(r['matched'] for r in rows),captured=len(captured))))
for r in captured:
 if r['key'][1] in (97,98,99,100,101,102) or r is captured[-1]:
  print(json.dumps(dict(key=r['key'],accepted=r['accepted'],phase=r['phase'],fit=r['fit'],reason=r['wall'].get('reason'),count=r['wall'].get('actual_return_count'))))
