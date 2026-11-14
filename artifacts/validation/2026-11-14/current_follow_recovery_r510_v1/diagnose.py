from pathlib import Path
import sys,json,hashlib
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.follow_trace import freeze,comparable_decision,equal_output
from arena_autonomy.moving_pass import MovingPassAdapter
from arena_autonomy.local_follow import LocalFollow
source=A.parent/'moving_pass_facility_drive_r503_v1/run1/follow_decisions.jsonl'
raw=source.read_bytes();plans=[];expected={}
for line in raw.splitlines():
 r=json.loads(line);k,d=r['kind'],r['data']
 if k=='runtime_evaluator_header':header=d['header']
 elif k=='runtime_plan_submitted':plans.append(d)
 elif k=='runtime_plan_completed':expected[(d['generation'],d['sequence'])]=d['output']
e=Evaluator('plan',header);original=MovingPassAdapter.command;alternates={};captured=[];key=None

def inspect(self,scene,history,speed,side,previous,dt,*,raw_scan=None):
 output=original(self,scene,history,speed,side,previous,dt,raw_scan=raw_scan)
 if self.active:
  local=alternates.setdefault(self,LocalFollow(self.owner.config))
  alternative=local.command(scene,history,speed,side,previous,dt,raw_scan=raw_scan)
  captured.append(dict(key=key,speed=speed,previous=previous,dt=dt,side=side,
   original=output,alternative=alternative,phase=self.guide.phase))
 return output
MovingPassAdapter.command=inspect;generation=None;rows=[]
try:
 for d in plans:
  key=(d['generation'],d['sequence'])
  if key[0]!=generation:e.reset(d['reset_reason']);generation=key[0]
  out=e(d['input']);rows.append(dict(key=key,matched=key in expected and equal_output(comparable_decision(out),comparable_decision(expected[key]))))
finally:MovingPassAdapter.command=original;e.close()
assert source.read_bytes()==raw
(A/'diagnostic.json').write_text(json.dumps(freeze(dict(source_sha256=hashlib.sha256(raw).hexdigest(),rows=rows,comparisons=captured)),indent=2))
print(json.dumps(dict(calls=len(rows),matched=sum(r['matched'] for r in rows),captured=len(captured),recoverable=sum(x['original'][0]==0 and x['alternative'][0]>0 for x in captured))))
for x in captured:
 if x['key'][1] in (94,95,96,97,98,99,100,101,102,103) or x is captured[-1]:
  o,a=x['original'],x['alternative'];print(json.dumps(dict(key=x['key'],old=o[:2],new=a[:2],reason=a[2].get('reason'),selector=a[2].get('selector_choice'),path_source=a[2].get('path_source_s'),boundary_from_memory=a[2].get('boundary_from_memory'))))
