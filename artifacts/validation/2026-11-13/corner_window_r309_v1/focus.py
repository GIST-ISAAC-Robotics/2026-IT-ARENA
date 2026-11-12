from pathlib import Path
import json,sys,copy
A=Path(__file__).resolve().parent;R=A.parents[3]
sys.path[:0]=[str(A),str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,replay_reset,freeze
import candidate
p=R/'artifacts/validation/2026-11-13/straight_fragment_drive_r308_v1/run1/follow_decisions.jsonl'
with p.open() as f:
 h=json.loads(next(f))['data'];events=[json.loads(x) for x in f]
candidate.install();controller,motion=make_controller(copy.deepcopy(h));out=[]
for e in events:
 if e['kind']=='reset':replay_reset(controller,e['data'])
 if e['kind']!='call':continue
 c=e['data'];result=freeze(replay_call(controller,c['input'],motion));t=c['input']['reference_s']
 if any(abs(t-x)<.011 for x in (20.4,20.9,21.2,27.6)):out.append(dict(time=t,result=result))
(A/'focus.json').write_text(json.dumps(out,indent=2)+'\n')
for v in out:
 print(json.dumps(dict(time=v['time'],speed=v['result'][0],info={k:x for k,x in v['result'][2].items() if not isinstance(x,(list,dict))})))
