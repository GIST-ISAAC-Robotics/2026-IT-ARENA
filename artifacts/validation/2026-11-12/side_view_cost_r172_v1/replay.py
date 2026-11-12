from pathlib import Path
import importlib.util,json,hashlib
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
A=Path(__file__).resolve().parent
T=A.parent/'released_runtime_r168_v1/two_actor_run1/run1/follow_decisions.jsonl'
spec=importlib.util.spec_from_file_location('r167_baseline',A.parent/'adjacent_live_r152_v1/candidate.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.install()
import candidate
candidate.install()
events=[json.loads(s)for s in T.open()];h=hashlib.sha256(T.read_bytes()).hexdigest()
owner,motion=make_controller(events[0]['data']);commands=[];full=[];count=0
for e in events[1:-1]:
 if e['kind']=='reset':owner.reset()
 elif e['kind']=='call':
  count+=1;o=freeze(replay_call(owner,e['data']['input'],motion));expected=e['data']['output']
  if not equal_output(o[:2],expected[:2]):commands.append(e['seq'])
  if not equal_output(comparable_decision(o),comparable_decision(expected)):full.append(e['seq'])
r=dict(calls=count,command_mismatches=commands,full_mismatches=full,source='r152 planner with equivalent compact-only threat_view',trace_sha256=h,trace_unchanged=h==hashlib.sha256(T.read_bytes()).hexdigest())
(A/'replay.json').write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r));assert not commands and not full and r['trace_unchanged']
