from pathlib import Path
import hashlib,importlib.util,json,sys,time
from arena_autonomy.follow_trace import MotionConfig,FollowConfig,replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.bypass_corridor_trial import CorridorGuide
A=Path(__file__).resolve().parent
def module(name,file):
 spec=importlib.util.spec_from_file_location(name,A/file);m=importlib.util.module_from_spec(spec)
 sys.modules[name]=m;spec.loader.exec_module(m);return m
module('arena_autonomy.adaptive_corridor_offset','roomy_candidate.py')
plan=module('staged_corridor_r40','bypass_corridor_candidate.py').CorridorGuide._plan
handoff=module('staged_handoff_r40','bypass_handoff_candidate.py')
handoff.HandoffExecutionConfig=HandoffExecutionConfig
initial=handoff.BypassHandoffFollow.__init__
old_plan=CorridorGuide._plan;old_init=BypassHandoffFollow.__init__
trace=A.parent/'short_entry_transition_v1/run1/follow_decisions.jsonl'
matched=0;first_detail=first_command=None;start=time.perf_counter()
for line in trace.open():
 e=json.loads(line);d=e['data']
 if e['kind']=='header':
  mc=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config']);execution=HandoffExecutionConfig.from_mapping(d['execution_config'])
  old=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
  BypassHandoffFollow.__init__=initial
  new=BypassHandoffFollow(cfg,execution,allow_simulation_trial=True)
  BypassHandoffFollow.__init__=old_init
 elif e['kind']=='reset':old.reset();new.reset()
 elif e['kind']=='call':
  CorridorGuide._plan=old_plan;actual=freeze(replay_call(old,d['input'],mc));matched+=1
  assert equal_output(comparable_decision(actual),comparable_decision(d['output'])),('old',d['input']['reference_s'])
  CorridorGuide._plan=plan;changed=freeze(replay_call(new,d['input'],mc))
  if first_detail is None and not equal_output(comparable_decision(changed),comparable_decision(actual)):
   first_detail=dict(t=d['input']['reference_s'],old=actual,new=changed)
  if not equal_output(changed[:2],actual[:2]):
   first_command=dict(t=d['input']['reference_s'],old=actual,new=changed);break
CorridorGuide._plan=old_plan
out=dict(scope='exact r39 prefix until candidate first changed command; not new driving',matched=matched,
 first_detail_difference=first_detail,first_command_difference=first_command,wall_s=time.perf_counter()-start,
 trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest())
(A/'staged_replay.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print('matched',matched,'detail',first_detail and first_detail['t'],'command',first_command and first_command['t'],flush=True)
