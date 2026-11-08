"""실제 명령을 바꾸지 않고 벽 반환 분류 후보의 발생 위치만 관찰한다."""
from pathlib import Path
import hashlib
import json
import weakref
import numpy as np
from candidate import WallReturnMemory
from arena_autonomy.follow_trace import MotionConfig, FollowConfig, replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.moving_pass import MovingPassGuide

A=Path(__file__).resolve().parent
trace=A.parent/'moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl'
states=weakref.WeakKeyDictionary(); histories=weakref.WeakKeyDictionary(); events=[]
old_eval=MovingPassGuide.evaluate; old_reset=MovingPassGuide.reset; old_pad=MovingPassGuide._faster_outer_padding
def evaluate(self,scene,history,*args,**kw):
    histories[self]=history
    return old_eval(self,scene,history,*args,**kw)
def reset(self):
    states.pop(self,None)
    return old_reset(self)
def padding(self,scene,config,side,wall,growth=0.):
    original,evidence=old_pad(self,scene,config,side,wall,growth)
    state=states.setdefault(self,WallReturnMemory())
    comparison=state.compare(scene,config,side,wall,histories.get(self),self.raw_scan,self.sectors)
    if comparison['accepted']:
        ids=np.asarray([r.source_index for r in scene.rays if r.status=='return'])
        mask=np.isin(ids,comparison['current_indices'])&(original>.025+config.measurement_padding_m+1e-12)
        if mask.any():events.append(dict(t=scene.t_ref_s,source=scene.source_first_s,side=side,
            proposed_indices=ids[mask].tolist(),comparison=comparison))
    return original,evidence  # shadow only; actual source and decisions unchanged
MovingPassGuide.evaluate=evaluate;MovingPassGuide.reset=reset;MovingPassGuide._faster_outer_padding=padding
matched=total=0;mismatch=[]
try:
    for line in trace.open():
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            mc=MotionConfig(**d['motion_config'])
            engine=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
        elif e['kind']=='reset':engine.reset()
        elif e['kind']=='call':
            actual=freeze(replay_call(engine,d['input'],mc));total+=1
            if equal_output(comparable_decision(actual),comparable_decision(d['output'])):matched+=1
            else:mismatch.append(d['input']['reference_s'])
finally:
    MovingPassGuide.evaluate=old_eval;MovingPassGuide.reset=old_reset;MovingPassGuide._faster_outer_padding=old_pad
out=dict(scope='shadow current-return classification only; actual padding and commands unchanged',
    total=total,historical_matches=matched,mismatch_times=mismatch,events=events,
    distinct_sources=len({e['source'] for e in events}),
    trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
    candidate_sha256=hashlib.sha256((A/'candidate.py').read_bytes()).hexdigest())
(A/'shadow_result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in out.items() if k not in ('events','mismatch_times')},ensure_ascii=False),flush=True)
