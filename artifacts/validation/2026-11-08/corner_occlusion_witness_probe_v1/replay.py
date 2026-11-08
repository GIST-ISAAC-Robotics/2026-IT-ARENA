"""미연결 후보의 연속 저장입력 비교. 변경 뒤 궤적은 실제 주행 결과가 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, freeze, replay_call
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.local_follow import LocalFollow
import arena_autonomy.corner_boundary_support as live
import candidate

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
trace=A.parent/'current_corner_support_v1/run1/follow_decisions.jsonl'
original=LocalFollow._baseline_command
scene_now=None;proofs=[]
def baseline(self,scene,*args,**kw):
    global scene_now
    scene_now=scene
    return original(self,scene,*args,**kw)
def propose(*args,**kw):
    out=candidate.propose(*args,**kw,scene=scene_now)
    proof=out[1].get('far_occlusion_support')
    if proof is not None:
        proofs.append(dict(t=scene_now.t_ref_s,source=scene_now.source_first_s,proof=proof))
    return out
LocalFollow._baseline_command=baseline;live.propose=propose
events=Counter();old_zero=Counter();new_zero=Counter();rows=[];changed=0;first=None
for line in trace.open():
    e=json.loads(line);d=e['data'];events[e['kind']]+=1
    if e['kind']=='header':
        c=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
        mc=MotionConfig(**d['motion_config'])
    elif e['kind']=='reset':c.reset()
    elif e['kind']=='call':
        out=freeze(replay_call(c,d['input'],mc));t=d['input']['reference_s'];old=d['output']
        different=not np.allclose(out[:2],old[:2],atol=1e-10,rtol=0)
        changed+=different
        if different and first is None:first=dict(t=t,old=old[:2],new=out[:2])
        def reason(v):
            q=v[2].get('normal',v[2]);return q.get('reason',v[2].get('reason'))
        if t>=33:
            if old[0]<=0:old_zero[reason(old)]+=1
            if out[0]<=0:new_zero[reason(out)]+=1
        if t>=33 or different:rows.append(dict(t=t,old=old[:2],new=out[:2],old_reason=reason(old),new_reason=reason(out),detail=out[2].get('normal',out[2])))
assert sources=={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
(A/'replay_rows.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
summary=dict(events=events,commands_changed=changed,first_difference=first,old_zero_after33=old_zero,new_zero_after33=new_zero,
             far_support_calls=len(proofs),far_support_scans=len(set(r['source'] for r in proofs)),proofs=proofs,
             source_unchanged=True,trace_sha256=sha(trace),source_sha256=sources)
(A/'replay_summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps({k:v for k,v in summary.items() if k not in ('source_sha256','proofs')}))
