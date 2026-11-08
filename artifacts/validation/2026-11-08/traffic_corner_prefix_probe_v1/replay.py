from collections import Counter
from pathlib import Path
import hashlib
import json
import numpy as np
from arena_autonomy.follow_trace import FollowConfig,MotionConfig,replay_call,freeze
from arena_autonomy.local_follow import LocalFollow
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from loader import load
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
m=load();LocalFollow._baseline_command=m.LocalFollow._baseline_command
trace=A.parent/'corner_slowing_actor_v1/run1/follow_decisions.jsonl'
rows=[];changed=0;first=None;old_zero=Counter();new_zero=Counter();reasons=Counter();calls=0
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        c=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
        mc=MotionConfig(**d['motion_config'])
    elif e['kind']=='reset':c.reset()
    elif e['kind']=='call':
        out=freeze(replay_call(c,d['input'],mc));old=d['output'];t=d['input']['reference_s'];calls+=1
        bad=not np.allclose(out[:2],old[:2],atol=1e-10,rtol=0);changed+=bad
        if bad and first is None:first=dict(t=t,old=old[:2],new=out[:2])
        n=out[2].get('normal',out[2]);o=old[2].get('normal',old[2])
        if t>=33:
            if old[0]<=0:old_zero[str((o.get('reason'),o.get('baseline_reason')))]+=1
            if out[0]<=0:new_zero[str((n.get('reason'),n.get('baseline_reason')))]+=1
        if n.get('current_corner_prefix_agreement'):reasons[n['current_corner_prefix_agreement']]+=1
        rows.append(dict(t=t,old=old[:2],new=out[:2],reason=n.get('reason'),baseline=n.get('baseline_reason'),
            agreement=n.get('current_corner_prefix_agreement'),boundary=n.get('boundary_selection')))
assert source=={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
summary=dict(calls=calls,changed=changed,first_difference=first,old_zero_after33=old_zero,new_zero_after33=new_zero,
    retained_agreement_calls=reasons,source_unchanged=True,trace_sha256=sha(trace),source_sha256=source)
(A/'replay_summary.json').write_text(json.dumps(summary,indent=2))
(A/'replay_rows.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
print(json.dumps({k:v for k,v in summary.items() if k!='source_sha256'}))
