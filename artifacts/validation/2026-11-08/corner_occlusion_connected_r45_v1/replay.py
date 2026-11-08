"""실행 연결 명령을 사본/직전 주행과 비교한다. 실차 또는 폐루프 검사가 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import FollowConfig,MotionConfig,replay_call,freeze
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig

A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
expected={r['t']:r['new'] for r in map(json.loads,(BASE/'corner_occlusion_witness_probe_v1/replay_rows.jsonl').read_text().splitlines())}
cases={'corner':BASE/'current_corner_support_v1/run1/follow_decisions.jsonl',
       'faster_follow':BASE/'faster_entry_consistency_r44_v1/straight_moving_lead/run1/follow_decisions.jsonl',
       'recessed_medium':BASE/'medium_recessed_pass_v1/straight_moving_lead/run1/follow_decisions.jsonl'}
summary={}
for name,trace in cases.items():
    calls=mismatch=0;first=None;far_calls=0
    for line in trace.open():
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            c=BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
            mc=MotionConfig(**d['motion_config'])
        elif e['kind']=='reset':c.reset()
        elif e['kind']=='call':
            out=freeze(replay_call(c,d['input'],mc));t=d['input']['reference_s'];calls+=1
            target=expected.get(t,d['output'][:2]) if name=='corner' else d['output'][:2]
            bad=not np.allclose(out[:2],target,atol=1e-10,rtol=0);mismatch+=bad
            if bad and first is None:first=dict(t=t,expected=target,actual=out[:2])
            normal=out[2].get('normal',out[2]);proof=normal.get('current_corner_support',{}).get('far_occlusion_support')
            if proof:
                far_calls+=1;assert 'foreground_source_indices' not in proof
    summary[name]=dict(calls=calls,mismatch=mismatch,first_mismatch=first,far_evidence_calls=far_calls,trace_sha256=sha(trace))
    print(name,json.dumps(summary[name]),flush=True)
assert sources=={str(p.relative_to(ROOT)):sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
(A/'replay_summary.json').write_text(json.dumps(summary,indent=2))
assert all(v['mismatch']==0 for v in summary.values())
