"""연결한 r44가 저장 사본과 같고 저속/중속의 기존 명령을 보존하는지 확인."""
import hashlib
import json
from pathlib import Path

import numpy as np
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, freeze, replay_call
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig

A = Path(__file__).resolve().parent
traces = {
    'r32': ('moving_pass_wall_mask_v1', 'r32'),
    'right20': ('faster_onroad_space_v2', 'right20'),
    'recessed_medium': ('medium_recessed_pass_v1', 'medium'),
    'medium': ('medium_pass_connected_v1', None),
    'slow': ('../2026-11-07/traffic_piecewise_v1', None),
}
summary = {}
for label, (folder, probe) in traces.items():
    path = A.parent / folder / 'straight_moving_lead/run1/follow_decisions.jsonl'
    expected = [] if probe is None else [json.loads(line)['new'] for line in
        (A.parent/'faster_entry_audit_v1'/f'{probe}_probe_rows.jsonl').open()]
    count = mismatch = recorded_difference = active = 0
    first_difference = None
    for line in path.open():
        e = json.loads(line); d = e['data']
        if e['kind'] == 'header':
            c = BypassHandoffFollow(FollowConfig(**d['follow_config']),
                HandoffExecutionConfig.from_mapping(d['execution_config']), allow_simulation_trial=True)
            mc = MotionConfig(**d['motion_config'])
        elif e['kind'] == 'reset':
            c.reset()
        elif e['kind'] == 'call':
            out = freeze(replay_call(c, d['input'], mc))
            want = expected[count] if probe else d['output'][:2]
            mismatch += not np.allclose(out[:2], want, atol=1e-10, rtol=0)
            differs = not np.allclose(out[:2], d['output'][:2], atol=1e-10, rtol=0)
            recorded_difference += differs
            if differs and first_difference is None:
                first_difference = dict(t=d['input']['reference_s'], old=d['output'][:2],
                    new=out[:2], reason=out[2].get('reason'))
            active += c.moving_pass.active
            count += 1
    assert probe is None or count == len(expected)
    summary[label] = dict(calls=count, expected_mismatch=mismatch, recorded_difference=recorded_difference,
        first_difference=first_difference, active_calls=active,
        input_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
(A/'replay.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps(summary))
assert all(r['expected_mismatch'] == 0 for r in summary.values())
