"""이전 r13/r15 기록과의 기존 차이를 r44 자체 회귀와 구분한다."""
import hashlib
import json
import sys
import types
import zipfile
from pathlib import Path
from unittest.mock import patch

import numpy as np
import arena_autonomy.moving_pass as current
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, freeze, replay_call
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig

A = Path(__file__).resolve().parent
with zipfile.ZipFile(A/'before.zip') as z:
    source = z.read('src/arena_autonomy/arena_autonomy/moving_pass.py')
before = types.ModuleType('arena_autonomy._moving_pass_before_r44')
before.__package__ = 'arena_autonomy'
sys.modules[before.__name__] = before
exec(compile(source, '<preserved moving_pass before r44>', 'exec'), before.__dict__)
result = {}
for label, folder in [('medium','medium_pass_connected_v1'), ('slow','../2026-11-07/traffic_piecewise_v1')]:
    path = A.parent/folder/'straight_moving_lead/run1/follow_decisions.jsonl'
    n = mismatch = before_recorded = after_recorded = 0
    for line in path.open():
        e = json.loads(line); d = e['data']
        if e['kind'] == 'header':
            cfg = FollowConfig(**d['follow_config']); ex = HandoffExecutionConfig.from_mapping(d['execution_config'])
            with patch.object(current, 'MovingPassAdapter', before.MovingPassAdapter):
                old = BypassHandoffFollow(cfg, ex, allow_simulation_trial=True)
            new = BypassHandoffFollow(cfg, ex, allow_simulation_trial=True)
            assert type(old.moving_pass) is before.MovingPassAdapter and type(new.moving_pass) is current.MovingPassAdapter
            mc = MotionConfig(**d['motion_config'])
        elif e['kind'] == 'reset': old.reset(); new.reset()
        elif e['kind'] == 'call':
            x = freeze(replay_call(old, d['input'], mc)); y = freeze(replay_call(new, d['input'], mc))
            mismatch += not np.allclose(x[:2], y[:2], atol=1e-10, rtol=0)
            before_recorded += not np.allclose(x[:2], d['output'][:2], atol=1e-10, rtol=0)
            after_recorded += not np.allclose(y[:2], d['output'][:2], atol=1e-10, rtol=0)
            n += 1
    result[label] = dict(calls=n, before_after_mismatch=mismatch,
        before_recorded_difference=before_recorded, after_recorded_difference=after_recorded,
        preserved_before_sha256=hashlib.sha256(source).hexdigest())
(A/'legacy_compare.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
assert all(r['before_after_mismatch']==0 for r in result.values())
