"""2 m/s 실패 입력의 경로 선택기 전후를 읽는다. 차체는 저장 궤적 그대로다."""
import argparse
import hashlib
import json
from pathlib import Path
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.pass_release_trial import PassReleaseSelector

here = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('phase', choices=['before', 'after'])
phase = parser.parse_args().phase
path = here / 'solo_2p0_race_path/follow_decisions.jsonl'
original = PassReleaseSelector._select
out = {'phase': phase, 'trace_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
       'old_fault_rows': 0, 'new_positive_on_old_fault': 0, 'new_fault_on_old_fault': 0,
       'target_diff_rows': 0, 'first_fault_inputs': None}
sample = {}
def observe(self, scene, baseline, parallel, speed, previous, dt, **kw):
    sample.update(reference_s=scene.t_ref_s, baseline_target=baseline[0], parallel_target=parallel[0],
                  baseline_reason=baseline[2]['reason'], open_route=baseline[2].get('open_route_speed_enabled'),
                  baseline_bound=self.follow_config.max_speed_mps, open_route_bound=self.follow_config.open_route_speed_mps,
                  motor_speed=speed, previous_steer=previous, steering_limit=self.follow_config.max_steering_rad,
                  baseline_steer=baseline[1], parallel_steer=parallel[1], dt=dt)
    return original(self, scene, baseline, parallel, speed, previous, dt, **kw)
PassReleaseSelector._select = observe
for line in path.open():
    e = json.loads(line)
    d = e['data']
    if e['kind'] == 'header':
        ctrl = LocalFollow(FollowConfig(**d['follow_config']))
        motion = MotionConfig(**d['motion_config'])
    elif e['kind'] == 'reset':
        ctrl.reset()
    elif e['kind'] == 'call':
        old = d['output']; new = replay_call(ctrl, d['input'], motion)
        out['target_diff_rows'] += int(abs(old[0]-new[0]) > 1e-9)
        if old[2].get('selector_reason') == 'invalid_motion_or_output':
            out['old_fault_rows'] += 1
            if out['first_fault_inputs'] is None:
                out['first_fault_inputs'] = dict(sample)
            out['new_fault_on_old_fault'] += int(new[2].get('selector_reason') == 'invalid_motion_or_output')
            out['new_positive_on_old_fault'] += int(new[0] > 0.)
assert out['old_fault_rows'] and out['first_fault_inputs']['open_route']
if phase == 'before':
    assert out['target_diff_rows'] == 0 and out['new_fault_on_old_fault'] == out['old_fault_rows']
else:
    assert out['new_positive_on_old_fault'] > 0 and out['new_fault_on_old_fault'] == 0
with (here/f'selector_replay_{phase}.json').open('x') as f:
    json.dump(out, f, indent=2)
print(json.dumps(out, indent=2))
