"""빌드 뒤 실제 설치 경로·바이트·정책 기본값과 명시 선택 확인."""
import importlib
import json
from pathlib import Path

root = Path.cwd()
names = ('local_follow', 'local_pursuit', 'rear_lane_keep_trial', 'observed_lane_preference_trial',
         'observed_parallel_speed_trial', 'parallel_lane_trial', 'measured_lane_preference_trial',
         'measured_parallel_speed_trial', 'measured_parallel_trial','fresh_parallel_trial','fresh_parallel_speed_trial')
report = {}
for name in names:
    module = importlib.import_module('arena_autonomy.'+name)
    path = Path(module.__file__)
    same = path.read_bytes() == (root/'src/arena_autonomy/arena_autonomy'/f'{name}.py').read_bytes()
    installed = 'install' in path.parts and not path.is_symlink()
    report[name] = dict(path=str(path), matches_source=same, real_installed_file=installed)
    assert same and installed, name
from arena_autonomy.local_follow import FollowConfig, LocalFollow
assert FollowConfig().rear_lane_keep_trial is False
assert FollowConfig().rear_lane_keep_policy == 'timed'
chain = dict(partial_observation_policy='bounded_local_v4', boundary_evidence_mode='corner_distance_candidate',
             occupancy_timing_mode='per_ray_candidate', parallel_hold_trial=True, pass_release_trial=True,
             guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True,
             corridor_fallback_enabled=True, rear_lane_keep_trial=True)
selected = {}
for policy in ('timed', 'observed', 'observed_measured_v1','observed_fresh_v2'):
    follow = LocalFollow(FollowConfig(**chain, rear_lane_keep_policy=policy))
    selected[policy] = dict(selector=type(follow.pass_selector).__module__,
                            parallel=type(follow.parallel_trial).__module__)
assert selected['observed_measured_v1'] == dict(selector='arena_autonomy.measured_lane_preference_trial',
                                                parallel='arena_autonomy.measured_parallel_trial')
assert selected['observed']['parallel'] == selected['timed']['parallel'] == 'arena_autonomy.parallel_lane_trial'
print(json.dumps(dict(modules=report, candidate_default_off=True, default_policy='timed',
                      selected=selected), indent=2))

assert selected['observed_fresh_v2']==dict(selector='arena_autonomy.measured_lane_preference_trial',parallel='arena_autonomy.fresh_parallel_trial')
