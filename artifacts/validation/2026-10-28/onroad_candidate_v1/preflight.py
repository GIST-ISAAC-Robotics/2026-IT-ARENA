import json
import compare_saved as probe

with probe.CONFIG_SOURCE.open(encoding='utf-8') as stream:
    header = json.loads(stream.readline())
config = probe.FollowConfig(**header['data']['follow_config'])
keys = ('parallel_hold_trial', 'pass_release_trial', 'guard_curve_trial',
        'traffic_recovery_trial', 'traffic_path_recovery_trial')
assert all(getattr(config, key) for key in keys)
assert all(path.is_file() for path in probe.INPUTS.values())
print('config and input preflight passed')
