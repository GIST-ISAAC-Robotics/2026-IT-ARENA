"""빌드 뒤 실제 설치 경로·바이트·정책 기본값 확인."""
import hashlib
import importlib
import json
from pathlib import Path

root = Path.cwd()
names = ('local_follow', 'local_pursuit', 'rear_lane_keep_trial',
         'observed_lane_preference_trial', 'observed_parallel_speed_trial')
report = {}
for name in names:
    module = importlib.import_module('arena_autonomy.'+name)
    path = Path(module.__file__)
    same = path.read_bytes() == (root/'src/arena_autonomy/arena_autonomy'/f'{name}.py').read_bytes()
    installed = 'install' in path.parts and not path.is_symlink()
    report[name] = dict(path=str(path), matches_source=same, real_installed_file=installed)
    assert same and installed, name
from arena_autonomy.local_follow import FollowConfig
assert FollowConfig().rear_lane_keep_trial is False
assert FollowConfig().rear_lane_keep_policy == 'timed'
print(json.dumps(dict(modules=report, candidate_default_off=True, default_policy='timed'), indent=2))
