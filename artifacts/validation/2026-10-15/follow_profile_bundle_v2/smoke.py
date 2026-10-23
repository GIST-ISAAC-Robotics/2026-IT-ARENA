"""새 임시 폴더에 검증된 자기 묶음을 추출해 실행한다. Jetson/ROS 접속 없음."""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[4]
base = Path(__file__).resolve().parent
archive = ROOT/'build/jetson_g1_profile_20261015_v2.tar.gz'
spec = importlib.util.spec_from_file_location('verify_bundle', ROOT/'scripts/verify_follow_profile_bundle.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
manifest = json.loads((base/'manifest.json').read_text())
assert module.verify(archive, manifest)['passed']
assert not (base/'smoke.json').exists()
extract = Path(tempfile.mkdtemp(prefix='g1-profile-v2-', dir=ROOT/'build'))
with tarfile.open(archive) as bundle:
    # verify() already rejected links, absolute/escaping paths, duplicate and non-files.
    bundle.extractall(extract, filter='data')
env = dict(os.environ, PYTHONPATH=':'.join(str(extract/p) for p in (
    'src/arena_autonomy','src/arena_vehicle_interface','scripts')), PYTHONDONTWRITEBYTECODE='1')
for key in ('AMENT_PREFIX_PATH','COLCON_PREFIX_PATH','CMAKE_PREFIX_PATH','LD_LIBRARY_PATH','ROS_DISTRO'):
    env.pop(key, None)
commands = [
    [sys.executable, '-B', '-c',
     'import arena_autonomy.local_follow as f,arena_autonomy.occupancy_timing as o,sys;'
     'from pathlib import Path;'
     'assert all(Path(m.__file__).is_relative_to(Path.cwd()) for m in (f,o));'
     'assert "rclpy" not in sys.modules;print("isolated import ok")'],
]
trace = extract/'artifacts/validation/2026-10-13/g1_per_ray_v1/side_left_uniform_s0/follow_decisions.jsonl'
commands += [[sys.executable,'-B','scripts/profile_follow_trace.py',str(trace),'--repeats','1','--warmups','0',
              '--output',str(extract/'artifacts/smoke_profile')],
             [sys.executable,'-B','scripts/audit_follow_occupancy.py',str(trace),'--mode','per_ray_candidate',
              '--output',str(extract/'artifacts/smoke_occupancy.json')]]
result = dict(passed=False, extracted_path=str(extract), steps=[], transferred=False,
              remote_execution_verified=False, scope='local_isolated_dependency_smoke_not_cost_or_hardware')
try:
    for index, command in enumerate(commands):
        with (base/f'smoke_{index}.log').open('x') as log:
            completed = subprocess.run(command, cwd=extract, env=env, stdout=log, stderr=subprocess.STDOUT, timeout=180)
        result['steps'].append(dict(command=command, returncode=completed.returncode))
        if completed.returncode:
            raise RuntimeError('isolated step failed')
    result['member_hashes_unchanged'] = all(hashlib.sha256((extract/name).read_bytes()).hexdigest() == entry['sha256']
        for name,entry in manifest['files'].items())
    result['passed'] = result['member_hashes_unchanged']
    for name, path in [('profile','smoke_profile/report.json'),('occupancy','smoke_occupancy.json')]:
        (base/f'isolated_{name}.json').write_bytes((extract/'artifacts'/path).read_bytes())
except Exception as error:
    result['error'] = repr(error)
finally:
    with (base/'smoke.json').open('x') as stream:
        json.dump(result,stream,indent=2)
print(json.dumps(result))
raise SystemExit(0 if result['passed'] else 1)
