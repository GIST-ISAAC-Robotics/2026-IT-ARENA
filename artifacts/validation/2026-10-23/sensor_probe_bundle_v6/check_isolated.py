"""검증한 새 묶음의 독립 순수 회귀. 기존 repo나 과거 묶음을 덮어쓰지 않는다."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/'scripts'))
from verify_sensor_probe_bundle import verify

archive = ROOT/'build/sensor_probe_tools_20261023_v6.tar.gz'
manifest = json.loads((HERE/'manifest.json').read_text())
assert verify(archive, manifest)['passed']
assert not (HERE/'isolated_report.json').exists()
assert not (HERE/'isolated_tests.log').exists()
target = Path(tempfile.mkdtemp(prefix='sensor_probe_v6_isolated_', dir=ROOT/'build')).resolve()
assert target.is_relative_to((ROOT/'build').resolve())
with tarfile.open(archive, 'r:gz') as bundle:
    for member in bundle:
        out = target/member.name
        assert out.resolve().is_relative_to(target)
        out.parent.mkdir(parents=True, exist_ok=True)
        with bundle.extractfile(member) as source, out.open('xb') as dest:
            shutil.copyfileobj(source, dest)
copied = {rel: hashlib.sha256((target/rel).read_bytes()).hexdigest() for rel in manifest['files']}
assert all(copied[rel] == spec['sha256'] for rel, spec in manifest['files'].items())
code = '''
import pathlib,sys
root=pathlib.Path.cwd()
sys.path[:0]=[str(root/'src/arena_vehicle_interface'),str(root/'scripts'),str(root/'tests')]
import sensor_recording_profile, audit_sensor_probe, rosbag_writer_compat
import sensor_probe_preflight, analyze_sensor_probe_timing, inspect_sensor_probe_inputs, sensor_probe_session
from arena_vehicle_interface import bag_contract
for module in (sensor_recording_profile,audit_sensor_probe,rosbag_writer_compat,bag_contract,
               sensor_probe_preflight,analyze_sensor_probe_timing,inspect_sensor_probe_inputs,sensor_probe_session):
    assert pathlib.Path(module.__file__).resolve().is_relative_to(root)
import pytest
raise SystemExit(pytest.main(['-q','-p','no:cacheprovider','tests']))
'''
env = dict(os.environ, PYTHONPATH='', PYTHONDONTWRITEBYTECODE='1')
result = subprocess.run([sys.executable, '-I', '-B', '-c', code], cwd=target,
                        env=env, text=True, capture_output=True, timeout=60)
with (HERE/'isolated_tests.log').open('x') as stream:
    stream.write(result.stdout+result.stderr)
report = dict(passed=result.returncode == 0, exit_code=result.returncode,
              extracted_root=str(target), copied_sha256=copied,
              modules_resolved_inside_bundle=True if result.returncode == 0 else None,
              original_repo_on_pythonpath=False, python_isolated=True,
              test_environment='PC Python 3.12', jetson_verified=False,
              python310_runtime_verified=False, transferred=False, ready_to_drive=False)
with (HERE/'isolated_report.json').open('x') as stream:
    stream.write(json.dumps(report, indent=2)+'\n')
print(json.dumps({k: v for k, v in report.items() if k != 'copied_sha256'}))
raise SystemExit(result.returncode)
