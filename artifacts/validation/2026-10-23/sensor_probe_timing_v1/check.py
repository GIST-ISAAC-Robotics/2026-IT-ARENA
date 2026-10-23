"""고정 단위/저장 CDR 검사. 새 ROS 노드나 시뮬레이션을 시작하지 않는다."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
FILES = ['scripts/audit_sensor_probe.py', 'scripts/analyze_sensor_probe_timing.py',
         'scripts/audit_sensor_bag.py', 'scripts/sensor_recording_profile.py',
         'tests/test_sensor_probe_audit.py', 'tests/test_sensor_probe_timing.py',
         'src/arena_vehicle_interface/arena_vehicle_interface/bag_contract.py',
         'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py']
TESTS = ['tests/test_sensor_probe_timing.py', 'tests/test_sensor_probe_audit.py',
         'tests/test_sensor_recording_profile.py', 'tests/test_bag_contract.py',
         'tests/test_sensor_probe_bundle.py', 'tests/test_bag_reader_compat.py']
before = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in FILES}
for rel in FILES:
    dest = HERE/'source_snapshot'/rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    with dest.open('xb') as output, (ROOT/rel).open('rb') as source:
        shutil.copyfileobj(source, output)
with (HERE/'hashes_before.json').open('x') as stream:
    json.dump(before, stream, indent=2)
with (HERE/'tests.log').open('x') as log:
    test = subprocess.run([sys.executable, '-B', '-m', 'pytest', '-q', '-p', 'no:cacheprovider',
                           *TESTS, '--junitxml='+str(HERE/'junit.xml')], cwd=ROOT, stdout=log, stderr=log)
cases = []
if test.returncode == 0:
    for label, original in [('system', 'sensor_probe_ros_v6/system'),
                             ('simulation', 'sensor_probe_ros_v4/sim_valid')]:
        capture = ROOT/'artifacts/validation/2026-10-23'/original/'sensor_recording'
        before_input = {str(p.relative_to(capture)): hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in capture.rglob('*') if p.is_file()}
        with (HERE/(label+'.log')).open('x') as log:
            result = subprocess.run([sys.executable, '-B', 'scripts/analyze_sensor_probe_timing.py',
                                     str(capture), '--output', str(HERE/(label+'.json'))],
                                    cwd=ROOT, stdout=log, stderr=log)
        after_input = {str(p.relative_to(capture)): hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in capture.rglob('*') if p.is_file()}
        cases.append(dict(name=label, exit_code=result.returncode,
                          input_unchanged=before_input == after_input, original_input_sha256=before_input))
after = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in FILES}
passed = test.returncode == 0 and before == after and all(c['exit_code'] == 0 and c['input_unchanged'] for c in cases)
report = dict(passed=passed, tests_exit=test.returncode, cases=cases, source_unchanged=before == after,
              new_ros_nodes_started=False, simulation_started=False, jetson_verified=False,
              acquisition_timing_verified=False, ready_to_drive=False)
with (HERE/'report.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps({k: v for k, v in report.items() if k != 'cases'}))
raise SystemExit(0 if passed else 1)
