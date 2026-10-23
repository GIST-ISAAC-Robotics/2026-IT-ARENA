"""독립 묶음과 설치 ROS만으로 기존 MCAP 감사/진단. ROS graph와 장치를 열지 않는다."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
target = Path(json.loads((HERE/'isolated_report.json').read_text())['extracted_root'])
recording = ROOT/'artifacts/validation/2026-10-23/sensor_probe_ros_v9/full_input/sensor_recording'
bad_recording = ROOT/'artifacts/validation/2026-10-23/sensor_probe_writer_faults_v2/write/sensor_recording'
manifest = json.loads((HERE/'manifest.json').read_text())
def input_hashes():
    paths = [p for base in (recording, bad_recording) for p in base.rglob('*') if p.is_file()]
    paths += [target/p for p in manifest['files']]
    return {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
before = input_hashes()
# -I excludes PYTHONPATH; admit installed ROS only, not source checkout/overlay.
ros_paths = [p for p in sys.path if Path(p).is_relative_to('/opt/ros/jazzy')]
assert ros_paths
code = '''
import json,pathlib,runpy,sys
root=pathlib.Path.cwd()
sys.path[:0]=[str(root/'src/arena_vehicle_interface'),str(root/'scripts'),*json.loads(sys.argv[1])]
import rclpy, rosbag2_py, sensor_probe_preflight, audit_sensor_probe, analyze_sensor_probe_timing, inspect_sensor_probe_inputs
for module in (sensor_probe_preflight,audit_sensor_probe,analyze_sensor_probe_timing,inspect_sensor_probe_inputs):
    assert pathlib.Path(module.__file__).resolve().is_relative_to(root)
assert pathlib.Path(rclpy.__file__).is_relative_to('/opt/ros/jazzy')
def forbidden(*args,**kwargs):
    raise AssertionError('no ROS initialization/node permitted in this file-only check')
rclpy.init=rclpy.create_node=forbidden
script=sys.argv[2]
sys.argv=[script,*sys.argv[3:]]
runpy.run_path(str(root/'scripts'/script),run_name='__main__')
'''
cases = [
    ('session_help', 'sensor_probe_session.py', ['--help'], 0),
    ('preflight', 'sensor_probe_preflight.py', ['--capture', str(target/'artifacts/new/sensor_recording'),
        '--clock-mode', 'system', '--sensor-topic', '/camera/color/image_raw',
        '--sensor-topic', '/drive_motor/encoder', '--output', str(HERE/'portable_preflight.json')], 0),
    ('audit', 'audit_sensor_probe.py', [str(recording), '--output', str(HERE/'portable_audit.json')], 0),
    ('timing', 'analyze_sensor_probe_timing.py', [str(recording), '--output', str(HERE/'portable_timing.json')], 0),
    ('inputs', 'inspect_sensor_probe_inputs.py', [str(recording), '--expect-frame', '/scan=laser_frame', '--output', str(HERE/'portable_inputs.json')], 0),
    ('incomplete', 'audit_sensor_probe.py', [str(bad_recording), '--output', str(HERE/'invalid_audit.json')], 1),
]
rows = []
for name, script, args, expected in cases:
    command = [sys.executable, '-I', '-B', '-c', code, json.dumps(ros_paths), script, *args]
    result = subprocess.run(command, cwd=target, env=dict(os.environ, PYTHONPATH=''),
                            text=True, capture_output=True, timeout=60)
    with (HERE/('portable_'+name+'.log')).open('x') as stream:
        stream.write(result.stdout+result.stderr)
    rows.append(dict(case=name, exit_code=result.returncode, expected_exit=expected,
                     passed=result.returncode == expected))
report = dict(passed=all(r['passed'] for r in rows) and before == input_hashes()
              and not (target/'artifacts/new').exists() and not (HERE/'invalid_audit.json').exists(),
              cases=rows, source_and_recordings_unchanged=before == input_hashes(),
              before_sha256=before, ros_paths=ros_paths, original_source_on_pythonpath=False,
              ros_node_created=False, hardware_verified=False, jetson_verified=False,
              transferred=False, ready_to_drive=False)
with (HERE/'portable_runtime_report.json').open('x') as stream:
    json.dump(report, stream, indent=2)
print(json.dumps({k:v for k,v in report.items() if k != 'before_sha256'}))
raise SystemExit(0 if report['passed'] else 1)
