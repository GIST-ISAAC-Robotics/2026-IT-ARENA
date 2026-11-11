"""ROS overlay 누락으로 센서0/주행0인 첫 시도 보존 후 같은 조건 1회 실행."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
old_report = json.loads((HERE/'zero_bias_control/report.json').read_text())
if old_report.get('sensor_delivery', {}).get('imu', {}).get('samples', 0) != 0:
    raise SystemExit('initial attempt had sensor data; do not retry')
output = HERE/'zero_bias_after_overlay'
registration = HERE/'zero_bias_after_overlay_registered.json'
if output.exists() or registration.exists():
    raise SystemExit('already registered; preserve evidence')
prior = json.loads((HERE/'zero_bias_registered.json').read_text())
for name,digest in prior['input_sha256'].items():
    if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
        raise SystemExit('control input changed')
args = list(prior['args'])
args[args.index('--output')+1] = str(output)
registration.write_text(json.dumps(dict(args=args, reason='initial launch failed before Gazebo: ROS overlay not sourced',
    previous='zero_bias_control', changed_trial_conditions=[],
    input_sha256=prior['input_sha256']), indent=2)+'\n')
raise SystemExit(subprocess.call([sys.executable, '-B', *args], cwd=ROOT))
