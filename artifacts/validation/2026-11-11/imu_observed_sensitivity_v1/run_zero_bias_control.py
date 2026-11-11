"""같은 IMU 주기/잡음의 영점 오차 0 대조 1회. 기존 실패는 보존한다."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
output = HERE / 'zero_bias_control'
registration = HERE / 'zero_bias_registered.json'
if output.exists() or registration.exists():
    raise SystemExit('control already registered; do not overwrite/retry')
prior = json.loads((HERE / 'registered.json').read_text())
for name, digest in prior['inputs'].items():
    if hashlib.sha256((ROOT/name).read_bytes()).hexdigest() != digest:
        raise SystemExit('original trial input changed')
profile = json.loads((HERE/'motion.json').read_text())
profile['imu']['bias'] = 0.
(HERE/'zero_bias_motion.json').write_text(json.dumps(profile, indent=2)+'\n', encoding='utf-8')
args = list(prior['args'])
args[args.index('--motion-test-profile')+1] = str(HERE/'zero_bias_motion.json')
args[args.index('--output')+1] = str(output)
registration.write_text(json.dumps(dict(args=args, compared_to='solo_run1',
    sole_profile_change='imu.bias from -0.015615217079382257 to 0 rad/s',
    limitations='same configuration except bias; Gazebo noise and scheduling are not a deterministic paired replay',
    input_sha256={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                  for p in (HERE/'sensor.yaml', HERE/'zero_bias_motion.json', Path(__file__))}), indent=2)+'\n')
raise SystemExit(subprocess.call([sys.executable, '-B', *args], cwd=ROOT))
