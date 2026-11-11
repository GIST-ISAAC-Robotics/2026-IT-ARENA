"""등록한 선택형 IMU 조건의 단독 1회. 기존 결과가 있으면 거절한다."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from run_new_course_solo import preset_args

output = HERE / 'solo_run1'
if output.exists() or (HERE / 'registered.json').exists():
    raise SystemExit('registered trial already exists; do not overwrite/retry')
args = preset_args('new-course-solo', output)
args.remove('--video')
args[args.index('--local-sensor-config') + 1] = str(HERE / 'sensor.yaml')
args.extend(['--motion-test-profile', str(HERE / 'motion.json')])
inputs = [HERE / name for name in ('sensor.yaml', 'motion.json', 'preflight.json', 'run_once.py')]
registration = dict(args=args, independent_variables=['IMU rate', 'IMU Gaussian scatter', 'assumed z gyro bias'],
    scope='one bounded solo simulation; no hardware or timing qualification; historical run is not a paired A/B',
    inputs={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
(HERE / 'registered.json').write_text(json.dumps(registration, indent=2)+'\n', encoding='utf-8')
raise SystemExit(subprocess.call([sys.executable, '-B', *args], cwd=ROOT))
