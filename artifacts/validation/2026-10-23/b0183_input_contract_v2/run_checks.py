"""v1의 ROS Python 경로 누락 수정. v1 수집 실패/소스는 그대로 보존한다."""
import hashlib
import json
from pathlib import Path
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
FILES = ['tests/test_b0183_image_handoff.py', 'tests/test_vision_process.py',
         'tests/test_official_vision_pipeline.py', 'tests/test_basic_autonomy.py',
         'scripts/audit_start_signal_layout.py',
         'src/arena_autonomy/arena_autonomy/core.py',
         'src/arena_autonomy/arena_autonomy/vision_process.py',
         'src/arena_autonomy/arena_autonomy/race_start.py']
(OUT/'source').mkdir(exist_ok=False)
before = {}
for name in FILES:
    before[name] = hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
    dest = OUT/'source'/name
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT/name, dest)
(OUT/'hashes_before.json').write_text(json.dumps(before, indent=2))
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1',
    PYTHONPATH='src/arena_autonomy:scripts:tests:'+os.environ.get('PYTHONPATH', ''))
command = ['python3', '-m', 'pytest', *FILES[:4], '-q', '--junitxml='+str(OUT/'junit.xml')]
run = subprocess.run(command, cwd=ROOT, env=env, capture_output=True, text=True, timeout=180)
(OUT/'stdout.txt').write_text(run.stdout)
(OUT/'stderr.txt').write_text(run.stderr)
after = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in FILES}
(OUT/'hashes_after.json').write_text(json.dumps(after, indent=2))
report = dict(test_exit=run.returncode, source_unchanged=before == after,
              hardware_verified=False, ros_verified=False, operational_changes=False,
              environment_fix='source /opt/ros/jazzy/setup.bash and append inherited PYTHONPATH')
(OUT/'report.json').write_text(json.dumps(report, indent=2))
print(run.stdout)
print(json.dumps(report))
raise SystemExit(0 if run.returncode == 0 and before == after else 1)
