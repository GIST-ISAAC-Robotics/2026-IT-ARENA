"""후속 방위축 회귀까지 원본과 구별해 보존. 주행/장치 접근 없음."""
import hashlib
import json
from pathlib import Path
import os
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent/'checks_v2'
OUT.mkdir(exist_ok=False)
FILES = ['scripts/audit_c1_motion_pipeline.py', 'tests/test_c1_motion_pipeline.py',
         'tests/test_lidar_point_review.py', 'tests/test_lidar_motion.py',
         'tests/fixtures/c1_scan_pipeline_stub.cpp',
         'src/arena_autonomy/arena_autonomy/lidar_motion.py',
         'src/arena_autonomy/arena_autonomy/lidar_point_review.py']
hashes = {}
for name in FILES:
    source = ROOT/name
    hashes[name] = hashlib.sha256(source.read_bytes()).hexdigest()
    target = OUT/'source'/name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source, target)
(OUT/'hashes_before.json').write_text(json.dumps(hashes, indent=2))
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONPATH='src/arena_autonomy:scripts')
run = subprocess.run(['python3', '-m', 'pytest', 'tests/test_c1_motion_pipeline.py',
                     'tests/test_lidar_point_review.py', 'tests/test_lidar_motion.py', '-q'],
                    cwd=ROOT, env=env, capture_output=True, text=True, timeout=120)
(OUT/'stdout.txt').write_text(run.stdout)
(OUT/'stderr.txt').write_text(run.stderr)
after = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in FILES}
(OUT/'hashes_after.json').write_text(json.dumps(after, indent=2))
report = dict(test_exit=run.returncode, source_unchanged=hashes == after)
(OUT/'report.json').write_text(json.dumps(report, indent=2))
print(run.stdout)
print(json.dumps(report))
raise SystemExit(0 if run.returncode == 0 and hashes == after else 1)
