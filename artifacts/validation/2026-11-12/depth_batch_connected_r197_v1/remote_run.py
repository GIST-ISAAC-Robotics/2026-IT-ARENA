"""해시 확인한 새 폴더에서 저장 판단만 계산한다. ROS/센서/MCU 포트는 열지 않는다."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

BASE = Path(__file__).resolve().parent
WORK = BASE/'worktree'
TRACE = 'artifacts/validation/2026-11-12/jetson_replay_difference_r194_v1/platform_input.jsonl'
OUT = 'artifacts/validation/2026-11-12/depth_batch_connected_r197_v1/jetson'

def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()

manifest = json.loads((BASE/'bundle_manifest.json').read_text())
assert sha(BASE/'bundle.tar.gz') == manifest['archive_sha256']
WORK.mkdir(exist_ok=False)
with tarfile.open(BASE/'bundle.tar.gz') as archive:
    members = archive.getmembers()
    assert len(members) == len({member.name for member in members})
    assert {member.name for member in members} == set(manifest['files'])
    for member in members:
        target = WORK/member.name
        assert member.isfile() and target.resolve().is_relative_to(WORK.resolve())
        info = manifest['files'][member.name]
        assert member.size == info['size']
        content = archive.extractfile(member).read()
        assert hashlib.sha256(content).hexdigest() == info['sha256']
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open('xb') as output:
            output.write(content)
result = dict(archive_sha256=sha(BASE/'bundle.tar.gz'), members=len(members),
              all_hashes_match=True, scope='recorded_input_cpu_only_no_ros_devices_or_motor_commands')
(BASE/'remote_integrity.json').write_text(json.dumps(result, indent=2)+'\n')
environment = os.environ.copy()
environment['PYTHONPATH'] = ':'.join(str(WORK/path) for path in
    ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'))
environment['PYTHONDONTWRITEBYTECODE'] = '1'
command = [sys.executable, '-B', 'scripts/profile_follow_trace.py', TRACE,
           '--output', OUT, '--repeats', '1', '--warmups', '1', '--input-mode', 'preload']
print(json.dumps(dict(stage='integrity_passed_start_cpu_profile', **result)), flush=True)
with (BASE/'remote_profile.log').open('xb') as log:
    completed = subprocess.run(command, cwd=WORK, env=environment, stdout=log,
                               stderr=subprocess.STDOUT, timeout=600)
result['exit_code'] = completed.returncode
report_path = WORK/OUT/'report.json'
if report_path.is_file():
    report = json.loads(report_path.read_text())
    result.update({key: report.get(key) for key in ('passed', 'baseline_matched', 'timed_calls',
        'error', 'input_unchanged', 'source_unchanged', 'environment')})
    result['all'] = report.get('groups', {}).get('all')
(BASE/'remote_summary.json').write_text(json.dumps(result, indent=2)+'\n')
with tarfile.open(BASE/'results.tar.gz', 'x:gz') as archive:
    for name in ('remote_integrity.json', 'remote_summary.json', 'remote_profile.log'):
        archive.add(BASE/name, arcname=name)
    if (WORK/OUT).is_dir():
        archive.add(WORK/OUT, arcname='jetson')
print(json.dumps(dict(**result, result_archive_sha256=sha(BASE/'results.tar.gz'))), flush=True)
raise SystemExit(completed.returncode)
