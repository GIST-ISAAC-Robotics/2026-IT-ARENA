"""기존 r7 실행판을 그대로 잠그고 새 노면 대조 2회만 등록한다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
PARENT = ROOT/'artifacts/validation/2026-10-11/scheduled_apply_v1/registered_manifest.json'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
prior = json.loads(PARENT.read_text())
for path, expected in prior['sources'].items():
    if sha(ROOT/path) != expected:
        raise SystemExit(f'registered source changed: {path}')
binary = ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so'
contract = ROOT/'config/tests/scheduled_command_r7_sha256.json'
if sha(binary) != prior['plugin_sha256'] or sha(contract) != prior['contract_sha256']:
    raise SystemExit('r7 binary/contract changed')
files = {**prior['sources'], **{str(p.relative_to(ROOT)): sha(p) for p in
         (BASE/'README.md', BASE/'register.py', BASE/'run.sh', contract)}}
with zipfile.ZipFile(BASE/'registered_source.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
    for path in sorted(files):
        bundle.write(ROOT/path, path)
with (BASE/'registered_manifest.json').open('x') as stream:
    json.dump({'sources': files, 'plugin_sha256': sha(binary), 'contract_sha256': sha(contract),
               'parent_manifest_sha256': sha(PARENT)}, stream, indent=2)
for name, surface in (('s1_official', 'official'), ('f1_flat', 'flat')):
    args = ['--surface', surface, '--zero-speed-mode', 'passive_damping', '--command-schedule',
            'sim_grid', '--schedule-phase-ms', '10', '--command-profile',
            'artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json',
            '--plugin-command-trace', '--evaluation-gc', 'defer', '--evaluation-startup',
            'embedded_paused', '--evaluation-scheduled-apply']
    with (BASE/f'plan_{name}.json').open('x') as stream:
        json.dump({'base': BASE.relative_to(ROOT).as_posix(), 'cases': [
            {'name': name, 'expect': 'pass', 'args': args}]}, stream, indent=2)
print('r7 source/binary unchanged; two surface cases registered')
