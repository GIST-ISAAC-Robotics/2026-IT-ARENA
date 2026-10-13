"""이전 실행판 및 새 wrapper를 실행 전 동결한다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
parent = ROOT/'artifacts/validation/2026-10-11/scheduled_apply_v1/registered_manifest.json'
prior = json.loads(parent.read_text())
report = ROOT/'artifacts/validation/2026-10-11/scheduled_apply_v1/t2_scheduled/report.json'
if sha(report) != '359bb526edd60f3eaf15a7e9c1ba6ec6ad5efda9d4d611d4c9976d69a31123be':
    raise SystemExit('prior report changed')
sources = {**prior['sources'], **json.loads(report.read_text())['source_sha256']}
for path, expected in sources.items():
    if sha(ROOT/path) != expected:
        raise SystemExit('prior source changed: '+path)
binary = ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so'
contract = ROOT/'config/tests/scheduled_command_r7_sha256.json'
if sha(binary) != prior['plugin_sha256'] or sha(contract) != prior['contract_sha256']:
    raise SystemExit('r7 binary/contract changed')
extra = [ROOT/'scripts/probe_solver_compare.py', ROOT/'tests/test_solver_compare.py',
         *[BASE/name for name in ('README.md', 'register.py', 'run.sh', 'execute.py')]]
sources.update({p.relative_to(ROOT).as_posix(): sha(p) for p in extra})
with zipfile.ZipFile(BASE/'registered_source.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for name in sources:
        z.write(ROOT/name, name)
with (BASE/'registered_manifest.json').open('x') as f:
    json.dump({'sources': sources, 'plugin_sha256': sha(binary), 'contract_sha256': sha(contract),
               'parent_manifest_sha256': sha(parent), 'prior_report_sha256': sha(report)}, f, indent=2)
for name, solver in (('d1_dantzig', 'dantzig'), ('p1_pgs', 'pgs')):
    with (BASE/f'plan_{name}.json').open('x') as f:
        json.dump({'base': BASE.relative_to(ROOT).as_posix(), 'cases': [
            {'name': name, 'expect': 'pass', 'args': ['--solver', solver]}]}, f, indent=2)
print('r7 runtime/probe unchanged; solver comparison registered')
