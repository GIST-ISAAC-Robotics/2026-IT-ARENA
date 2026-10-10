"""r5 계약과 사전 계획을 배타적으로 생성한다. 원본 코드는 편집하지 않는다."""
import copy
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p, data):
    with p.open('x', encoding='utf-8') as f: json.dump(data, f, ensure_ascii=False, indent=2)

parent_path = 'config/tests/scheduled_command_r4_sha256.json'
parent = json.loads((ROOT/parent_path).read_text())
c = copy.deepcopy(parent)
c.update(scope='2026-10-11 evaluation-only embedded vehicle / paused readiness gate',
         parent_contract=parent_path, parent_contract_sha256=sha(ROOT/parent_path),
         approved_changes={}, approved_evaluation_scripts={}, approved_implementation_changes={})
launch = 'src/arena_bringup/launch/simulation.launch.py'
probe = 'scripts/probe_braking_contact.py'
helper = 'scripts/evaluation_startup.py'
c['approved_changes'][launch] = {'before': parent['sha256'][launch], 'after': sha(ROOT/launch),
                               'reason': 'opt-in embedded/paused; default service path unchanged'}
c['sha256'][launch] = sha(ROOT/launch)
c['approved_evaluation_scripts'][probe] = {'before': c['sha256'].pop(probe), 'after': sha(ROOT/probe)}
c['approved_implementation_changes'][helper] = {'before': None, 'after': sha(ROOT/helper), 'reason': 'evaluation-only readiness gate'}
c['implementation_sha256'][helper] = sha(ROOT/helper)
write(ROOT/'config/tests/scheduled_command_r5_sha256.json', c)
common = ['--surface','official','--zero-speed-mode','passive_damping','--command-schedule','sim_grid',
          '--schedule-phase-ms','10','--command-profile','artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json',
          '--plugin-command-trace','--evaluation-gc','defer']
for name, mode in [('s1_service','service'), ('e1_embedded','embedded_paused'), ('e2_embedded','embedded_paused')]:
    write(BASE/f'plan_{name}.json', {'base': BASE.relative_to(ROOT).as_posix(), 'cases': [
        {'name': name, 'expect':'pass', 'args':common+['--evaluation-startup',mode]}]})
files = list(dict.fromkeys([*c['sha256'], *c['implementation_sha256'], *c['evaluation_tools_sha256'], probe,
                           'config/tests/scheduled_command_r5_sha256.json', 'tests/test_evaluation_startup.py']))
files += [p.relative_to(ROOT).as_posix() for p in BASE.glob('*') if p.is_file()]
with zipfile.ZipFile(BASE/'pre_source.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for p in files: z.write(ROOT/p, p)
write(BASE/'pre_manifest.json', {p:sha(ROOT/p) for p in files})
print('r5 / 3 plans / source snapshot created')
