"""E1 실패 후 접촉 publisher 수 수정만 후속 등록한다."""
import copy
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def write(p, data):
    with p.open('x', encoding='utf-8') as f: json.dump(data,f,ensure_ascii=False,indent=2)

parent_path = 'config/tests/scheduled_command_r5_sha256.json'
parent = json.loads((ROOT/parent_path).read_text())
c = copy.deepcopy(parent)
helper = 'scripts/evaluation_startup.py'
probe = 'scripts/probe_braking_contact.py'
c.update(scope='2026-10-11 readiness correction: exact four wheel contact publishers',
         parent_contract=parent_path,parent_contract_sha256=sha(ROOT/parent_path),
         approved_changes={}, approved_evaluation_scripts={},
         approved_implementation_changes={helper:{'before':parent['implementation_sha256'][helper],
                                                 'after':sha(ROOT/helper),'reason':'four independent wheel contact sensors'}})
c['sha256'][probe] = parent['approved_evaluation_scripts'][probe]['after']
c['implementation_sha256'][helper] = sha(ROOT/helper)
assert sha(ROOT/probe) == c['sha256'][probe]
write(ROOT/'config/tests/scheduled_command_r6_sha256.json',c)
files = [*c['sha256'],*c['implementation_sha256'],'scripts/analyze_initial_startup.py','tests/test_evaluation_startup.py',
         'tests/test_initial_startup_analysis.py','config/tests/scheduled_command_r6_sha256.json']
files += [p.relative_to(ROOT).as_posix() for p in BASE.glob('*') if p.is_file() and p.suffix in ('.md','.py','.sh')]
with zipfile.ZipFile(BASE/'pre_source_r6.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files: z.write(ROOT/p,p)
write(BASE/'pre_manifest_r6.json',{p:sha(ROOT/p) for p in files})
print('r6 source registered; remaining E2 only')
