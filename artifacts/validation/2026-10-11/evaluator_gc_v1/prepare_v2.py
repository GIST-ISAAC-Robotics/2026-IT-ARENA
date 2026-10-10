"""계측 순서만 고친 r4와 수정본 사전 소스를 동결한다."""
import json
import zipfile
from prepare import BASE, ROOT, sha, write

parent = ROOT/'config/tests/scheduled_command_r3_sha256.json'
contract = json.loads(parent.read_text(encoding='utf-8'))
contract.update(parent_contract=parent.relative_to(ROOT).as_posix(), parent_contract_sha256=sha(parent),
                scope='2026-10-11 GC scope correction only: disable before diagnostic allocation/RSS first import')
probe = contract['approved_evaluation_scripts'].pop('scripts/probe_braking_contact.py')
contract['sha256']['scripts/probe_braking_contact.py'] = probe['after']
helper = 'scripts/evaluation_gc.py'
before = contract['implementation_sha256'][helper]
contract['implementation_sha256'][helper] = sha(ROOT/helper)
contract['approved_implementation_changes'] = {helper: {'before': before, 'after': sha(ROOT/helper),
    'reason': 'defer before RSS import/diagnostic allocation; startup restoration tested'}}
path = ROOT/'config/tests/scheduled_command_r4_sha256.json'
write(path, contract)
files = [ROOT/p for p in {**contract['sha256'], **contract['implementation_sha256']}]
files += [ROOT/'tests/test_evaluation_gc.py', ROOT/'tests/test_evaluation_gc_analysis.py',
          ROOT/'scripts/analyze_evaluator_gc.py', BASE/'ADDENDUM.md', BASE/'prepare_v2.py', path]
with zipfile.ZipFile(BASE/'pre_source_v2.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for p in files:
        z.write(p, p.relative_to(ROOT).as_posix())
write(BASE/'pre_manifest_v2.json', {'sha256': {p.relative_to(ROOT).as_posix(): sha(p) for p in files}})
