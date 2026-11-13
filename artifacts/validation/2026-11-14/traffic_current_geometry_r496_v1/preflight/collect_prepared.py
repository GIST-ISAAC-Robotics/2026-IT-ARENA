from pathlib import Path
import hashlib,json,sys,xml.etree.ElementTree as ET
A=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(A))
from prepare import verify_prepared,sha
prepared=verify_prepared();native=json.loads((A/'native_v1/receipt.json').read_bytes())
pids=[]
for name in ('run.pid','run_attempt1.pid'):
    p=A/'native_v1'/name
    if p.exists():
        pid=int(p.read_text());pids.append(dict(pid=pid,alive=Path('/proc',str(pid)).exists()))
assert not any(p['alive'] for p in pids)
suites=list(ET.parse(A/'native_v1/pytest.xml').iter('testsuite'))
assert sum(int(s.get('tests')) for s in suites)==166
assert sum(int(s.get('failures'))+int(s.get('errors')) for s in suites)==0
assert native['exit_code']==0 and native['production_unchanged'] and native['native_enabled_after']
receipt=dict(inputs=len(prepared['frozen_inputs']),prepared_sha256=sha(A/'prepared.json'),
    source_archive_sha256=sha(A/'sources.zip'),native_tests_passed=166,
    native_receipt_sha256=sha(A/'native_v1/receipt.json'),common_imports=native['imports'],
    r494_normalized_stage_exact=native['normalized_r494_stage_exact'],build_reused=True,
    source_check_passed=True,owned_pid_checks=pids,owned_remaining_pids=[],actual_driving_runs=0,
    registered_present=(A/'registered.json').exists(),root_review_present=(A/'root_execution_review.json').exists(),
    first_native_attempt='Path/string harness error before pytest; original log retained',
    noncontrol_role_note='prepared source_delta keeps legacy generic role label; source_selection.noncontrol_roles is the precise classification')
(A/'preflight/prepared_review.json').write_text(json.dumps(receipt,indent=2),encoding='utf-8')
print(json.dumps(receipt))
