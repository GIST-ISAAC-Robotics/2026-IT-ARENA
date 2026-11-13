from pathlib import Path
import difflib
import hashlib
import json
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent;ROOT=A.parents[4]
relative=Path('tests/test_lidar_safety.py');before=A/'base'/relative;after=ROOT/relative
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
counts=list(ET.parse(A/'pytest.xml').getroot().iter('testsuite'))
assert sum(int(s.get('tests')) for s in counts)==180
assert sum(int(s.get('failures'))+int(s.get('errors')) for s in counts)==0
collected=json.loads((A/'collected.json').read_bytes())
assert collected['source_unchanged'] and not collected['owned_child_pids_remaining'] and collected['pytest_exit']==0
patch=''.join(difflib.unified_diff(before.read_text(encoding='utf-8').splitlines(keepends=True),after.read_text(encoding='utf-8').splitlines(keepends=True),
    fromfile='a/'+relative.as_posix(),tofile='b/'+relative.as_posix()))
(A/'fixture.patch').write_text(patch,encoding='utf-8')
(A/'receipt.json').write_text(json.dumps(dict(path=relative.as_posix(),before_sha256=sha(before),after_sha256=sha(after),
    root_authorized=True,production_source_changes=False,pytest_passed=180,pytest_exit=0,
    source_unchanged_during_run=True,owned_pids_remaining=[],wsl_returned=True,
    initial_failure_preserved='../wsl_verify_v1/pytest.log',constructor_result='../wsl_verify_v1/constructor.json'),indent=2),encoding='utf-8')
print('fixture receipt sealed; 180 passed; WSL returned')
