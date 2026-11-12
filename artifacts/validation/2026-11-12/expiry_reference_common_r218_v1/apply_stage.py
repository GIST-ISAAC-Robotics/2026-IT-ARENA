"""검증 사본을 현재 소스 변경이 없을 때만 적용한다."""
from pathlib import Path
import hashlib,json,shutil
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=json.loads((A/'before.json').read_bytes())['files']
results=json.loads((A/'verify.json').read_bytes())
assert len(results)==3 and all(not r['mismatches'] and not r['recorded_mismatches'] for r in results)
assert '40 passed' in (A/'stage_tests.log').read_text()
for name,digest in before.items():assert sha(ROOT/name)==digest, name
new='src/arena_autonomy/arena_autonomy/expiry_reference.py'
assert not (ROOT/new).exists()
after={}
for name in [*before,new]:
    source=ROOT/'build/r218_stage'/name
    shutil.copy2(source,ROOT/name);after[name]=sha(ROOT/name)
    assert after[name]==sha(source)
test=ROOT/'tests/test_expiry_reference.py';assert not test.exists()
shutil.copy2(A/'test_expiry_reference.py',test)
with (A/'applied.json').open('x') as stream:json.dump(after,stream,indent=2);stream.write('\n')
print('Applied verified seven-file source state; default OFF.')
