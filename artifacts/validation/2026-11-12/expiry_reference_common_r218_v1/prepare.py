"""실행 중 소스는 유지하고 공통화 전 사본/작업 사본을 build에 준비한다."""
from pathlib import Path
from datetime import datetime, timezone
import hashlib
import json
import shutil
import zipfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
package=Path('src/arena_autonomy/arena_autonomy')
changed=[str(package/name) for name in (
    'local_follow.py','decision_process.py','fresh_submission.py',
    'local_pursuit.py','g1_g2_node.py')]
changed+=['src/arena_bringup/launch/g1_follow.launch.py']
hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in changed}
with zipfile.ZipFile(A/'before_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
    for p in changed:archive.write(ROOT/p,p)
for tag in ('r218_before','r218_stage'):
    target=ROOT/'build'/tag
    assert not target.exists(),target
    shutil.copytree(ROOT/package,target/package,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
    for p in changed:
        out=target/p;out.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/p,out)
with (A/'before.json').open('x',encoding='utf8') as stream:
    json.dump(dict(created_at=datetime.now(timezone.utc).isoformat(),files=hashes),stream,indent=2)
    stream.write('\n')
print('Prepared six-file archive and isolated before/stage trees; runtime sources unchanged.')
