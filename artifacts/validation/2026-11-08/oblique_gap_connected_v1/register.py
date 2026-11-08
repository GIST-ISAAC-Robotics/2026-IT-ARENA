"""r11 기존 저속 경계 재획득·원래 단차·차량·속도·보호의 새 본선 단독1회 등록."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];REL=HERE.relative_to(ROOT).as_posix()
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (HERE/'registered.json').exists()
entry=['scripts/run_current_solo.py',f'{REL}/run1','--track','official_v2026_11_06',
       '--local-sensor-config','src/arena_description/config/b0390_c1_jr60p20_sim.yaml','--piecewise-wall','--standing-wall-reacquire','--straight-budget','--oblique-gap']
args=json.loads(subprocess.check_output([sys.executable,'-B',*entry,'--print-args'],cwd=ROOT,text=True))
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files=sorted({p for p in files if (ROOT/p).is_file()}|{f'{REL}/register.py',f'{REL}/run.sh',f'{REL}/assess.py',f'{REL}/replay.py'})
with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    contract='config/tests/oblique_gap_r14_sha256.json',scope='same failed new-course straight-budget trial with opt-in oblique gap support only',
    entry=entry,args=args,input_sha256={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(HERE/'source_snapshot.zip'))
(HERE/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('registered',len(files))
