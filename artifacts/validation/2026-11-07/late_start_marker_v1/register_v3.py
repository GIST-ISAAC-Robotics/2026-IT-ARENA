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
assert not (HERE/'registered_v3.json').exists()
entry=[f'{REL}/occlusion_eval_v3.py','--output',f'{REL}/occlusion_run3','--track','official_v2026_11_06',
       '--local-sensor-config','src/arena_description/config/b0390_c1_jr60p20_sim.yaml','--wall-limit-s','600']
args=entry
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files=sorted({p for p in files if (ROOT/p).is_file()}|{f'{REL}/register_v3.py',f'{REL}/run_v3.sh',f'{REL}/occlusion_eval_v3.py'})
with zipfile.ZipFile(HERE/'source_snapshot_v3.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    contract='config/tests/marker_late_start_r12_sha256.json',scope='actual RGB optical occlusion then late visual start with B3; screen is not a full vehicle actor',
    entry=entry,args=args,input_sha256={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(HERE/'source_snapshot_v3.zip'))
(HERE/'registered_v3.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('registered',len(files))
