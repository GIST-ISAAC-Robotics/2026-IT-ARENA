import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (HERE/'registered.json').exists() and not (HERE/'run1').exists()
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files=sorted({n for n in files if (ROOT/n).is_file()}|{p.relative_to(ROOT).as_posix() for p in HERE.iterdir() if p.suffix in ('.py','.sh','.md','.json')})
with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for n in files:z.write(ROOT/n,n)
r=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sources={n:sha(ROOT/n) for n in files},snapshot_sha256=sha(HERE/'source_snapshot.zip'),scope='new-course one stationary lead, same-race STOP START without RESET or reposition; evaluation only',criteria=dict(actor_s_m=43.26,first_stop_s_m=42.26,resume_permitted_s_max=3.,stop_to_dense_hold_s_max=1.6,hold_s=.5,max_hold_speed_mps=.05,max_hold_drift_m=.02,progress_m_min=1.2,max_off_wheels=2))
(HERE/'registered.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print('registered',len(files))
