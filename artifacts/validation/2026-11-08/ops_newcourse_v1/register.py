import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (HERE/'registered.json').exists() and not (HERE/'run1').exists()
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files=sorted({n for n in files if (ROOT/n).is_file()}|{(HERE/n).relative_to(ROOT).as_posix() for n in ('register.py','run.py','run.sh','assess.py')})
with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for n in files:z.write(ROOT/n,n)
r=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sources={n:sha(ROOT/n) for n in files},snapshot_sha256=sha(HERE/'source_snapshot.zip'),scope='new course RGB start -> short straight -> HTTP STOP -> START without RESET -> 1.2m progress -> STOP; no browser, no teleport, G1 not traffic',criteria=dict(resume_permitted_s_max=3.,stopped_hold_s=.49,stopped_drift_m_max=.02,dense_stop_speed_mps_max=.05,post_resume_progress_m_min=1.2,body_outside_area_m2_max=1e-6,default_world_and_production_unchanged=True))
(HERE/'registered.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print('registered',len(files))
