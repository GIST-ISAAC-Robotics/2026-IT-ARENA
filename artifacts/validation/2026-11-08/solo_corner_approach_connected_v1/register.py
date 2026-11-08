import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
parent=ROOT/'artifacts/validation/2026-11-08/oblique_gap_connected_v1/registered.json'
args=json.loads(parent.read_text())['args'];args[args.index('--output')+1]=str(HERE/'run1');args.append('--g1-solo-corner-approach')
assert not (HERE/'registered.json').exists() and not (HERE/'run1').exists()
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files=sorted({n for n in files if (ROOT/n).is_file()}|{(HERE/n).relative_to(ROOT).as_posix() for n in ('register.py','run.py','run.sh','.gitignore')})
with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for n in files:z.write(ROOT/n,n)
r=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),args=args,sources={n:sha(ROOT/n) for n in files},snapshot_sha256=sha(HERE/'source_snapshot.zip'),prior_registration_sha256=sha(parent),scope='same straight-budget/oblique-gap/wheel-contact instrumentation and original surface with explicit corner-approach speed cap only; not an open-loop contrast',new_measurement=['/eval/wheels/contact','/model/arena_car/drivetrain'],criteria='retain original lap, surface, STOP and shutdown criteria; inspect contacts for wheel/body disagreement without treating Gazebo collision-name order as force orientation proof')
(HERE/'registered.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print('registered',len(files))
