"""중속 상대0.50/자차0.90 명시 프로필14초 실행 등록."""
import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
old=ROOT/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/registered.json'
args=json.loads(old.read_text())['args']
args[args.index('--config')+1]=(HERE/'new_track_course.json').relative_to(ROOT).as_posix()
args+=['--moving-pass-profile','medium_v1']
folder=HERE/'straight_moving_lead';folder.mkdir(exist_ok=False)
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
helpers=('register.py','run.py','run.sh','assess.py','new_track_course.json')
files=sorted({p for p in files if (ROOT/p).is_file()}|{(HERE/p).relative_to(ROOT).as_posix() for p in helpers})
with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='explicit medium profile, lead 0.50m/s for 14s, original raw gates preserved alongside registered passing gates',args=args,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('registered',len(files))
