"""정지차20초·기존 저속 프로필 대표 이관 등록."""
import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
old=ROOT/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/registered.json'
args=json.loads(old.read_text())['args']
args[args.index('--config')+1]=(HERE/'new_track_course.json').relative_to(ROOT).as_posix()
args[args.index('--case')+1]='persistent_block'
args+=['--handoff-wall-witness']
folder=HERE/'persistent_block';folder.mkdir(exist_ok=False)
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
helpers=('register.py','run.py','run.sh','assess.py','goal_eval.py','geometry.py','new_track_course.json')
files=sorted({p for p in files if (ROOT/p).is_file()}|{(HERE/p).relative_to(ROOT).as_posix() for p in helpers})
with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='same stationary central lead for 20s with explicit rear-wall witness and 0.8s rear-face motion observations; raw and bypass gates unchanged',args=args,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('registered',len(files))
