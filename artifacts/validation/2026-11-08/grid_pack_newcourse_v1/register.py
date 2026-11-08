"""뒤에서 접근하는 상대의 각본·판정 및 실행 입력을 시작 전에 등록한다."""
import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
old=ROOT/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/registered.json'
args=json.loads(old.read_text())['args']
args[args.index('--config')+1]=(HERE/'new_track_course.json').relative_to(ROOT).as_posix()
args[args.index('--case')+1]='five_vehicle_grid'
args+=['--handoff-wall-witness','--moving-pass-profile','medium_v1','--actor-overlay',(HERE/'additional_actors.json').relative_to(ROOT).as_posix()]
folder=HERE/'five_vehicle_grid';folder.mkdir(exist_ok=False)
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
helpers=('register.py','run.py','run.sh','assess.py','geometry.py','new_track_course.json','PLAN.md','run_live.sh','additional_actors.json')
files=sorted({p for p in files if (ROOT/p).is_file()}|{(HERE/p).relative_to(ROOT).as_posix() for p in helpers})
with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='new official grid 3 ego plus 4 actors in slots 4/0/1/2, 0.5s wait then 0.5m/s for 8s; 180mm assumed actor height',args=args,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('registered',len(files))
