"""뒤에서 접근하는 상대의 각본·판정 및 실행 입력을 시작 전에 등록한다."""
import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
old=ROOT/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/registered.json'
args=json.loads(old.read_text())['args']
args[args.index('--config')+1]=(HERE/'new_track_course.json').relative_to(ROOT).as_posix()
args[args.index('--case')+1]='left_rear_overtakes'
args+=['--handoff-wall-witness','--moving-pass-profile','medium_v1']
folder=HERE/'left_rear_overtakes';folder.mkdir(exist_ok=False)
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
helpers=('register.py','run.py','run.sh','assess.py','geometry.py','new_track_course.json','PLAN.md','run_live.sh','run_live_v2.sh')
files=sorted({p for p in files if (ROOT/p).is_file()}|{(HERE/p).relative_to(ROOT).as_posix() for p in helpers})
with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='new course, 6.8s rear overtaker -1.10m, left +0.280m, 100Hz, explicit two wheel centers; r23 controller baseline',args=args,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('registered',len(files))
