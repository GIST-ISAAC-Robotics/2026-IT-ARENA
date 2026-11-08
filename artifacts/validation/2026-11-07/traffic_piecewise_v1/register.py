"""같은 저속 추월14초에 기존 직각 옵션만 추가하는 실행 등록."""
import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
old=HERE.parent/'steady_pass_newcourse_v1/straight_moving_lead/registered.json'
args=json.loads(old.read_text())['args']
args[args.index('--config')+1]=(HERE/'new_track_course.json').relative_to(ROOT).as_posix()
args+=['--g1-piecewise-wall']
folder=HERE/'straight_moving_lead';folder.mkdir(exist_ok=False)
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files=sorted({p for p in files if (ROOT/p).is_file()}|{p.relative_to(ROOT).as_posix() for p in HERE.iterdir() if p.is_file()})
with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='same steady low-speed pass with existing piecewise walls, original gates unchanged',args=args,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('registered',len(files))
