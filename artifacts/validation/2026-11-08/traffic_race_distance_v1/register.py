"""r22 run1 등록 인자를 그대로 쓰고 설정·사례·overlay 경로만 바꿔 실행 전에 등록한다."""
import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent; ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
CASE='two_actor_race_distance'
old=ROOT/'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/registered.json'
prior=json.loads(old.read_text(encoding='utf-8'))
args=prior['args']
assert all(sha(ROOT/p)==h for p,h in prior['sources'].items() if p.startswith(('src/','scripts/','config/'))),'controller/evaluation inputs changed since prerequisite'
args[args.index('--config')+1]=(HERE/'new_track_course.json').relative_to(ROOT).as_posix()
args[args.index('--case')+1]=CASE
args[args.index('--actor-overlay')+1]=(HERE/'additional_actors.json').relative_to(ROOT).as_posix()
assert sum(a.startswith('artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1') for a in args)==0
contract='config/tests/traffic_standing_reacquire_r27_sha256.json'
for path,change in json.loads((ROOT/contract).read_text(encoding='utf-8'))['approved_evaluation_scripts'].items():
    assert sha(ROOT/path)==change['after'],path
prerequisite=HERE.parent/'traffic_faster_reacquire_live_v1/two_actor_course_lap/assessment_run1.json'
assert json.loads(prerequisite.read_bytes())['passed'] is True,'60-second faster following not passed'
folder=HERE/CASE;folder.mkdir(exist_ok=False)
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
helpers=('register.py','run.py','run.sh','assess.py','geometry.py','new_track_course.json','PLAN.md','additional_actors.json','course_geometry.py','geometry_source.json')
files=sorted({p for p in files if (ROOT/p).is_file()}|{(HERE/p).relative_to(ROOT).as_posix() for p in helpers})
assert contract in files
with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    scope=f'new course300seconds five-lap distance, two0.8mps actors, after passed60seconds; existing controller ({contract} sha256 {sha(ROOT/contract)}); args from60second run with config/case/overlay only',
    args_source=old.relative_to(ROOT).as_posix(),args=args,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('registered',len(files))
