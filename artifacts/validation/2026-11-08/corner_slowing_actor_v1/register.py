import datetime,hashlib,json,subprocess,sys,zipfile
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'current_corner_fallback_r47_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert (A/'prepare_v1.exit').read_text().strip()=='0'
previous=json.loads((OLD/'registered.json').read_bytes())
for path,h in previous['sources'].items():
    if path.startswith(('src/','scripts/','config/')):assert sha(ROOT/path)==h,path
assert (OLD/'tests_v1.exit').read_text().strip()=='0'
assert all(v['mismatch']==0 for v in json.loads((OLD/'replay_summary.json').read_bytes()).values())
old=json.loads((OLD/'new_track_course.json').read_bytes());new=json.loads((A/'new_track_course.json').read_bytes())
old['scope']=new['scope']
for key in ('duration_s','schedule','expected'):old['cases'][0][key]=new['cases'][0][key]
assert old==new,'Only the declared actor schedule/duration and explanation may differ'
assert (OLD/'evaluation_source.json').read_bytes()==(A/'evaluation_source.json').read_bytes()
evaluation=json.loads((A/'evaluation_source.json').read_bytes())
for path,h in evaluation.items():assert sha(ROOT/path)==h
geometry=json.loads((A/'geometry.json').read_bytes())
assert geometry['initial_ego_actor_gap_m']>=.08 and geometry['route']['actor_max_off_wheel_centers']<=2
args=list(previous['args']);args[args.index('--config')+1]=(A/'new_track_course.json').relative_to(ROOT).as_posix()
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files|={p.relative_to(ROOT).as_posix() for p in A.iterdir() if p.suffix in ('.py','.sh','.json','.md')}
files|=set(evaluation)
files=sorted(p for p in files if (ROOT/p).is_file())
print('New slow-corner actor; unchanged r47 controller:',len(files),'inputs')
if '--check' in sys.argv:raise SystemExit(0)
assert not any((A/n).exists() for n in ('registered.json','source_snapshot.zip','run1'))
with zipfile.ZipFile(A/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
r=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    scope='new fixed slow-corner actor; r47 controller unchanged; original32s failure preserved',
    args_source=(OLD/'registered.json').relative_to(ROOT).as_posix(),args=args,
    sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(A/'source_snapshot.zip'),
    runs={'CASE':dict(folder=A.relative_to(ROOT).as_posix())})
(A/'registered.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
