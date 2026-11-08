from pathlib import Path
import datetime,hashlib,json,subprocess,sys,zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'corner_slowing_actor_v1'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
assert (A/'tests_v1.exit').read_text().strip()=='0'
assert (A/'replay_v1.exit').read_text().strip()=='0'
assert all(v['mismatch']==0 for v in json.loads((A/'replay_summary.json').read_bytes()).values())
contract='config/tests/observed_corner_continuity_r48_sha256.json';c=json.loads((ROOT/contract).read_bytes())
assert sha(ROOT/c['parent_contract'])==c['parent_contract_sha256']
assert sha(ROOT/c['before_snapshot'])==c['before_snapshot_sha256']
for p,d in c['approved_changes'].items():assert sha(ROOT/p)==d['after']
for p,d in c['new_dependencies'].items():assert sha(ROOT/p)==d
for name in ('new_track_course.json','evaluation_source.json','geometry.json'):
    assert (A/name).read_bytes()==(OLD/name).read_bytes(),name
evaluation=json.loads((A/'evaluation_source.json').read_bytes())
for p,h in evaluation.items():assert sha(ROOT/p)==h
g=json.loads((A/'geometry.json').read_bytes())
assert g['initial_ego_actor_gap_m']>=.08 and g['route']['actor_max_off_wheel_centers']<=2
previous=json.loads((OLD/'registered.json').read_bytes());args=list(previous['args'])
args[args.index('--config')+1]=(A/'new_track_course.json').relative_to(ROOT).as_posix()
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files|={p.relative_to(ROOT).as_posix() for p in A.iterdir() if p.suffix in ('.py','.sh','.md','.json','.zip')}
files|=set(evaluation)|{'tests/test_traffic_corner_prefix.py','tests/test_observed_guard_approach.py','tests/test_source_provenance_r0_plus_b3.py','artifacts/validation/2026-11-08/traffic_corner_prefix_probe_v1/fixture.json'}
files=sorted(p for p in files if (ROOT/p).is_file())
print('r48 sources',len(files),'unchanged slow-corner scene and criteria')
if '--check' in sys.argv:raise SystemExit(0)
assert not any((A/n).exists() for n in ('registered.json','source_snapshot.zip','run1'))
with zipfile.ZipFile(A/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(ROOT/p,p)
r=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),scope='r48 current corner prefix and strictly observed additional lateral motion; same36.863s slow corner actor',
    args_source=(OLD/'registered.json').relative_to(ROOT).as_posix(),args=args,contract=contract,
    sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(A/'source_snapshot.zip'),runs={'CASE':dict(folder=A.relative_to(ROOT).as_posix())})
(A/'registered.json').write_text(json.dumps(r,indent=2)+'\n',encoding='utf8')
