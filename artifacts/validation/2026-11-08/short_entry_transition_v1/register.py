import datetime,hashlib,json,subprocess,sys,zipfile
from pathlib import Path
from r33_tools import R33,TOOLS,verify,rel
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
c=json.loads((ROOT/'config/tests/short_entry_transition_r39_sha256.json').read_bytes())
assert sha(ROOT/c['parent_contract'])==c['parent_contract_sha256']
assert sha(ROOT/c['before_snapshot'])==c['before_snapshot_sha256']
for path,row in c['approved_changes'].items():assert sha(ROOT/path)==row['after']
for path,h in c['new_dependencies'].items():assert sha(ROOT/path)==h
verify()
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files=sorted({n for n in files if (ROOT/n).is_file()}|{p.relative_to(ROOT).as_posix() for p in HERE.iterdir() if p.suffix in ('.py','.sh','.md','.json')}
    |{rel(n) for n in TOOLS}|{(HERE/'before.zip').relative_to(ROOT).as_posix()}
    |{'tests/test_short_entry_transition.py','tests/test_adaptive_entry_preview.py','tests/test_stopped_entry_centering.py','tests/test_source_provenance_r0_plus_b3.py'})
print('sources',len(files),'same r33/r36 OPS')
if '--check' in sys.argv:raise SystemExit(0)
assert not (HERE/'registered.json').exists() and not (HERE/'run1').exists() and not (HERE/'source_snapshot.zip').exists()
with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for n in files:z.write(ROOT/n,n)
r=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sources={n:sha(ROOT/n) for n in files},snapshot_sha256=sha(HERE/'source_snapshot.zip'),
    scope='r39 same r33/r36 stationary lead STOP START without RESET or reposition; earlier initial transition candidate, r38 parent',
    runs={'OPS':dict(folder=HERE.relative_to(ROOT).as_posix())},
    criteria=dict(actor_s_m=43.26,first_stop_s_m=42.26,resume_permitted_s_max=3.,stop_to_dense_hold_s_max=1.6,hold_s=.5,max_hold_speed_mps=.05,max_hold_drift_m=.02,progress_m_min=1.2,max_off_wheels=2),
    r33_tools={rel(n):sha(R33/n) for n in TOOLS})
(HERE/'registered.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
