"""r36 등록. 후보 5모듈과 연결 2파일이 src에 적용된 뒤에만 기록한다. 실행 전 GPT 검토 대상."""
import datetime,hashlib,json,subprocess,zipfile
from pathlib import Path
from r33_tools import R33,TOOLS,verify,rel
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert not (HERE/'registered.json').exists() and not (HERE/'run1').exists()
manifest=json.loads((HERE/'wiring_manifest.json').read_text(encoding='utf-8'))
expected={**manifest['candidate_overlay']['after'],**manifest['after']}
missing=[p for p,h in expected.items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
if missing:raise SystemExit('candidate/wiring not applied to src: '+', '.join(missing))
verify()
files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
files=sorted({n for n in files if (ROOT/n).is_file()}|{p.relative_to(ROOT).as_posix() for p in HERE.iterdir() if p.suffix in ('.py','.sh','.md','.json')}
             |{rel(n) for n in TOOLS}|{'tests/test_adaptive_entry_preview.py','tests/test_stopped_entry_centering.py','tests/test_source_provenance_r0_plus_b3.py'})
with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for n in files:z.write(ROOT/n,n)
r=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),sources={n:sha(ROOT/n) for n in files},snapshot_sha256=sha(HERE/'source_snapshot.zip'),
    scope='r36: same r33 stationary-lead STOP START without RESET or reposition; evaluation only',
    criteria=dict(actor_s_m=43.26,first_stop_s_m=42.26,resume_permitted_s_max=3.,stop_to_dense_hold_s_max=1.6,hold_s=.5,max_hold_speed_mps=.05,max_hold_drift_m=.02,progress_m_min=1.2,max_off_wheels=2),
    differences_from_r33=['launch evaluation_adaptive_entry_preview:=true (new candidate, stopped centering stays ON)',
        'source parent includes r35 piecewise_wall.py noise split; g1_piecewise_wall_trial is ON in r33 flags, so a change vs r33 can come from either'],
    applied=expected,r33_tools={rel(n):sha(R33/n) for n in TOOLS})
(HERE/'registered.json').write_text(json.dumps(r,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print('registered',len(files))
