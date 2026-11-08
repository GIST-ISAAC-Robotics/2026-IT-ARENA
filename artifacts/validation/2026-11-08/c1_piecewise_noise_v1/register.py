import datetime,hashlib,json,subprocess,sys,zipfile
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    old=ROOT/'artifacts/validation/2026-11-08/g1g2_lidar_stress_pair_v1'
    contract=ROOT/'config/tests/piecewise_noisy_corner_r35_sha256.json'
    c=json.loads(contract.read_bytes())
    assert sha(ROOT/c['parent_contract'])==c['parent_contract_sha256']
    assert sha(ROOT/c['before_snapshot'])==c['before_snapshot_sha256']
    for p,v in c['approved_changes'].items():assert sha(ROOT/p)==v['after']
    for name in ('new_track_course.json','actor_overlay.json'):assert sha(HERE/name)==sha(old/name)
    args=json.loads((HERE/'run_args.json').read_bytes())
    expected=json.loads((old/'registered.json').read_bytes())['runs']['B']['args']
    assert args==[s.replace(old.relative_to(ROOT).as_posix(),HERE.relative_to(ROOT).as_posix()) for s in expected]
    files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
    files|={'tests/test_piecewise_noise.py','tests/test_piecewise_wall.py','tests/test_source_provenance_r0_plus_b3.py'}
    files|={(HERE/n).relative_to(ROOT).as_posix() for n in ('PLAN.md','run_args.json','register.py','run.py','run.sh','assess.py','check_runtime.py','new_track_course.json','actor_overlay.json','before.zip')}
    files=sorted(p for p in files if (ROOT/p).is_file())
    runs={'B':dict(folder=(HERE/'B_c1_injection').relative_to(ROOT).as_posix(),args=args)}
    print('sources',len(files),'run B only')
    if '--check' in sys.argv:return
    assert not (HERE/'registered.json').exists() and not (HERE/'source_snapshot.zip').exists() and not (HERE/'B_c1_injection').exists()
    with zipfile.ZipFile(HERE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files:z.write(ROOT/p,p)
    data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='r35 same original C1 B, one changed production module, preregistered case-window scan coverage',
        runs=runs,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(HERE/'source_snapshot.zip'))
    (HERE/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    (HERE/'B_c1_injection').mkdir()
if __name__=='__main__':main()
