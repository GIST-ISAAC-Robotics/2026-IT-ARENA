"""승인한 r486/r489 변경만 r485 단독 변형의 새 source로 동결한다."""
import argparse,json,zipfile
from pathlib import Path
from prepare import A,ROOT,CASES
from variation_launch import sha,write_new

OLD=A.parent/'solo_variation_recovery_r485_v1'
NOMINAL=A.parent/'solo_full_lap_r475_v1'
def read(path):return json.loads(path.read_bytes())
def case_dir(name):
    assert name in CASES
    return A/'cases'/name

def arguments(name):
    args=read(OLD/'execution_prepared.json')['args'][name]
    old=OLD.relative_to(ROOT).as_posix();new=A.relative_to(ROOT).as_posix()
    return [word.replace(old,new) for word in args]

def nominal_success():
    report=read(NOMINAL/'run1/report.json')
    assert report['passed'] and report['shutdown_clean'] and report['evaluation']['gates']['full_lap']
    assert not read(NOMINAL/'execution_v1/owned_processes_after.json')['remaining_owned_work']
    assert not read(OLD/'cases/start_left20_yaw_plus3/execution_v1/owned_processes_after.json')['remaining_owned_work']
    assert read(OLD/'cases/start_left20_yaw_plus3/run1/report.json')['passed'] is False
    return True

def verify_connection():
    selection=read(A/'source_selection.json')
    assert selection['common_integration_confirmed'] and selection['freeze_authorized'] and selection['wsl_owner_confirmed']
    original=read(OLD/'execution_prepared.json')
    assert sha(OLD/'execution_sources.zip')==original['source_archive_sha256']
    allowed={r['path']:r for r in selection['policy_changes']}
    common={p:h for p,h in original['frozen_inputs'].items()
        if p.startswith(('src/','scripts/','tests/','config/')) or 'old_signal_610_r326_v1/' in p}
    assert sum(r['role']=='production' for r in allowed.values())==7
    assert sum(r['role']=='fixture' for r in allowed.values())==1
    selected={};delta=[]
    for p,h in common.items():
        current=sha(ROOT/p)
        if p in allowed:
            assert h==allowed[p]['before_sha256'] and current==allowed[p]['after_sha256'],p
        else:
            assert current==h,p
        if current!=h:delta.append(dict(path=p,before_sha256=h,after_sha256=current,
            role=allowed[p]['role'],origin=allowed[p]['origin']))
        selected[p]=current
    for p,r in allowed.items():
        if p in common:continue
        assert r['before_sha256'] is None,p
        current=sha(ROOT/p);assert current==r['after_sha256'],p
        selected[p]=current
        delta.append(dict(path=p,before_sha256=None,after_sha256=current,role=r['role'],origin=r['origin']))
    assert sum(x['role']=='production' for x in delta)==7
    for p,h in selection['integration_receipts'].items():
        assert sha(ROOT/p)==h,p
        selected[p]=h
    native='src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'
    assert selected[native]==selection['expected_native_sha256']
    oldcfg=read(OLD/'config.json');newcfg=read(A/'config.json')
    newcfg['scope']=oldcfg['scope'];newcfg['launch']['entry_g1']=oldcfg['launch']['entry_g1']
    assert oldcfg==newcfg
    assert sha(A/'expected_header.json')==sha(OLD/'expected_header.json')
    assert sha(A/'connection_v1/start_left20_yaw_plus3.sdf')==sha(OLD/'connection_v1/start_left20_yaw_plus3.sdf')
    assert read(A/'prepared/manifest.json')==read(OLD/'prepared/manifest.json')
    nominal_success()
    return dict(source_delta=delta,args={n:arguments(n) for n in CASES}),selected

def freeze():
    value,selected=verify_connection()
    assert read(A/'runtime_preflight/result.json')['passed']
    for p in [*A.glob('*.py'),*A.glob('*.json'),*A.glob('*.sdf'),
              *(A/'launch').glob('*.py'),*(A/'execution').glob('*'),
              A/'connection_v1/start_left20_yaw_plus3.sdf',A/'prepared/manifest.json',
              A/'runtime_preflight/result.json',OLD/'execution_prepared.json',OLD/'execution_sources.zip',
              OLD/'cases/start_left20_yaw_plus3/run1/report.json',
              OLD/'cases/start_left20_yaw_plus3/execution_v1/owned_processes_after.json',
              NOMINAL/'run1/report.json',NOMINAL/'execution_v1/owned_processes_after.json']:
        if p.is_file():selected[p.relative_to(ROOT).as_posix()]=sha(p)
    with zipfile.ZipFile(A/'execution_sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in selected:
            if not p.endswith('.zip'):z.write(ROOT/p,p)
    prepared=dict(frozen_inputs=selected,args=value['args'],source_delta=value['source_delta'],
        source_archive_sha256=sha(A/'execution_sources.zip'),actor_count=0,facilities=3,
        execution_limit_per_case=1,repeat_on_failure=False,root_execution_review_required=True,
        source_case='r485 start_left20_yaw_plus3; independent-guard failure preserved; r486/r489 integration only',
        max_sim_seconds=80,no_progress_stop_sim_seconds=8,real_time_factor=.25)
    write_new(A/'execution_prepared.json',prepared)
    return dict(frozen=True,inputs=len(selected),prepared_sha256=sha(A/'execution_prepared.json'),
        source_archive_sha256=prepared['source_archive_sha256'],source_delta=value['source_delta'])

def verify_prepared():
    value=read(A/'execution_prepared.json')
    changed=[p for p,h in value['frozen_inputs'].items() if sha(ROOT/p)!=h]
    assert not changed,changed
    assert sha(A/'execution_sources.zip')==value['source_archive_sha256']
    return value

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--freeze',action='store_true');p.add_argument('--check',action='store_true');a=p.parse_args()
    print(json.dumps(freeze() if a.freeze else dict(unchanged=True,inputs=len(verify_prepared()['frozen_inputs']))
        if a.check else dict(connection=verify_connection()[0])))
