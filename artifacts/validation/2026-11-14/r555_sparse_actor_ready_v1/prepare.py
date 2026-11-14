"""r539 장면과 r546 sparse actor를 유지하고 root가 선택한 r553 소스를 동결한다."""
import argparse,ast,json,zipfile
from pathlib import Path
import hashlib
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'field_wall_guard_relevance_drive_r539_v1'
def read(path):return json.loads(path.read_bytes())
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def write_new(path,value):
    with path.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')

def verify_recipe():
    source=read(A/'source_selection.json');assert source['freeze_authorized'] and source['wsl_exclusive_owner']
    assert source['drive_authorized'] and not source['final_production_source_selection_pending']
    old=read(OLD/'prepared.json');assert sha(OLD/'sources.zip')==old['source_archive_sha256']
    cfg=read(A/'config.json');before=read(OLD/'config.json')
    assert cfg==read(A/'expected_config.json')
    expected=[x.replace('field_wall_guard_relevance_drive_r539_v1','r555_sparse_actor_ready_v1') for x in old['args']]
    expected[expected.index('--case')+1]='right_side_start'
    expected[expected.index('--vehicle-config')+1]='src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml'
    assert read(A/'arguments.json')==expected
    header=read(OLD/'expected_header.json');header['follow_config']['max_steering_rad']=0.28475436568260193
    assert read(A/'expected_header.json')==header
    assert cfg['acceptance']==before['acceptance'] and cfg['cases']==before['cases']
    assert cfg['actor']==read(A/'actor_selection.json')['selected_actor']
    for name in ('facilities_only.sdf','facility_geometry.py'):
        assert sha(A/name)==sha(OLD/name),name
    # 장면은 r539, 제어 소스는 root가 확정한 r553 동결본에서 재사용한다.
    source_base=ROOT/source['source_baseline']
    baseline=read(source_base/'prepared.json')
    assert sha(source_base/'sources.zip')==baseline['source_archive_sha256']
    common={p:h for p,h in baseline['frozen_inputs'].items() if p.startswith(('src/','scripts/','tests/','config/')) or 'old_signal_610_r326_v1/' in p}
    allowed={x['path']:x for x in source['control_changes']}
    selected={};delta=[]
    for p,h in common.items():
        now=sha(ROOT/p)
        if p in allowed:assert h==allowed[p]['before_sha256'] and now==allowed[p]['after_sha256'],p
        elif now!=h:assert p in source['noncontrol_changes'],p
        if now!=h:delta.append(dict(path=p,before_sha256=h,after_sha256=now,role='control_python' if p in allowed else 'noncontrol_not_launched_in_sim'))
        selected[p]=now
    assert sum(x['role']=='control_python' for x in delta)==len(allowed)
    for p in [*source['additional_tests'],*source.get('additional_sources',[])]:
        assert p not in selected,p
        selected[p]=sha(ROOT/p)
    for p,h in {**source.get('reviewed_noncontrol_hashes',{}),**source.get('additional_source_hashes',{}),**source.get('additional_test_hashes',{})}.items():
        assert selected[p]==h,p
    from field_wall_geometry import verify_derived
    assert verify_derived()['static_geometry_passed']
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    for p in A.rglob('*.sh'):assert b'\r' not in p.read_bytes(),p
    native='src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'
    assert selected[native]==common[native]
    return dict(args=expected,source_delta=delta,additional_tests=source['additional_tests'],additional_sources=source.get('additional_sources',[]),native_sha256=selected[native]),selected

def freeze():
    assert not any((A/p).exists() for p in ('prepared.json','sources.zip','registered.json','run1'))
    info,selected=verify_recipe()
    source_base=ROOT/read(A/'source_selection.json')['source_baseline']
    assert read(A/'preflight/result.json')['passed']
    assert read(A/'geometry_right_side.json')['passed']
    for name,digest in read(A/'source_selection.json').get('integration_evidence_sha256',{}).items():
        assert sha(ROOT/name)==digest,name
        selected[name]=digest
    for name, digest in read(A/'recipe_provenance.json')['original_evidence'].items():
        assert sha(ROOT/name)==digest,name
        selected[name]=digest
    for p in [*A.glob('*.py'),*A.glob('*.json'),*A.glob('*.sdf'),*A.glob('*.png'),*A.glob('*.md'),*A.glob('*.sh'),
              *(A/'launch').glob('*.py'),*(A/'execution_v1').glob('*.sh'),*(A/'execution_v1').glob('*.bash'),
              *(A/'execution_v1').glob('*.py'),A/'preflight/result.json',OLD/'prepared.json',OLD/'sources.zip',source_base/'prepared.json',source_base/'sources.zip']:
        if p.is_file():selected[p.relative_to(ROOT).as_posix()]=sha(p)
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in selected:
            if not p.endswith('.zip'):z.write(ROOT/p,p)
    result=dict(**info,frozen_inputs=selected,source_archive_sha256=sha(A/'sources.zip'),
        actor_count=1,facilities=3,real_time_factor=.25,max_sim_seconds=12,no_progress_stop_sim_seconds=8,
        one_execution_only=True,repeat_on_failure=False,root_run_confirmation_required=True,
        included_integrations=read(A/'source_selection.json')['included_integrations'],build_reused=True)
    write_new(A/'prepared.json',result)
    return dict(inputs=len(selected),prepared_sha256=sha(A/'prepared.json'),source_archive_sha256=result['source_archive_sha256'],source_delta=info['source_delta'])

def verify_prepared():
    v=read(A/'prepared.json');changed=[p for p,h in v['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    assert not changed,changed
    assert sha(A/'sources.zip')==v['source_archive_sha256']
    return v

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--check',action='store_true');p.add_argument('--freeze',action='store_true');a=p.parse_args()
    print(json.dumps(freeze() if a.freeze else dict(checked=True,inputs=len(verify_prepared()['frozen_inputs'])) if a.check else verify_recipe()[0]))
