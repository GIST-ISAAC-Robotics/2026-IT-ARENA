"""r550 실행 필수 입력과 관측 목록을 분리한다. 최종 승인 전 동결을 거절한다."""
import argparse,ast,hashlib,json,zipfile
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'measured_steering_reacquire_lap_r550_v1'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
def write_new(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')
def arguments():return read(A/'arguments.json')
def verify_traffic_review():
    # 기존 run.py의 호출 이름만 유지한다. r544 원 실패를 성공으로 재해석하지 않는다.
    provenance=read(A/'recipe_provenance.json')
    for name,h in provenance['preserved_evidence'].items():assert sha(BASE/name)==h,name
    assert read(BASE/'run1/report.json')['shutdown_clean']
    assert not read(BASE/'execution_v1/owned_processes_after.json')['remaining_owned_work']
    return dict(previous_run_passed=False,previous_run_recovered=True)
def verify_recipe():
    selection=read(A/'source_selection.json')
    assert all(selection[k] for k in ('freeze_authorized','wsl_exclusive_owner','drive_authorized'))
    assert not selection['final_production_source_selection_pending']
    verify_traffic_review()
    cfg=read(A/'config.json');assert cfg==read(A/'expected_config.json')
    assert json.loads(json.dumps(cfg).replace(A.name,BASE.name))==read(BASE/'config.json')
    assert arguments()==[v.replace(BASE.name,A.name) for v in read(BASE/'arguments.json')]
    assert sha(A/'expected_header.json')==sha(BASE/'expected_header.json')
    for item in read(A/'recipe_provenance.json')['files']:assert sha(A/item['path'])==item['sha256'],item['path']
    selected=selection['required_sources']
    for p,h in selected.items():assert sha(ROOT/p)==h,p
    native='src/arena_autonomy/arena_autonomy/_numeric_kernels.cpython-312-x86_64-linux-gnu.so'
    return dict(args=arguments(),source_delta=selection['source_delta'],native_sha256=selected[native]),dict(selected)
def freeze():
    assert not any((A/p).exists() for p in ('prepared.json','sources.zip','registered.json','run1'))
    info,required=verify_recipe();assert read(A/'preflight/result.json')['passed']
    for p in [*A.glob('*.py'),*A.glob('*.json'),*A.glob('*.sdf'),*A.glob('*.md'),*A.glob('*.sh'),
              *(A/'launch').glob('*.py'),*(A/'execution_v1').glob('*.py'),*(A/'execution_v1').glob('*.sh'),*(A/'execution_v1').glob('*.bash'),A/'preflight/result.json']:
        if p.suffix=='.py':ast.parse(p.read_bytes())
        required[p.relative_to(ROOT).as_posix()]=sha(p)
    observed={p:sha(ROOT/p) for p in read(A/'observational_baseline.json') if p not in required and (ROOT/p).is_file()}
    for p in ('tests/test_guard_status_payload.py','tests/test_guard_status_payload_ros.py','tests/test_current_wall_reacquire.py'):
        if p not in required and (ROOT/p).is_file():observed[p]=sha(ROOT/p)
    with zipfile.ZipFile(A/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in {**required,**observed}:
            if not p.endswith('.zip'):z.write(ROOT/p,p)
    value=dict(**info,frozen_inputs=required,observed_inputs=observed,source_archive_sha256=sha(A/'sources.zip'),actor_count=0,facilities=3,
        max_sim_seconds=80,real_time_factor=.25,no_progress_stop_sim_seconds=8,package_build_executed=read(A/'build_result.json')['passed'],
        native_binary_changed=read(A/'build_before.json')['native_sha256']!=read(A/'build_result.json')['native_sha256'],
        one_execution_only=True,repeat_on_failure=False,status_detail='full')
    write_new(A/'prepared.json',value)
    return dict(required_inputs=len(required),observational_inputs=len(observed),prepared_sha256=sha(A/'prepared.json'),archive_sha256=value['source_archive_sha256'])
def verify_prepared():
    value=read(A/'prepared.json');changed=[p for p,h in value['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    assert not changed,changed
    assert sha(A/'sources.zip')==value['source_archive_sha256']
    value['observational_changes_at_check']=[p for p,h in value['observed_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
    return value
if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--freeze',action='store_true');parser.add_argument('--check',action='store_true');options=parser.parse_args()
    print(json.dumps(freeze() if options.freeze else dict(checked=True,required_inputs=len(verify_prepared()['frozen_inputs'])) if options.check else verify_recipe()[0]))
