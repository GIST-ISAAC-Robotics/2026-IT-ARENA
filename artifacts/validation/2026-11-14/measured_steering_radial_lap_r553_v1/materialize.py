"""GO 후 r550 실행 레시피와 승인된 r552 입력을 별도 단위에 연결한다."""
import ast,hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'measured_steering_reacquire_lap_r550_v1';R552=A.parent/'radial_motion_reacquire_r552_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())
def write(p,v):
    with p.open('x',encoding='utf-8') as f:json.dump(v,f,ensure_ascii=False,indent=2);f.write('\n')
def main():
    assert not any((A/p).exists() for p in ('config.json','prepared.json','sources.zip','registered.json','run1'))
    old=read(BASE/'prepared.json');report=read(BASE/'run1/report.json');prefix=BASE.relative_to(ROOT).as_posix()+'/'
    assert sha(BASE/'sources.zip')==old['source_archive_sha256'] and report['shutdown_clean']
    assert not read(BASE/'execution_v1/owned_processes_after.json')['remaining_owned_work']
    names=['audit_runtime.py','collect_evidence.py','collect_result.py','course.launch.py','facility_geometry.py','field_wall_geometry.py',
      'header_comparison.py','launch_adapter.py','run.py','solo_evaluation.py','solo_finalize.py','solo_runner.py','steering_runtime_audit.py','stop_owned.py','world_resources.py',
      'config.json','expected_config.json','expected_header.json','field_geometry_audit.json','field_static_geometry.json','geometry_right_side.json','resource_check.json','resource_manifest.json',
      'facilities_only.sdf','field_world.sdf','launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py',
      'execution_v1/check_owned_processes.py','execution_v1/observe.py','execution_v1/execute_once.sh','execution_v1/stop_once.sh','execution_v1/finish_collection.bash',
      'prepare.py','preflight.py','prepare_once.sh','build_once.sh','build_receipt.py']
    copied=[]
    for name in names:
        source=BASE/name;dest=A/name;assert sha(source)==old['frozen_inputs'][prefix+name];assert not dest.exists();dest.parent.mkdir(parents=True,exist_ok=True)
        data=source.read_bytes()
        if dest.suffix in ('.py','.json','.sh','.bash'):
            text=data.decode('utf-8').replace(BASE.name,A.name)
            if name=='prepare.py':text=text.replace("BASE=A.parent/'measured_steering_lap_r544_attempt2_v1'", "BASE=A.parent/'"+BASE.name+"'")
            if name=='preflight.py':
                text=text.replace("'guard_memory_command'):","'guard_memory_command','current_path_recheck._geometry','race_runtime_policy.retained_recheck._geometry'):")
                text=text.replace("+name+'.py'","+name.replace('.','/')+'.py'")
            if name.startswith('execution_v1/') and dest.suffix in ('.sh','.py'):text=text.replace('r544','r553')
            data=text.encode('utf-8')
        dest.write_bytes(data);copied.append(dict(path=name,baseline_sha256=sha(source),sha256=sha(dest),path_relocated=True,source_connection_change=name in ('prepare.py','preflight.py')))
    args=read(A/'arguments.json');assert args==[v.replace(BASE.name,A.name) for v in read(BASE/'arguments.json')]
    manifest=read(R552/'integrated_manifest.json')
    for p,h in manifest.items():assert sha(ROOT/p)==h,p
    panel=read(A.parent/'panel_preview_idle_r551_v1/common_transfer/manifest.json');observed_extra={p:v['after_sha256'] for p,v in panel.items() if p.endswith(('race_ops_node.py','race_ops_page.py'))}
    baseline={p:h for p,h in {**old['frozen_inputs'],**report['source_sha256']}.items() if not p.startswith(prefix)}
    selected={};delta=[];observed_delta=[]
    for p,before in baseline.items():
        after=sha(ROOT/p)
        if p in observed_extra:
            assert after==observed_extra[p];observed_delta.append(dict(path=p,before_sha256=before,after_sha256=after,classification='unused panel module; source collector may include broad package inventory'));continue
        if after!=before:assert manifest.get(p)==after,p;delta.append(dict(path=p,before_sha256=before,after_sha256=after))
        selected[p]=after
    assert len(delta)==5,delta
    tests={}
    for name,count in [('native_regression.xml',195),('recheck_verification/junit.xml',12)]:
        path=R552/name;rows=ET.parse(path).getroot().findall('.//testcase')
        assert len(rows)==count and all(x.find('failure') is None and x.find('error') is None and x.find('skipped') is None for x in rows)
        tests[path.relative_to(ROOT).as_posix()]=dict(sha256=sha(path),passed=count)
    selection=dict(freeze_authorized=True,wsl_exclusive_owner=True,drive_authorized=True,final_production_source_selection_pending=False,
      source_baseline=BASE.relative_to(ROOT).as_posix(),required_sources=selected,source_delta=delta,observational_source_delta=observed_delta,
      status_detail='full',native_rebuild_authorized=True,validation=tests,integrated_manifest_sha256=sha(R552/'integrated_manifest.json'),
      status='root GO: revision3 ray alignment plus r552 radial low-speed helper/current/retained; one drive only, no repeat')
    write(A/'source_selection.json',selection)
    observed={**old['observed_inputs'],**observed_extra}
    for p,h in manifest.items():
        if p not in selected:observed[p]=h
    write(A/'observational_baseline.json',observed)
    preserved={name:sha(BASE/name) for name in ('prepared.json','sources.zip','run1/report.json','integrity.json','execution_v1/owned_processes_after.json')}
    write(A/'recipe_provenance.json',dict(baseline=BASE.relative_to(ROOT).as_posix(),preserved_evidence=preserved,files=copied,
      world_sha256=sha(A/'field_world.sdf'),expected_header_sha256=sha(A/'expected_header.json'),seven_angle_audit_reused=True,original_acceptance_speed_stop_unchanged=True))
    cfg=read(A/'config.json');assert json.loads(json.dumps(cfg).replace(A.name,BASE.name))==read(BASE/'config.json')
    assert sha(A/'expected_header.json')==sha(BASE/'expected_header.json') and sha(A/'field_world.sdf')==sha(BASE/'field_world.sdf')
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    write(A/'materialization_receipt.json',dict(passed=True,source_selection_sha256=sha(A/'source_selection.json'),source_delta=delta,
      observed_panel_changes=observed_delta,integrated_manifest=manifest,validation=tests,command_sha256=sha(A/'arguments.json'),
      measured_yaml_sha256=manifest['src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml'],
      hardware_profile_sha256=manifest['config/hardware/steering_zero.assembled_2026_11_14.json'],hardware_profile_runtime_selected=False,
      expected_header_sha256=sha(A/'expected_header.json'),world_sha256=sha(A/'field_world.sdf'),native_expected_sha256=old['native_sha256'],
      actual_runs=0,one_execution_only=True,repeat_on_failure=False))
    print(json.dumps(dict(materialized=True,required_external_inputs=len(selected),source_delta=delta,actual_runs=0)))
if __name__=='__main__':main()
