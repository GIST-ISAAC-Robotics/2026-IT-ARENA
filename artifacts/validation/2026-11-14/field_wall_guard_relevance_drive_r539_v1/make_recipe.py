"""r530 실제 실패의 동일조건 비교 레시피. Windows 준비만, 새 실행 없음."""
import ast,hashlib,json,shutil
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3];OLD=A.parent/'field_wall_resource_drive_r530_v1'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
EXACT=['expected_header.json','facilities_only.sdf','facility_geometry.py','goal_eval.py','course.launch.py','launch_adapter.py',
 'launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py','.gitignore',
 'field_world.sdf','field_wall_geometry.py','world_resources.py','resource_manifest.json','resource_check.json',
 'field_geometry_audit.json','field_static_geometry.json','geometry_right_side.json','safety_capture_preparation.json',
 'traffic_runner.py','header_comparison.py','collect_evidence.py','collect_result.py','assess.py','analyze_controller_guard.py']
LOCAL=['run.py','preflight.py','prepare.py','stop_owned.py','prepare_once.sh','execution_v1/execute_once.sh',
 'execution_v1/check_owned_processes.py','execution_v1/observe.py','execution_v1/stop_once.sh','execution_v1/finish_collection.bash']


def write(p,value):
    with p.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')


def main():
    assert not (A/'config.json').exists()
    assert read(OLD/'run1/report.json')['passed'] is False and read(OLD/'integrity.json')['changed_inputs']==[]
    assert not read(OLD/'execution_v1/owned_processes_after.json')['remaining_owned_work']
    baseline=read(OLD/'prepared.json');assert sha(OLD/'sources.zip')==baseline['source_archive_sha256']
    copied=[]
    for name in EXACT+LOCAL:
        destination=A/name;destination.parent.mkdir(parents=True,exist_ok=True);assert not destination.exists()
        if name in EXACT:shutil.copyfile(OLD/name,destination)
        else:
            content=(OLD/name).read_text(encoding='utf-8')
            if name=='prepare.py':content=content.replace('field_wall_drive_r520_v1','__R530_BASELINE__')
            content=content.replace(OLD.name,A.name).replace('__R530_BASELINE__',OLD.name)
            content=content.replace('r530_owned_stop','r539_owned_stop').replace("print('r530 actual","print('r539 actual").replace("printf 'r530 ","printf 'r539 ")
            destination.write_text(content,encoding='utf-8',newline='\n')
        copied.append(dict(path=name,source=(OLD/name).relative_to(ROOT).as_posix(),before_sha256=sha(OLD/name),after_sha256=sha(destination),byte_exact=name in EXACT))
    cfg=read(OLD/'config.json');cfg['scope']='r539 same r53012sim s scene; guard-memory braking-envelope relevance candidate pending final root SHA and tests'
    cfg['registered_at']='2026-11-14 Windows preparation only; not frozen or execution-authorized'
    cfg['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    for name in ('config.json','expected_config.json'):write(A/name,cfg)
    write(A/'arguments.json',[v.replace(OLD.name,A.name) for v in read(OLD/'arguments.json')])
    source=read(OLD/'source_selection.json')
    for key in ('control_changes','noncontrol_changes','additional_sources','additional_tests','pending_integrations'):source[key]=[]
    for key in ('reviewed_noncontrol_hashes','additional_source_hashes','additional_test_hashes','integration_evidence_sha256'):source[key]={}
    source.update(freeze_authorized=False,wsl_exclusive_owner=False,drive_authorized=False,final_production_source_selection_pending=True,
        expected_control_change_paths=['src/arena_autonomy/arena_autonomy/guard_memory_command.py'],
        pending_integrations=['r538 braking-envelope relevance guard; root final common SHA and related tests pending'],
        status='Same r530 scene and recorder prepared; only guard_memory_command.py plus approved tests expected',
        root_authorization='Windows preparation only; WSL/common source root-owned; no freeze/run authorization')
    write(A/'source_selection.json',source)
    original=dict(read(OLD/'recipe_provenance.json')['original_evidence'])
    for n in ('prepared.json','sources.zip','run1/report.json','integrity.json','evidence_collection.json','controller_guard_supplement.json','field_world.sdf','RESULT.md'):
        original[(OLD/n).relative_to(ROOT).as_posix()]=sha(OLD/n)
    write(A/'recipe_provenance.json',dict(source='r530 actual12sim s run retained as failed comparison baseline',files=copied,original_evidence=original,
        same_geometry=True,geometry_recalculated=False,static_actor601_reused=True,threshold_changes=False,case_actor_acceptance_unchanged=True,
        one_execution_only=True,repeat_on_failure=False,common_source_selection_pending=True,actual_simulation_started=False))
    write(A/'same_scene_receipt.json',dict(world_sha256=sha(A/'field_world.sdf'),world_byte_identical=sha(A/'field_world.sdf')==sha(OLD/'field_world.sdf'),
        facilities_byte_identical=sha(A/'facilities_only.sdf')==sha(OLD/'facilities_only.sdf'),
        traffic_runner_byte_identical=sha(A/'traffic_runner.py')==sha(OLD/'traffic_runner.py'),
        raw_status_recorder_unchanged=True,field_collision_count=475,resource_references_reused=10,
        geometry_recalculated=False,static_actor601_reused=True,actor_count=1,facilities=3,max_sim_seconds=12,real_time_factor=.25,
        r530_failed_report_sha256=sha(OLD/'run1/report.json'),r530_failed_original_passed=False,
        r530_raw_hashes_reused=read(OLD/'evidence_collection.json')['evidence_files'],
        WSL_used=False,frozen=False,actual_run_count=0))
    for p in A.rglob('*.py'):ast.parse(p.read_bytes())
    print(json.dumps(dict(copied=len(copied),same_world=True,world_sha256=sha(A/'field_world.sdf'),same_recorder=True,WSL_used=False,run_count=0)))


if __name__=='__main__':main()
