"""r539 실행 레시피와 r537 actor의 Windows 준비. 동결/장치/ROS 실행 없음."""
import ast, copy, hashlib, json, shutil
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
BASE=A.parent/'field_wall_guard_relevance_drive_r539_v1'
SPARSE=A.parent/'sparse_actor_recipe_r537_v1'
VEHICLE='src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml'
VEHICLE_SHA='dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021'
LIMIT=.28475436568260193
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
EXACT=['facilities_only.sdf','facility_geometry.py','goal_eval.py','course.launch.py',
 'launch/demo.launch.py','launch/local_pursuit.launch.py','launch/simulation.launch.py','.gitignore',
 'field_world.sdf','field_wall_geometry.py','world_resources.py','resource_manifest.json','resource_check.json',
 'field_geometry_audit.json','field_static_geometry.json','geometry_right_side.json','safety_capture_preparation.json',
 'traffic_runner.py','header_comparison.py','collect_evidence.py','collect_result.py','assess.py','analyze_controller_guard.py']
LOCAL=['run.py','launch_adapter.py','preflight.py','prepare.py','stop_owned.py','prepare_once.sh',
 'execution_v1/execute_once.sh','execution_v1/check_owned_processes.py','execution_v1/observe.py',
 'execution_v1/stop_once.sh','execution_v1/finish_collection.bash']

def write(path,value):
    with path.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')

def replace_once(text,before,after):
    assert text.count(before)==1,before
    return text.replace(before,after)

def main():
    assert not (A/'config.json').exists(),'preserve prepared recipe'
    assert sha(ROOT/VEHICLE)==VEHICLE_SHA
    copied=[]
    for name in EXACT+LOCAL:
        dest=A/name;dest.parent.mkdir(parents=True,exist_ok=True);assert not dest.exists()
        if name in EXACT:shutil.copyfile(BASE/name,dest)
        else:
            text=(BASE/name).read_text(encoding='utf-8')
            text=text.replace(BASE.name,'__NEXT_ARTIFACT__').replace('field_wall_resource_drive_r530_v1',BASE.name).replace('__NEXT_ARTIFACT__',A.name)
            text=text.replace('r539_owned_stop','r546_owned_stop').replace("printf 'r539 ","printf 'r546 ")
            if name=='run.py':
                text=replace_once(text,'    original_actor = target.actor_sdf',
                    '    from actor_fixture import wrap_actor_factory\n    original_actor = wrap_actor_factory(target.actor_sdf)')
                text=replace_once(text,"'field_geometry_audit.json')", "'field_geometry_audit.json','actor.sdf','config.json','marker_source.png')")
            if name=='launch_adapter.py':
                text=replace_once(text,"str(A/'course.launch.py')","str(A/'marker_course.launch.py')")
            if name=='prepare.py':
                text=replace_once(text,"    expected[expected.index('--case')+1]='right_side_start'",
                    "    expected[expected.index('--case')+1]='right_side_start'\n    expected[expected.index('--vehicle-config')+1]='"+VEHICLE+"'")
                text=replace_once(text,"    for name in ('expected_header.json','facilities_only.sdf','facility_geometry.py'):",
                    "    header=read(OLD/'expected_header.json');header['follow_config']['max_steering_rad']="+repr(LIMIT)+"\n    assert read(A/'expected_header.json')==header\n    assert cfg['acceptance']==before['acceptance'] and cfg['cases']==before['cases']\n    assert cfg['actor']==read(A/'actor_selection.json')['selected_actor']\n    for name in ('facilities_only.sdf','facility_geometry.py'):")
            dest.write_text(text,encoding='utf-8',newline='\n')
        copied.append(dict(path=name,source=(BASE/name).relative_to(ROOT).as_posix(),before_sha256=sha(BASE/name),after_sha256=sha(dest),byte_exact=name in EXACT))
    for name in ('actor_fixture.py','actor.sdf','marker_source.png'):
        shutil.copyfile(SPARSE/name,A/name)
        copied.append(dict(path=name,source=(SPARSE/name).relative_to(ROOT).as_posix(),before_sha256=sha(SPARSE/name),after_sha256=sha(A/name),byte_exact=True))
    launcher=(SPARSE/'marker_course.launch.py').read_text(encoding='utf-8')
    launcher=replace_once(launcher,"BASE=A.parent/'field_wall_resource_drive_r530_v1/course.launch.py'","BASE=A/'course.launch.py'")
    (A/'marker_course.launch.py').write_text(launcher,encoding='utf-8',newline='\n')
    cfg=read(BASE/'config.json');old_actor=copy.deepcopy(cfg['actor'])
    cfg['registered_at']='2026-11-14 Windows preparation only; root freeze/run approval pending'
    cfg['scope']='r546 one sparse marked actor; r53912sim s scene and verdicts; measured-count steering; camera marker assistance OFF first'
    cfg['launch']['entry_g1']=(A/'marker_course.launch.py').relative_to(ROOT).as_posix()
    cfg['vehicle_and_sensor_unchanged']='r539 same vehicle/sensors/physics/speed, except selected measured-count-derived steering YAML. central L=.28475436568260193; wheel angles CAD-derived, not measured.'
    sparse_actor=read(SPARSE/'config.json')['actor']
    cfg['actor']=sparse_actor
    for name in ('config.json','expected_config.json'):write(A/name,cfg)
    write(A/'actor_selection.json',dict(original_actor=old_actor,selected_actor=sparse_actor,source=(SPARSE/'config.json').relative_to(ROOT).as_posix(),source_sha256=sha(SPARSE/'config.json')))
    args=[v.replace(BASE.name,A.name) for v in read(BASE/'arguments.json')]
    args[args.index('--vehicle-config')+1]=VEHICLE
    write(A/'arguments.json',args)
    header=read(BASE/'expected_header.json');header['follow_config']['max_steering_rad']=LIMIT
    write(A/'expected_header.json',header)
    selection=read(BASE/'source_selection.json')
    for key in ('control_changes','noncontrol_changes','additional_sources','additional_tests','pending_integrations','included_integrations'):selection[key]=[]
    for key in ('reviewed_noncontrol_hashes','additional_source_hashes','additional_test_hashes','integration_evidence_sha256'):selection[key]={}
    selection.update(freeze_authorized=False,wsl_exclusive_owner=False,drive_authorized=False,final_production_source_selection_pending=True,
        expected_control_change_paths=[],pending_integrations=['root selects final common source after r544; measured steering YAML and sparse marked actor'],
        status='Windows recipe prepared; no freeze or run; marker assistance OFF only',
        root_authorization='Preparation only; root owns common source, freeze and any WSL run',
        additional_sources=[VEHICLE],additional_source_hashes={VEHICLE:VEHICLE_SHA})
    write(A/'source_selection.json',selection)
    evidence={str((BASE/n).relative_to(ROOT)).replace('\\','/'):sha(BASE/n) for n in ('prepared.json','sources.zip','config.json','run1/report.json','integrity.json','field_world.sdf')}
    evidence.update({str((SPARSE/n).relative_to(ROOT)).replace('\\','/'):sha(SPARSE/n) for n in ('actor.sdf','actor_fixture.py','audit.json','marker_source.png')})
    write(A/'recipe_provenance.json',dict(source='r539 same12sim scene/STOP/verdict; r537 sparse marked geometry reused',files=copied,original_evidence=evidence,
        geometry_generated=False,full_geometry_audit_repeated=False,acceptance_unchanged=True,one_execution_only=True,repeat_on_failure=False,actual_simulation_started=False))
    write(A/'recipe.json',dict(status='prepared_not_frozen_waiting_root',base=BASE.relative_to(ROOT).as_posix(),actor_source=SPARSE.relative_to(ROOT).as_posix(),
        case='right_side_start',seed=0,duration_sim_s=12.,actor_speed_mps=.5,actor_count=1,facilities=3,real_time_factor=.25,no_progress_stop_sim_seconds=8,
        vehicle_config=VEHICLE,vehicle_config_sha256=VEHICLE_SHA,command_limit_rad=LIMIT,
        marker=dict(id=1,dictionary='DICT_4X4_50',board_m=.05,black_code_m=.0348,same_marked_actor_for_off_on=True,assistance='OFF',
            launch_argument='r537_marker_profile',value='',on_requires_separate_root_approval=True,on_profile='config/perception/vehicle_marker_approx_b0390.json'),
        actor_sdf_sha256=sha(A/'actor.sdf'),world_sha256=sha(A/'field_world.sdf'),render_detection_runs=0,dynamic_drive_runs=0,
        original_verdict_unchanged=True,STOP_unchanged=True,actor_truth_to_controller=False))
    for path in A.rglob('*.py'):ast.parse(path.read_bytes())
    print(json.dumps(dict(copied=len(copied),actor_sha256=sha(A/'actor.sdf'),vehicle_sha256=VEHICLE_SHA,WSL_used=False,runs=0,frozen=False)))

if __name__=='__main__':main()
