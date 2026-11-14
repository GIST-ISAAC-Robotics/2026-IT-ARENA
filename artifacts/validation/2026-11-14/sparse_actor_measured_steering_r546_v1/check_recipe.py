"""Windows 입력/연결 정적 검사. actor 생성·기하 전수 감사·동결·ROS 실행 없음."""
import ast,hashlib,importlib.util,json
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
BASE=A.parent/'field_wall_guard_relevance_drive_r539_v1';SPARSE=A.parent/'sparse_actor_recipe_r537_v1'
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    cfg=read(A/'config.json');old=read(BASE/'config.json');recipe=read(A/'recipe.json')
    assert cfg==read(A/'expected_config.json')
    for key in ('acceptance','cases','seeds','schedule_anchor','ground_truth_allowlist_evaluation_only'):
        assert cfg[key]==old[key],key
    launch=dict(cfg['launch']);launch['entry_g1']=old['launch']['entry_g1'];assert launch==old['launch']
    assert cfg['actor']==read(SPARSE/'config.json')['actor']==read(A/'actor_selection.json')['selected_actor']
    actor=cfg['actor'];assert actor['visual_outline']=='sparse_upper_posts' and actor['bottom_m']==.02
    assert (actor['length_m'],actor['width_m'],actor['height_m'])==(.2,.15,.22)
    case=cfg['cases'];assert len(case)==1 and case[0]['name']=='right_side_start' and case[0]['duration_s']==12.
    assert case[0]['schedule']==[dict(from_s=0.,to_s=12.,speed_mps=.5)]
    assert (case[0]['actor_start_center_gap_m'],case[0]['actor_lateral_offset_m'])==(.5,-.23)
    for name in ('actor.sdf','actor_fixture.py','marker_source.png'):assert sha(A/name)==sha(SPARSE/name),name
    for name in ('field_world.sdf','facilities_only.sdf','traffic_runner.py','collect_evidence.py'):
        assert sha(A/name)==sha(BASE/name),name
    args=[x.replace(BASE.name,A.name) for x in read(BASE/'arguments.json')]
    args[args.index('--vehicle-config')+1]=recipe['vehicle_config'];assert args==read(A/'arguments.json')
    assert sha(ROOT/recipe['vehicle_config'])==recipe['vehicle_config_sha256']=='dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021'
    header=read(BASE/'expected_header.json');header['follow_config']['max_steering_rad']=recipe['command_limit_rad'];assert header==read(A/'expected_header.json')
    launcher=(A/'marker_course.launch.py').read_text(encoding='utf8')
    assert "BASE=A/'course.launch.py'" in launcher
    assert "DeclareLaunchArgument('r537_marker_profile',default_value='')" in launcher
    assert "SetParameter(name='vehicle_marker_observer_enabled',value=False)" in launcher
    assert "str(A/'marker_course.launch.py')" in (A/'launch_adapter.py').read_text(encoding='utf8')
    run=(A/'run.py').read_text(encoding='utf8')
    assert 'original_actor = wrap_actor_factory(target.actor_sdf)' in run
    assert 'compose_facilities(original_actor(config, pose))' in run
    assert recipe['marker']['assistance']=='OFF' and recipe['marker']['value']==''
    assert recipe['marker']['on_requires_separate_root_approval']
    source=read(A/'source_selection.json')
    assert not any(source[k] for k in ('freeze_authorized','wsl_exclusive_owner','drive_authorized'))
    assert source['final_production_source_selection_pending']
    for name in ('prepared.json','sources.zip','registered.json','run1','preflight','root_execution_review.json'):
        assert not (A/name).exists(),name
    for row in read(A/'recipe_provenance.json')['files']:
        assert sha(ROOT/row['source'])==row['before_sha256'] and sha(A/row['path'])==row['after_sha256'],row['path']
    for path in A.rglob('*.py'):ast.parse(path.read_bytes())
    for path in [*A.rglob('*.sh'),*A.rglob('*.bash')]:assert b'\r' not in path.read_bytes(),path
    spec=importlib.util.spec_from_file_location('r546_prepare_check',A/'prepare.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    refused=False
    try:module.verify_recipe()
    except AssertionError:refused=True
    assert refused
    result=dict(passed=True,world_byte_identical=True,actor_byte_identical=True,marker_texture_byte_identical=True,
        evaluator_and_evidence_collector_byte_identical=True,case_schedule_acceptance_unchanged=True,
        marker_assistance='OFF',on_approval_pending=True,vehicle_sha256=recipe['vehicle_config_sha256'],
        command_limit_rad=recipe['command_limit_rad'],actor_sha256=sha(A/'actor.sdf'),
        header_expected_limit_updated=True,unauthorized_freeze_rejected=True,
        geometry_generated=False,full_geometry_audit_repeated=False,WSL_used=False,Jetson_used=False,
        frozen=False,render_detection_runs=0,dynamic_drive_runs=0)
    (A/'preparation_receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(json.dumps(result))
if __name__=='__main__':main()
