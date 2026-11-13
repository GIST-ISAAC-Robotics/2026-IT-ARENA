"""Windows recipe 비교만 한다. common scope 생성/동결/빌드/ROS 실행은 하지 않는다."""
import ast
import copy
import json
import xml.etree.ElementTree as ET
from prepare import A,BASE,ROOT,sha,read,write_new,arguments


def main():
    assert not any((A/name).exists() for name in ('prepared.json','sources.zip','source_scope.json','registered.json','run1'))
    records=read(A/'baseline_copy.json')
    exact=[]
    for row in records:
        source=BASE/row['path']
        target=A/row['path']
        assert sha(source)==row['baseline_sha256'],row['path']
        if row['change']=='byte_exact_recipe_copy':
            assert source.read_bytes()==target.read_bytes()
            exact.append(row['path'])
    config=read(A/'config.json')
    expected=copy.deepcopy(read(BASE/'config.json'))
    expected['scope']=config['scope']
    expected['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
    assert config==expected
    before=read(BASE/'registered.json')['args']
    old_relative=BASE.relative_to(ROOT).as_posix()
    new_relative=A.relative_to(ROOT).as_posix()
    expected_args=[s.replace(old_relative,new_relative) for s in before]
    assert arguments()==expected_args
    assert config['cases'][0]['duration_s']==80. and config['launch']['grid_slot']==3
    models=ET.parse(A/'facilities_only.sdf').getroot().findall('model')
    assert {m.get('name') for m in models}=={'eval_r406_trial_old610_signal_support',
        'eval_r406_trial_old610_marker20_support','eval_r406_trial_old610_marker30_support'}
    assert all(m.findtext('static')=='true' for m in models)
    # 수정 run에는 이름/기준시험 기록만 바뀐다. 알고리즘은 baseline의 정확한 원문이다.
    old=(BASE/'run.py').read_text(encoding='utf8')
    transformed=old.replace(old_relative,new_relative).replace('r469','r475').replace('R469','R475').replace("baseline='r465'","baseline='r469'")
    transformed=transformed.replace("F=A.parent/'old_signal_610_r326_v1'", "F=ROOT/'artifacts/validation/2026-11-13/old_signal_610_r326_v1'")
    assert (A/'run.py').read_text(encoding='utf8')==transformed
    fixture=ROOT/'artifacts/validation/2026-11-13/old_signal_610_r326_v1'
    for path in (fixture/'fixture.json',fixture/'support_frame_m.stl'):
        assert sha(path)==read(BASE/'prepared.json')['frozen_inputs'][path.relative_to(ROOT).as_posix()]
    stop=(A/'stop_owned.py').read_text(encoding='utf8')
    assert "expected = [b'python3', b'artifacts/validation/2026-11-14/solo_full_lap_r475_v1/run.py']" in stop
    wrapper=(A/'execution_v1/execute_once.sh').read_text(encoding='utf8')
    assert 'python3 "$artifact/run.py"' in wrapper and 'python3 -B "$artifact/run.py"' not in wrapper
    assert 'r469_plus_exact_root_reviewed_delta' in wrapper
    assert 'r452_plus_r459_exact_six_files' not in wrapper
    # 보조 observer는 r469 명령에서 선택되지 않으며 공통 선언 기본값도 OFF다.
    marker_source=ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py'
    marker_tree=ast.parse(marker_source.read_bytes())
    declarations=[node for node in ast.walk(marker_tree) if isinstance(node,ast.Call)
        and isinstance(node.func,ast.Attribute) and node.func.attr=='declare_parameter'
        and len(node.args)>=2 and isinstance(node.args[0],ast.Constant)
        and node.args[0].value=='vehicle_marker_observer_enabled']
    assert len(declarations)==1 and isinstance(declarations[0].args[1],ast.Constant)
    assert declarations[0].args[1].value is False
    assert not any('vehicle_marker_observer_enabled' in value for value in arguments())
    paths=[*A.glob('*.py'),*(A/'launch').glob('*.py'),*(A/'build_v1').glob('*.py'),*(A/'execution_v1').glob('*.py')]
    for path in paths:ast.parse(path.read_bytes())
    for path in [*(A/'build_v1').glob('*.sh'),*(A/'execution_v1').glob('*.sh')]:
        assert b'\r' not in path.read_bytes()
    copied_files={r['path']:sha(A/r['path']) for r in records}
    all_recipe={p.relative_to(A).as_posix():sha(p) for p in
        [*paths,*(A/'build_v1').glob('*.sh'),*(A/'execution_v1').glob('*.sh'),A/'config.json',
         A/'facilities_only.sdf',A/'expected_header.json',A/'baseline_copy.json']}
    command=read(BASE/'launch_command.json')
    command=[s.replace(old_relative,new_relative) for s in command]
    write_new(A/'launch_command_expected.json',command)
    all_recipe['launch_command_expected.json']=sha(A/'launch_command_expected.json')
    result=dict(recipe_verified=True,byte_exact_baseline_copies=exact,copied_file_sha256=copied_files,
        recipe_files=all_recipe,baseline_archive_sha256=sha(BASE/'sources.zip'),
        baseline_prepared_sha256=sha(BASE/'prepared.json'),args=arguments(),
        actor_count=0,facilities=3,config_changes=['scope','launch.entry_g1'],
        unchanged=['runner','evaluator','finalize','sensor','vehicle','speed','physics','runtime_world_overrides',
                   'controller_selection','80sim_seconds','8sim_seconds_no_progress_stop','shutdown','trace_option'],
        python_ast_files=len(paths),raw_baseline_results_copied=False,old_execution_failures_copied=False,
        common_source_changes_by_this_task=False,source_scope_created=False,source_frozen=False,
        build_executed=False,ros_initialized=False,gazebo_started=False,
        vehicle_marker_observer_enabled=False,
        pending=['r474 final checks and exact actual common delta review','build/import/fixed CLI','freeze',
                 'delegated final execution review under root one-run authorization'])
    write_new(A/'recipe_manifest.json',result)
    print(json.dumps(dict(recipe_verified=True,exact_copies=len(exact),python_ast_files=len(paths),
        recipe_files=len(all_recipe),source_frozen=False,build_executed=False,simulation_started=False)))


if __name__=='__main__':main()
