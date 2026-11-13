"""평가 기준·선택 명령·원 시설 함수 보존. 주행/노드 실행 없음."""
import ast
import copy
import json
from pathlib import Path

import pytest
from prepare import A, ROOT, BASE, SIM, arguments, read, static_contract, verify_prepared
from launch_adapter import rewrite_command, SELECTION
from audit_runtime import summarize, KEY


@pytest.fixture(scope='module', autouse=True)
def frozen():
    before = verify_prepared()
    yield
    assert verify_prepared() == before


def function(path, name):
    source = path.read_text(encoding='utf8')
    node = next(n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name == name)
    return ast.get_source_segment(source, node)


def test_config_thresholds_actor_and_argument_changes_are_bounded():
    assert static_contract()['thresholds_changed'] is False
    old = list(read(BASE/'registered.json')['args'])
    new = arguments()
    assert '--pilot-marker-z' not in new
    for flag in ('--config', '--actor-overlay', '--output', '--local-sensor-config', '--vehicle-config'):
        old[old.index(flag)+1] = new[new.index(flag)+1]
    i = old.index('--pilot-marker-z'); del old[i:i+2]
    assert old == new
    import yaml
    vehicle = yaml.safe_load((ROOT/new[new.index('--vehicle-config')+1]).read_bytes())['vehicle']
    assert vehicle['identification_marker']['xyz_m'][2] == .09
    expected = read(A/'expected_header.json')
    mask = json.loads(expected['follow_config']['known_structure_sectors_json'])
    assert len(mask['sectors_deg']) == len(mask['range_limits_m']) == 4
    old_header = json.loads((BASE/'run1/follow_decisions.jsonl').open(encoding='utf8').readline())['data']
    assert expected['execution_config'] == old_header['execution_config']
    assert expected['current_corner_budget'] == 'r396_v1' and expected['motion_config']['lidar_frame_id'] == 'laser_frame'


def test_command_uses_common_executable_without_node_prefix():
    command = ['ros2', 'launch', '--noninteractive', 'arena_bringup', 'g1_follow.launch.py',
               'g1_decision_execution:=process_candidate', 'headless:=true']
    result = rewrite_command(command)
    assert result[:4] == ['ros2', 'launch', '--noninteractive', str(A/'course.launch.py')]
    assert result[-len(SELECTION):] == SELECTION
    assert '--launch-prefix' not in result and not any('node_entry' in item for item in result)


@pytest.mark.parametrize('extra', ['pilot_marker_z:=0.140', 'g1_runtime_policy:=off'])
def test_conflicting_launch_selection_is_rejected(extra):
    with pytest.raises(AssertionError):
        rewrite_command(['ros2', 'launch', '--noninteractive', 'arena_bringup', 'g1_follow.launch.py', extra])


def test_launch_patch_changes_only_two_default_off_selections():
    before = (A/'common_before'/SIM).read_text(encoding='utf8')
    stage = (A/'staged'/SIM).read_text(encoding='utf8')
    tree = ast.parse(stage)
    keys = {'g1_runtime_policy': ['off', 'cad_runtime_r377_v1'], 'g1_current_corner_budget': ['off', 'r396_v1']}
    removed = set()
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id == 'DeclareLaunchArgument' and node.args
                and isinstance(node.args[0], ast.Constant) and node.args[0].value in keys):
            name = node.args[0].value; values = {k.arg: ast.literal_eval(k.value) for k in node.keywords}
            assert values['default_value'] == 'off' and values['choices'] == keys[name]
            removed.update(range(node.lineno, node.end_lineno+1))
    for index, line in enumerate(stage.splitlines(), 1):
        if any('"'+key+'": LaunchConfiguration("'+key+'").perform(context)' in line for key in keys):
            removed.add(index)
    assert len(removed) == 8
    assert '\n'.join(line for i,line in enumerate(stage.splitlines(),1) if i not in removed) == before.rstrip('\n')


def test_facility_geometry_function_bodies_preserved():
    local = A/'facility_geometry.py'
    sources = [('artifacts/validation/2026-11-09/signal_post_source_v1/analyze_stl.py', ('load','slice_xy')),
               ('artifacts/validation/2026-11-12/signal_post_revision_r291_v1/prepare_fixture.py', ('section',)),
               ('artifacts/validation/2026-11-13/old610_common_corner_r333_v1/run.py', ('slab_projection',))]
    for path, names in sources:
        for name in names:
            expected = function(ROOT/path, name).replace('analysis.audit.slice_xy(', 'slice_xy(').replace('fixture.section(', 'section(')
            assert ast.dump(ast.parse(function(local, name))) == ast.dump(ast.parse(expected))


def test_candidate_acceptance_is_not_reported_as_publication_without_ack():
    expected = {'current_corner_budget': 'r396_v1'}
    current = dict(reference_s=2., steering_rad=.2, speed_mps=.08,
                   **{KEY: dict(applied=True, reason='single_current_distance_budget_accepted')})
    rows = [dict(kind='call', data=dict(current_rebase={'output': current}, delivery={'kind': 'candidate'}))]
    assert summarize(rows, expected)['candidate_calculations_accepted'] == 1
    assert summarize(rows, expected)['candidate_parent_publications'] == 0
    publication = dict(kind='route_publication', data=dict(retained=True, reference_s=2., steering_rad=.2, speed_mps=.08))
    assert summarize(rows+[publication], expected)['candidate_parent_publications'] == 1
    assert summarize(rows+[dict(kind='reset',data={}), publication], expected)['candidate_parent_publications'] == 0
    wrong = copy.deepcopy(publication); wrong['data']['reference_s'] = 3.
    assert summarize(rows+[wrong], expected)['candidate_parent_publications'] == 0
