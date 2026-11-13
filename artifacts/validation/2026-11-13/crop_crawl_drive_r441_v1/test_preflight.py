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
    for flag in ('--config', '--actor-overlay', '--output'):
        old[old.index(flag)+1] = new[new.index(flag)+1]
    assert old == new
    import yaml
    vehicle = yaml.safe_load((ROOT/new[new.index('--vehicle-config')+1]).read_bytes())['vehicle']
    assert vehicle['identification_marker']['xyz_m'][2] == .09
    expected = read(A/'expected_header.json')
    mask = json.loads(expected['follow_config']['known_structure_sectors_json'])
    assert len(mask['sectors_deg']) == len(mask['range_limits_m']) == 4
    old_header = read(BASE/'expected_header.json')
    assert expected['execution_config'] == old_header['execution_config']
    assert expected['current_corner_budget'] == 'r396_v1' and expected['motion_config']['lidar_frame_id'] == 'laser_frame'
    assert expected['follow_config']['crop_edge_component_trial'] is True
    old_selection = ast.literal_eval(next(n.value for n in ast.parse((BASE/'launch_adapter.py').read_text(encoding='utf8')).body
        if isinstance(n, ast.Assign) and isinstance(n.targets[0], ast.Name) and n.targets[0].id == 'SELECTION'))
    assert SELECTION == old_selection


def test_command_uses_common_executable_without_node_prefix():
    command = ['ros2', 'launch', '--noninteractive', 'arena_bringup', 'g1_follow.launch.py',
               'g1_decision_execution:=process_candidate', 'headless:=true']
    result = rewrite_command(command)
    assert result[:4] == ['ros2', 'launch', '--noninteractive', str(A/'course.launch.py')]
    assert result[-len(SELECTION):] == SELECTION
    assert '--launch-prefix' not in result and not any('node_entry' in item for item in result)


@pytest.mark.parametrize('extra', ['pilot_marker_z:=0.140', 'g1_runtime_policy:=off', 'g1_crop_edge_component_trial:=false'])
def test_conflicting_launch_selection_is_rejected(extra):
    with pytest.raises(AssertionError):
        rewrite_command(['ros2', 'launch', '--noninteractive', 'arena_bringup', 'g1_follow.launch.py', extra])


def test_r409_header_comparison_source_is_unchanged():
    original = A.parent/'runtime_config_audit_r409_v1/audit_saved.py'
    assert (A/'header_comparison.py').read_bytes() == original.read_bytes()


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


def test_only_structure_json_spacing_can_be_equivalent():
    from header_comparison import compare_headers
    expected = read(A/'expected_header.json')
    actual = copy.deepcopy(expected)
    field = 'known_structure_sectors_json'
    actual['follow_config'][field] = json.dumps(json.loads(actual['follow_config'][field]), indent=2)
    result = compare_headers(actual, expected)
    assert result['equivalent'] and not result['exact']
    actual['follow_config']['crop_edge_component_trial'] = False
    assert not compare_headers(actual, expected)['equivalent']
    actual = copy.deepcopy(expected)
    actual['follow_config'][field] = '{"sectors_deg":[],"sectors_deg":[],"range_limits_m":[]}'
    assert not compare_headers(actual, expected)['equivalent']


def test_component_diagnostic_presence_is_separate_from_positive_output():
    from audit_runtime import component_summary
    detail = {'normal': [0., .1, {'partial_observation': {'allowed': True,
        'crop_edge_component': {'scope': 'bounded_simulation_edge_component_not_free_space'}}}]}
    row = dict(kind='call', data=dict(output=[0., .1, detail]))
    report = component_summary([row])
    assert report['plan_call_outputs_with_diagnostic'] == 1
    assert report['positive_plan_call_outputs_with_diagnostic'] == 0
    assert report['diagnostic_is_not_guard_or_mcu_application'] is True


def test_new_current_retry_requires_matching_parent_publication():
    from audit_runtime import REACH_KEY,reach_summary
    current=dict(reference_s=2.,steering_rad=.2,speed_mps=.2,
                 **{REACH_KEY:dict(applied=True,reason='single_current_crop_reach_accepted')})
    rows=[dict(kind='call',data=dict(current_rebase={'output':current},delivery={'kind':'candidate'}))]
    assert reach_summary(rows)['calculations_accepted']==1
    assert reach_summary(rows)['adjacent_matching_parent_publications']==0
    publication=dict(kind='route_publication',data=dict(retained=True,reference_s=2.,steering_rad=.2,speed_mps=.2))
    assert reach_summary(rows+[publication])['adjacent_matching_parent_publications']==1
    assert reach_summary(rows+[dict(kind='reset',data={}),publication])['adjacent_matching_parent_publications']==0

    assert reach_summary(rows,crawl_only=True)['calculations_accepted']==0
    current[REACH_KEY]['crawl_retry']=dict(applied=True,result=dict(reason='single_current_crop_reach_accepted',proof=dict(reason='bounded_same_crop_current_reach_risk')))
    assert reach_summary(rows,crawl_only=True)['calculations_accepted']==1
    assert reach_summary(rows,crawl_only=True)['adjacent_matching_parent_publications']==0
    assert reach_summary(rows+[publication],crawl_only=True)['adjacent_matching_parent_publications']==1
    assert reach_summary(rows+[dict(kind='reset',data={}),publication],crawl_only=True)['adjacent_matching_parent_publications']==0
