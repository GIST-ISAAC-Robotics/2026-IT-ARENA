"""선택형 단독 회귀의 실행 경계·완주·밀집 정지 증거."""
import ast
from pathlib import Path

import pytest
from g1_traffic_eval import stop_pose_evidence

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'scripts/validate_basic_autonomy.py'


def helpers():
    tree = ast.parse(SOURCE.read_text())
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and
             n.name in ('g1_solo_arguments', 'solo_route_complete')]
    scope = {'Path': Path}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(SOURCE), 'exec'), scope)
    return scope


def test_default_launch_stays_unmodified():
    assert helpers()['g1_solo_arguments'](False, 'connected', 'oldest', False, '/tmp') == []


def test_r0_solo_accepts_existing_one_mps_profile_without_default_change():
    """교통 비교와 동일한 기존 프로필을 단독 CLI에서도 명시 선택한다."""
    tree = ast.parse(SOURCE.read_text())
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
             and isinstance(node.func, ast.Attribute) and node.func.attr == 'add_argument'
             and node.args and isinstance(node.args[0], ast.Constant)
             and node.args[0].value == '--speed-profile']
    assert len(calls) == 1
    options = {kw.arg: ast.literal_eval(kw.value) for kw in calls[0].keywords}
    assert options['default'] == 'cautious'
    assert 'g1_compare_1mps' in options['choices']


@pytest.mark.parametrize('enabled,boundary,occupancy,record', [
    (False,'connected','oldest',False), (True,'connected','per_ray_candidate',True),
    (True,'corner_distance_candidate','oldest',True), (True,'corner_distance_candidate','per_ray_candidate',False)])
def test_corridor_requires_all_explicit_dependencies(enabled,boundary,occupancy,record):
    with pytest.raises(ValueError):
        helpers()['g1_solo_arguments'](enabled,boundary,occupancy,record,'/tmp',True)


def test_corridor_choice_is_recorded_and_default_flag_is_absent():
    method=helpers()['g1_solo_arguments']
    assert 'g1_corridor_fallback_enabled:=true' in method(True,'corner_distance_candidate','per_ray_candidate',True,'/tmp',True)
    assert not any('corridor' in p for p in method(True,'corner_distance_candidate','per_ray_candidate',True,'/tmp'))


def test_observation_option_requires_corridor_and_remains_off_by_default():
    method=helpers()['g1_solo_arguments']
    with pytest.raises(ValueError):
        method(True,'corner_distance_candidate','per_ray_candidate',True,'/tmp',False,True)
    assert 'g1_recent_observation_enabled:=true' in method(
        True,'corner_distance_candidate','per_ray_candidate',True,'/tmp',True,True)
    assert not any('recent_observation' in p for p in method(
        True,'corner_distance_candidate','per_ray_candidate',True,'/tmp',True))


@pytest.mark.parametrize('boundary,occupancy,record', [
    ('fragmented_candidate', 'oldest', False), ('connected', 'per_ray_candidate', True),
    ('connected', 'oldest', True)])
def test_no_silent_g1_activation(boundary, occupancy, record):
    with pytest.raises(ValueError):
        helpers()['g1_solo_arguments'](False, boundary, occupancy, record, '/tmp')


def test_candidate_requires_record_and_passes_exact_options():
    method = helpers()['g1_solo_arguments']
    with pytest.raises(ValueError):
        method(True, 'fragmented_candidate', 'per_ray_candidate', False, '/tmp')
    assert method(True, 'fragmented_candidate', 'per_ray_candidate', True, '/tmp') == [
        'g1_boundary_evidence_mode:=fragmented_candidate', 'g1_occupancy_timing_mode:=per_ray_candidate',
        'g1_trace_path:=/tmp/follow_decisions.jsonl']


def test_corner_candidate_is_explicit_and_requires_record_even_with_oldest():
    method = helpers()['g1_solo_arguments']
    with pytest.raises(ValueError, match='recording'):
        method(True, 'corner_anchor_candidate', 'oldest', False, '/tmp')
    with pytest.raises(ValueError, match='G1-only'):
        method(False, 'corner_anchor_candidate', 'oldest', True, '/tmp')
    assert method(True, 'corner_anchor_candidate', 'oldest', True, '/tmp')[0] == \
        'g1_boundary_evidence_mode:=corner_anchor_candidate'
    with pytest.raises(ValueError):
        method(True, 'corner_distance_candidate', 'oldest', False, '/tmp')
    assert method(True, 'corner_distance_candidate', 'oldest', True, '/tmp')[0] == \
        'g1_boundary_evidence_mode:=corner_distance_candidate'


@pytest.mark.parametrize('b,o', [('bad', 'oldest'), ('connected', 'bad')])
def test_unknown_candidates_rejected(b, o):
    with pytest.raises(ValueError):
        helpers()['g1_solo_arguments'](True, b, o, True, '/tmp')


def test_route_requires_distance_even_with_all_markers():
    route = helpers()['solo_route_complete']
    assert not route(8., 30., True, {20, 30})
    assert not route(30., 30., True, {20})
    assert route(30., 30., True, {20, 30})
    assert route(8., 8., False, set())


def test_dense_stop_rejects_sparse_and_lateral_slide():
    assert not stop_pose_evidence([(0., 0., 0., 0.), (.5, 0., 0., 0.)], .5, .5)['passed']
    assert not stop_pose_evidence([(i*.01, 0., i*.0002, 0.) for i in range(51)], .5, .5)['passed']
    assert stop_pose_evidence([(i*.01, 0., 0., 0.) for i in range(51)], .5, .5)['passed']


def test_candidate_snapshot_and_failure_cleanup_are_not_omitted():
    source = SOURCE.read_text()
    for path in ('boundary_evidence.py', 'corner_evidence.py', 'occupancy_timing.py', 'follow_trace.py', 'scripts/g1_traffic_eval.py'):
        assert path in source
    assert "report['failure_cleanup_stop'] = cleanup_stop" in source
    assert "report['stop_stable'] &= report['dense_pose_stop_evidence']['passed']" in source
    assert "pose_rejected_times == 0 and report['source_unchanged']" in source
