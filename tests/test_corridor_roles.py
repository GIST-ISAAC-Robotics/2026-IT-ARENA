"""미연결 역할 구분: 좌우 대칭, 기존 성공 보존, 선호 경로/점유 의미의 경계."""
from pathlib import Path

import numpy as np
import pytest

import arena_autonomy.corridor_roles as module
from arena_autonomy.corner_corridor import corridor_evidence
from arena_autonomy.local_follow import FollowConfig
from test_corner_corridor_shadow import ring_scene


@pytest.mark.parametrize('inner,sign', [('left', 1), ('right', -1)])
@pytest.mark.parametrize('radius', [.075, .15, .30, .60])
def test_both_preferred_roles_keep_actual_points_and_geometry(inner, sign, radius):
    points = np.asarray(ring_scene(radius, sign).occupied_endpoints_xy_m)
    before = points.copy()
    cfg = FollowConfig()
    original = corridor_evidence(points, inner, cfg)
    assert original['supported'], original['reason']
    for preferred in ('left', 'right'):
        result = module.role_aware_evidence(points, preferred, cfg)
        assert result['supported'], result['reason']
        assert result['preferred_side'] == preferred
        assert result['inner_side'] == inner
        assert result['outer_side'] != inner
        np.testing.assert_array_equal(result['path'], original['path'])
        np.testing.assert_array_equal(points, before)
        assert result['witnesses'] == original['witnesses']
        assert result['role_selection'] == ('original_inner_role' if preferred == inner else 'preferred_outer_role')
        for name in ('wall_identity_certified', 'memory_refresh_authorized', 'free_area_certified',
                     'driving_authorized', 'unknown_gaps_filled', 'occupancy_evaluated',
                     'temporal_confirmation_evaluated', 'runtime_connected', 'branch_semantics_certified'):
            assert result[name] is False


@pytest.mark.parametrize('value', [None, 'bad', [], [[1, 2, 3]]*30, [[float('nan'), 0]]*30])
def test_invalid_points_are_rejected(value):
    result = module.role_aware_evidence(value, 'left', FollowConfig())
    assert not result['supported'] and result['path'] is None
    assert result['reason'] == 'invalid_input'


@pytest.mark.parametrize('side', [None, '', 'front'])
def test_invalid_side_does_not_choose_another_role(side):
    result = module.role_aware_evidence(ring_scene().occupied_endpoints_xy_m, side, FollowConfig())
    assert not result['supported'] and result['reason'] == 'invalid_input'


def test_no_inner_surface_stays_unsupported():
    p = np.column_stack((np.linspace(-1., 1.8, 60), np.full(60, -.425)))
    result = module.role_aware_evidence(p, 'right', FollowConfig())
    assert not result['supported'] and result['path'] is None
    assert not result['driving_authorized']


def test_original_success_never_replaced_by_another_role(monkeypatch):
    calls = []
    original = corridor_evidence(ring_scene().occupied_endpoints_xy_m, 'left', FollowConfig())
    def candidate(points, side, cfg):
        calls.append(side)
        return original
    monkeypatch.setattr(module, 'corridor_evidence', candidate)
    result = module.role_aware_evidence([], 'left', FollowConfig())
    assert calls == ['left']
    assert result['role_selection'] == 'original_inner_role'
    assert result['path'] is original['path']


@pytest.mark.parametrize('kind', ['missing', 'short', 'anchor', 'different_path'])
def test_alternative_cannot_replace_preferred_observation(monkeypatch, kind):
    points = ring_scene().occupied_endpoints_xy_m
    good, info = module.local_path(points, 'right', .425)
    if kind == 'missing':
        good = None
    elif kind == 'short':
        info = {**info, 'boundary_observed_ahead_m': .64}
    elif kind == 'anchor':
        info = {**info, 'wall_distance_m': .60}
    else:
        good = good+[0., .30]
    monkeypatch.setattr(module, 'local_path', lambda *args: (good, info))
    result = module.role_aware_evidence(points, 'right', FollowConfig())
    assert result['alternative_role']['supported']
    assert not result['supported'] and result['path'] is None
    assert result['role_rejection'] == 'preferred_observed_path_disagreement'


def test_no_operational_import_or_command_emission():
    root = Path(__file__).resolve().parents[1]
    package = root/'src/arena_autonomy/arena_autonomy'
    for name in ('local_follow.py', 'corridor_observer.py', 'local_pursuit.py', 'lidar_safety.py'):
        assert 'corridor_roles' not in (package/name).read_text()
    source = Path(module.__file__).read_text()
    assert not any(name in source for name in ('rclpy', 'gazebo', 'create_publisher'))
