"""주행 미연결 회랑 가설. 기하 지지와 장애물/벽 분류를 구별한다."""
from dataclasses import replace
import math

import numpy as np
import pytest

import probe_corner_corridor as probe
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from probe_corner_observability import annular_scan
from probe_short_corner_support import compact_circle_fixture
from test_corner_raycast import elbow
from test_local_follow import cast_scene


def ring_scene(radius=.075, sign=1, now=.4):
    ranges, _, _, _, _ = annular_scan(radius, sign)
    return scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=math.tau/500,
                           range_min=.05, range_max=12., t_ref_s=now,
                           acquired_at_s=(now-.1+np.arange(500)*.0002).tolist(),
                           rear_half_angle_deg=30.)


def controlled_reference(monkeypatch, sign=1, ahead=1.2, anchor=.425):
    points, path, _ = compact_circle_fixture(sign)
    # 경로 생성 자체가 아니라 후단 가설 계약을 분리해 검사한다.
    def fitted(*args):
        return path.copy(), dict(reason='controlled_fixture', boundary_observed_ahead_m=ahead,
                                 wall_distance_m=anchor)
    monkeypatch.setattr(probe, 'local_path', fitted)
    return np.vstack((points, np.column_stack((np.linspace(-1., 1., 40), np.full(40, -sign*2.)))))


@pytest.mark.parametrize('side,sign', [('left',1), ('right',-1)])
def test_compact_surface_can_support_geometry_but_never_becomes_a_wall(monkeypatch, side, sign):
    points = controlled_reference(monkeypatch, sign)
    result = probe.paired_geometry(points, side, FollowConfig())
    assert result['geometry_supported'], result
    assert result['common_prefix_m'] >= .65
    assert max(w['observed_extent_m'] for w in result['witnesses']) < .20
    assert result['inner_identity'] == 'unknown'
    assert result['inner_semantic_hypotheses'] == ['boundary_surface', 'other_occupied_object']
    for field in ('wall_identity_certified', 'memory_refresh_authorized', 'free_area_certified',
                  'driving_authorized', 'unknown_gaps_filled'):
        assert result[field] is False


@pytest.mark.parametrize('ahead,anchor', [(.64,.425), (1.2,.60)])
def test_long_outer_length_and_anchor_are_still_required(monkeypatch, ahead, anchor):
    points = controlled_reference(monkeypatch, ahead=ahead, anchor=anchor)
    result = probe.paired_geometry(points, 'left', FollowConfig())
    assert not result['geometry_supported']
    assert result['reason'] == 'long_outer_reference_unavailable'


def test_absent_inner_surface_is_not_filled(monkeypatch):
    controlled_reference(monkeypatch)
    points = np.column_stack((np.linspace(-1.,1.,40), np.full(40,-2.)))
    result = probe.paired_geometry(points, 'left', FollowConfig())
    assert not result['geometry_supported']
    assert result['reason'] == 'inner_surface_support_unavailable'


def test_short_fake_surface_with_wrong_spacing_is_not_evidence(monkeypatch):
    points = controlled_reference(monkeypatch)
    points[:len(points)-40,1] += .30
    assert not probe.paired_geometry(points, 'left', FollowConfig())['geometry_supported']


def test_all_returns_reach_both_existing_sweep_checks(monkeypatch):
    controlled_reference(monkeypatch)
    scene = ring_scene()
    captured = []
    def sweep(points, angle, wheelbase, **kwargs):
        captured.append((np.array(points), np.array(kwargs['margin'])))
        return .50
    monkeypatch.setattr(probe, 'swept_limit', sweep)
    result = probe.shadow_scene(scene, 0., 'left', 0., .02,
                                FollowConfig(occupancy_timing_mode='per_ray_candidate'))
    assert result['geometry_supported'], result
    assert result['preliminary_motion_supported']
    assert len(captured) == 2
    for points, padding in captured:
        np.testing.assert_array_equal(points, np.asarray(scene.occupied_endpoints_xy_m))
        assert len(padding) == len(points)
        assert np.min(padding) >= .055
    assert result['driving_authorized'] is False
    assert result['confirmation_evaluated'] is False


@pytest.mark.parametrize('clearance', [0., .16, .20, .279])
def test_occupancy_remains_a_separate_hold(monkeypatch, clearance):
    controlled_reference(monkeypatch)
    monkeypatch.setattr(probe, 'swept_limit', lambda *a, **kw: clearance)
    result = probe.shadow_scene(ring_scene(), 0., 'left', 0., .02, FollowConfig())
    assert result['geometry_supported'], result
    assert result['reason'] == 'existing_occupancy_margin_hold'
    assert result['proposed_speed_mps'] == 0
    assert not result['preliminary_motion_supported']


def test_previous_steering_sweep_can_veto_new_steering(monkeypatch):
    controlled_reference(monkeypatch)
    monkeypatch.setattr(probe, 'swept_limit', lambda p, angle, *a, **kw: .1 if angle == 0 else 1.)
    result = probe.shadow_scene(ring_scene(), 0., 'left', 0., .02, FollowConfig())
    assert result['steering_rad'] == pytest.approx(.06)
    assert result['swept_clearance_m'] == .1
    assert not result['preliminary_motion_supported']


@pytest.mark.parametrize('kind', ['unknown','stale','reverse'])
def test_existing_unknown_age_and_reverse_restrictions_remain(monkeypatch, kind):
    controlled_reference(monkeypatch)
    scene = ring_scene()
    if kind == 'unknown':
        rays = list(scene.rays)
        rays[250] = replace(rays[250], status='unknown', hit_xy_m=None)
        scene = replace(scene, rays=tuple(rays))
    elif kind == 'stale':
        scene = replace(scene, t_ref_s=.8)
    result = probe.shadow_scene(scene, -.01 if kind=='reverse' else 0., 'left', 0., .02, FollowConfig())
    assert result['proposed_speed_mps'] == 0
    assert not result['preliminary_motion_supported']
    assert result['reason'] == {'unknown':'front_unobserved','stale':'scene_stale','reverse':'reverse_motion'}[kind]


@pytest.mark.parametrize('value', [float('nan'),float('inf')])
def test_invalid_motion_never_proposes_command(value):
    result = probe.shadow_scene(ring_scene(), value, 'left', 0., .02, FollowConfig())
    assert result['reason'] == 'invalid_motion_state'
    assert result['driving_authorized'] is False


@pytest.mark.parametrize('kind', ['center_block', 'missing_inner'])
@pytest.mark.parametrize('sign', [1,-1])
def test_actual_first_return_block_or_missing_side_is_not_motion_support(kind, sign):
    world = (np.asarray(elbow(.30, kind))*[1,sign]).tolist()
    scene = cast_scene(.4, world)
    result = probe.shadow_scene(scene, 0., 'left' if sign==1 else 'right', 0., .02,
                                FollowConfig(occupancy_timing_mode='per_ray_candidate'))
    assert not result['preliminary_motion_supported'], result
    assert not result['driving_authorized']
