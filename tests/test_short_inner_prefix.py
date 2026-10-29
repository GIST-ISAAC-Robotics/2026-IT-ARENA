"""회전 오차 주행의 짧은 안쪽 지지: 점군 기하와 선택형 출력/제동 대조."""
import copy
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.corner_corridor import corridor_evidence, short_inner_prefix_evidence
from arena_autonomy.corridor_observer import CorridorObserver, current_proposal
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import LocalScene, SensorRay
from test_local_follow import history

POINTS = np.asarray(json.loads((Path(__file__).parent / 'fixtures/short_inner_boundary_points.json').read_text())['points'])


def config(enabled=True):
    return FollowConfig(boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate',
        corridor_fallback_enabled=True, short_boundary_prefix_enabled=enabled)


def scene(now=.4, side='left', obstacle=False, unknown=False):
    points = POINTS.copy()
    if side == 'right':
        points[:, 1] *= -1
    if obstacle:
        points = np.vstack((points, [[.15, 0.]]))
    rays = []
    first = now - .1
    for i, point in enumerate(points):
        direction = point - [.06, 0.]
        direction /= np.linalg.norm(direction)
        rays.append(SensorRay((.06, 0.), tuple(point), first, 'return', 'laser_frame', 0, i, tuple(direction)))
    if unknown:
        rays.append(SensorRay((.06, 0.), None, first, 'unknown', 'laser_frame', 0, len(rays), (1., 0.)))
    return LocalScene(now, first, first, 'laser_frame', 0, tuple(rays))


@pytest.mark.parametrize('side', ['left', 'right'])
def test_observed_inner_prefix_is_truncated_without_extending_or_modifying_points(side):
    sample = scene(side=side)
    points = np.asarray(sample.occupied_endpoints_xy_m)
    before = points.copy()
    old = corridor_evidence(points, side, config())
    assert old['reason'] == 'inner_surface_support_unavailable'
    result = short_inner_prefix_evidence(old, config())
    assert result['supported'] and result['short_prefix']
    assert .25 < result['base_observed_ahead_m'] < .65
    outer = np.asarray(old['outer_info']['path_points'])
    np.testing.assert_array_equal(result['path'], outer[:len(result['path'])])
    np.testing.assert_array_equal(points, before)
    assert result['inner_identity'] == 'unknown'
    assert not result['driving_authorized'] and not result['memory_refresh_authorized']


@pytest.mark.parametrize('side', ['left', 'right'])
def test_selection_remains_off_by_default_and_caps_speed_with_observed_budget(side):
    sample = scene(side=side)
    old = current_proposal(sample, 0., side, 0., .02, config(False))
    assert old['reason'] == 'inner_surface_support_unavailable'
    new = current_proposal(sample, 0., side, 0., .02, config())
    assert new['short_prefix'] and new['point_occupancy_supported']
    assert 0 < new['proposed_speed_mps'] <= .35
    assert new['stop_budget_m'] <= new['path_budget_m']
    assert new['path_budget_m'] <= new['geometry']['base_observed_ahead_m'] - .13 + 1e-10
    assert new['all_return_points_retained']


@pytest.mark.parametrize('fault', ['obstacle', 'unknown', 'stale', 'reverse', 'high_speed', 'frame'])
def test_observed_obstacles_and_motion_faults_remain_holds(fault):
    sample = scene(obstacle=fault == 'obstacle', unknown=fault == 'unknown')
    if fault == 'stale':
        sample = replace(sample, t_ref_s=1.)
    if fault == 'frame':
        sample = replace(sample, source_frame_id='invalid',
            rays=tuple(replace(ray, source_frame_id='invalid') for ray in sample.rays))
    speed = -0.1 if fault == 'reverse' else 2. if fault == 'high_speed' else 0.
    result = current_proposal(sample, speed, 'left', 0., .02, config())
    assert not result['point_occupancy_supported'] and result['proposed_speed_mps'] == 0.
    if fault == 'high_speed':
        assert result['reason'] == 'short_prefix_stop_budget'


@pytest.mark.parametrize('fault', ['wrong_reason', 'no_support', 'too_short', 'not_corner', 'long_outer_missing', 'bad_metadata'])
def test_uncorroborated_geometry_is_not_a_short_prefix(fault):
    old = corridor_evidence(POINTS, 'left', config())
    bad = copy.deepcopy(old)
    if fault == 'wrong_reason':
        bad['reason'] = 'not_consistent_inward_corner'
    if fault == 'no_support':
        bad['rejected'] = []
    if fault == 'too_short':
        for item in bad['rejected']:
            item['end_exclusive'] = 5
    if fault == 'not_corner':
        for item in bad['rejected']:
            item['turn_rad'] = .1
    if fault == 'long_outer_missing':
        bad['outer_info']['boundary_observed_ahead_m'] = .4
    if fault == 'bad_metadata':
        bad['outer_info']['wall_distance_m'] = float('nan')
    assert not short_inner_prefix_evidence(bad, config())['supported']


def test_current_distinct_scans_are_required_and_unknown_discards_confirmation():
    observer = CorridorObserver(config())
    h = history()
    assert not observer.evaluate(scene(.4), h, 0., 'left')['positive']
    assert not observer.evaluate(scene(.5), h, 0., 'left')['positive']
    assert not observer.evaluate(scene(.5), h, 0., 'left')['positive']
    result = observer.evaluate(scene(.6), h, 0., 'left')
    assert result['positive'] and result['short_prefix']
    assert not observer.evaluate(scene(.7, unknown=True), h, 0., 'left')['positive']
    assert not observer.evaluate(scene(.8), h, 0., 'left')['positive']


def test_limited_follow_does_not_promote_short_inner_surface_to_wall_memory():
    controller = LocalFollow(config())
    results = [controller.command(scene(t), history(), 0., 'left') for t in (.4, .5, .6, .7)]
    assert any(row[0] > 0 for row in results)
    for speed, _, detail in results:
        if speed > 0:
            assert speed <= .35 and detail['unfiltered_occupancy']
            assert detail['corridor_selected'] and not detail['boundary_memory_from_corridor']
    assert controller.path is None
