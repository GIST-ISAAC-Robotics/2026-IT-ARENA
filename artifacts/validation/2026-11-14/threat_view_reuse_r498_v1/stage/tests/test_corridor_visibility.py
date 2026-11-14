"""인접 광선의 실제 깊이/unknown 보존. 표본 사이의 자유 공간 증명은 아니다."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from audit_corridor_visibility import bracket_depth, path_visibility, footprint_queries


def scene(distance=2.):
    return scene_from_scan([distance]*500, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., t_ref_s=.4, acquired_at_s=(.3+np.arange(500)*.0002).tolist())


def query(angle=.003, distance=1.):
    return [.06+distance*math.cos(angle), distance*math.sin(angle)]


def test_bracketing_return_depth_is_only_sampled_support():
    result = bracket_depth(scene(), [query()], np.full(500, .055))[0]
    assert result['supported'] and result['source_indices'] == [250, 251]
    assert result['free_area_certified'] is False
    assert min(result['depth_margins_m']) == pytest.approx(.945)


def test_exact_beam_does_not_require_interpolation():
    result = bracket_depth(scene(), [query(0.)], np.zeros(500))[0]
    assert result['supported'] and result['source_indices'] == [250, 250]


@pytest.mark.parametrize('status', ['unknown', 'masked'])
def test_one_neighbor_unknown_is_never_filled(status):
    original = scene()
    rays = list(original.rays)
    rays[251] = replace(rays[251], status=status, hit_xy_m=None)
    result = bracket_depth(replace(original, rays=tuple(rays)), [query()], np.zeros(500))[0]
    assert not result['supported'] and result['reason'] == 'bracket_unknown_or_masked'


@pytest.mark.parametrize('distance,padding', [(2.1,0.), (1.96,.055), (2.,0.)])
def test_behind_first_return_or_within_margin_is_not_support(distance, padding):
    result = bracket_depth(scene(), [query(distance=distance)], np.full(500, padding))[0]
    assert not result['supported']
    assert result['reason'] == 'return_at_or_before_query_margin'


def test_missing_source_index_is_not_joined_as_adjacent_beams():
    original = scene()
    reduced = replace(original, rays=tuple(r for r in original.rays if r.source_index != 251))
    result = bracket_depth(reduced, [query()], np.zeros(499))[0]
    assert not result['supported'] and result['reason'] == 'angular_bracket_unavailable'


def test_rear_mask_is_still_unknown():
    result = bracket_depth(scene(), [query(math.pi-.01)], np.zeros(500))[0]
    assert not result['supported']


def test_per_ray_origin_is_used_instead_of_current_sensor_origin():
    original = scene()
    rays = tuple(replace(r, ray_origin_xy_m=(r.ray_origin_xy_m[0]+.5, r.ray_origin_xy_m[1]),
                        hit_xy_m=None if r.hit_xy_m is None else (r.hit_xy_m[0]+.5,r.hit_xy_m[1]))
                 for r in original.rays)
    moved = replace(original, rays=rays)
    result = bracket_depth(moved, [query(distance=2.2)], np.zeros(500))[0]
    assert result['supported']


def test_current_body_samples_are_excluded_not_called_free():
    path = np.column_stack((np.linspace(-.1,.8,46), np.zeros(46)))
    result = path_visibility(scene(), path, FollowConfig())
    assert result['depth_supported_prefix_m'] > .7
    assert result['current_body_queries_excluded'] > 0
    assert result['query_samples'] > 0
    for key in ('between_ray_volume_observed', 'free_area_certified', 'driving_authorized', 'continuous_sweep_verified'):
        assert result[key] is False


def test_short_return_limits_supported_prefix_without_erasing_failure():
    path = np.column_stack((np.linspace(-.1,1.,56), np.zeros(56)))
    result = path_visibility(scene(.55), path, FollowConfig())
    assert 0. < result['depth_supported_prefix_m'] < .5
    assert not result['whole_path_depth_supported']
    assert result['first_unsupported']['ray_evidence']['reason'] == 'return_at_or_before_query_margin'


def test_fitted_path_behind_ego_exposes_rear_mask_and_is_not_ignored():
    path = np.column_stack((np.linspace(-.1,.8,40), np.zeros(40)))
    result = path_visibility(scene(), path, FollowConfig())
    assert result['depth_supported_prefix_m'] == 0
    assert result['first_unsupported']['path_distance_m'] == 0
    assert result['first_unsupported']['ray_evidence']['reason'] == 'bracket_unknown_or_masked'


@pytest.mark.parametrize('q', [[[math.nan,0.]], [[1.,2.,3.]]])
def test_invalid_query_is_not_silently_supported(q):
    with pytest.raises(ValueError):
        bracket_depth(scene(), q, np.zeros(500))


def test_invalid_footprint_path_rejected():
    with pytest.raises(ValueError):
        footprint_queries([[0.,0.]]*3, FollowConfig())
