import math

import numpy as np
import pytest

from arena_autonomy.lidar_observation import audit_front_observation


_DEFAULT_RANGES = object()


def audit(ranges=_DEFAULT_RANGES, **kwargs):
    defaults = dict(angle_min=-math.pi, angle_increment=math.radians(1),
                    range_min=.05, range_max=12.)
    defaults.update(kwargs)
    values = np.full(360, 2.) if ranges is _DEFAULT_RANGES else ranges
    return audit_front_observation(values, **defaults)


def test_normal_finite_scan_and_inclusive_front_boundaries():
    result = audit()
    assert result.reason == 'ok'
    assert result.forward_samples == 201
    assert result.unknown_samples == 0


@pytest.mark.parametrize('distance', [.05, 12.])
def test_range_min_and_max_are_inclusive(distance):
    assert audit(np.full(360, distance)).reason == 'ok'


@pytest.mark.parametrize('distance', [math.inf, -math.inf, math.nan, .049, 12.001])
@pytest.mark.parametrize('indices', [[180], list(range(150, 190)), [80], [280]])
def test_single_contiguous_and_boundary_unknowns_stop(distance, indices):
    ranges = np.full(360, 2.)
    ranges[indices] = distance
    before = ranges.copy()
    result = audit(ranges)
    assert result.reason == 'front_unobserved'
    assert result.unknown_samples == len(indices)
    np.testing.assert_equal(ranges, before)


def test_all_infinite_scan_has_no_finite_front_observations():
    result = audit(np.full(360, math.inf))
    assert result.reason == 'front_unobserved'
    assert result.unknown_samples == result.forward_samples == 201


def test_only_rear_unknown_is_intentionally_ignored():
    ranges = np.full(360, 2.)
    ranges[:80] = math.inf
    ranges[281:] = math.nan
    assert audit(ranges).reason == 'ok'


def test_yaw_and_wrapping_are_in_body_coordinates():
    ranges = np.full(360, 2.)
    ranges[0] = math.inf
    assert audit(ranges).reason == 'ok'
    assert audit(ranges, lidar_yaw_rad=math.pi).reason == 'front_unobserved'
    assert audit(angle_min=3 * math.pi).reason == 'ok'
    assert audit(lidar_yaw_rad=4 * math.pi).forward_samples == 201


@pytest.mark.parametrize('increment', [0., -.01, math.nan, math.inf, -math.inf,
                                      math.radians(2.01)])
def test_invalid_increment(increment):
    assert audit(angle_increment=increment).reason == 'scan_invalid'


@pytest.mark.parametrize('kwargs', [
    {'angle_min': math.inf}, {'angle_min': math.nan},
    {'range_min': -.1}, {'range_min': 12.}, {'range_max': math.inf},
    {'range_max': math.nan}, {'range_max': 0.},
    {'lidar_yaw_rad': math.inf}, {'half_angle_rad': 0.},
    {'half_angle_rad': math.pi + .01}, {'half_angle_rad': math.nan},
    {'angle_increment': None}, {'angle_min': [0.]}, {'range_max': '12'},
])
def test_invalid_metadata_returns_diagnostic(kwargs):
    assert audit(**kwargs).reason == 'scan_invalid'


@pytest.mark.parametrize('ranges', [[], [2.], 2., None, [[2., 2.]],
                                   [[2.], [2., 2.]], ['bad'] * 360,
                                   [None] * 360, [True] * 360,
                                   np.full(360, 2 + 1j)])
def test_malformed_arrays_return_diagnostic(ranges):
    assert audit(ranges).reason == 'scan_invalid'


@pytest.mark.parametrize('count', [30, 200, 358, 362, 720])
def test_narrow_fov_misleading_metadata_and_repeated_wrap_are_invalid(count):
    assert audit(np.full(count, 2.)).reason == 'scan_invalid'


def test_standard_inclusive_endpoint_is_allowed_but_extra_duplicate_wrap_is_not():
    assert audit(np.full(361, 2.)).reason == 'ok'
    assert audit(np.full(362, 2.)).reason == 'scan_invalid'


def test_single_sample_rotation_tolerance_does_not_hide_front_gap():
    # 359개×1°: wrap seam에 정상 간격보다 큰 2° 공백이 있다.
    assert audit(np.full(359, 2.)).reason == 'ok'  # 공백은 후방에만 있다.
    result = audit(np.full(359, 2.), angle_min=0.)
    assert result.reason == 'front_unobserved'
    assert result.unknown_samples == 0


def test_float32_metadata_and_two_degree_sampling_contract():
    assert audit(angle_min=np.float32(-math.pi),
                 angle_increment=np.float32(2 * math.pi / 360)).reason == 'ok'
    assert audit(np.full(180, 2.), angle_increment=math.radians(2)).reason == 'ok'
    assert audit(np.full(100, 2.), angle_increment=2 * math.pi / 100).reason == 'scan_invalid'


@pytest.mark.parametrize('count', [181, 361, 501, 513, 721, 1025])
def test_inclusive_endpoint_accepts_float32_rounding_over_whole_rotation(count):
    assert audit(np.full(count, 2.), angle_min=np.float32(-math.pi),
                 angle_increment=np.float32(2 * math.pi / (count - 1))).reason == 'ok'


def test_required_sector_without_sample_stops():
    result = audit(angle_min=math.radians(.5), half_angle_rad=math.radians(.1))
    assert result.reason == 'front_unobserved'
    assert result.forward_samples == result.unknown_samples == 0
