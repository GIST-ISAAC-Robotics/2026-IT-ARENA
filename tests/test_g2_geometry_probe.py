import math

import pytest

from g0_geometry import TruthBox
from g2_geometry_probe import ProbeConfig, quintic, sample_path, assess


def test_quintic_boundary_and_derivative_bounds():
    assert quintic(0) == (0, 0, 0)
    assert quintic(1) == (1, 0, 0)
    for i in range(1001):
        f, d, dd = quintic(i/1000)
        assert -1e-12 <= f <= 1+1e-12
        assert 0 <= d <= 1.875+1e-12
        assert abs(dd) <= 10/math.sqrt(3)+1e-12


@pytest.mark.parametrize('offset', [-.15, -.1, 0., .1, .15])
def test_path_starts_centered_rejoins_and_respects_vertex_motion_bound(offset):
    path = sample_path(offset, .9)
    rows = path['samples']
    assert rows[0]['polygon'].centroid.x == pytest.approx(0)
    for row in (rows[0], rows[-1]):
        assert row['rear_y_m'] == 0
        assert row['yaw_rad'] == 0
    for a, b, bound in zip(rows, rows[1:], path['interval_motion_bounds_m']):
        for va, vb in zip(a['polygon'].exterior.coords, b['polygon'].exterior.coords):
            assert math.dist(va, vb) <= bound+1e-12


@pytest.mark.parametrize('y', [0., .10, .15])
def test_inside_road_actor_does_not_get_positive_clearance_authorization(y):
    for offset in (-.15, -.125, -.10, 0., .10, .125, .15):
        result = assess(sample_path(offset, .9), TruthBox(1.5, y, .12))
        assert not result['geometry_conditions_met']
        assert not result['driving_authorized']


def test_partial_grass_actor_has_geometry_candidate_but_no_driving_permission():
    result = assess(sample_path(-.10, .9), TruthBox(1.5, .24, .12))
    assert result['geometry_conditions_met']
    assert result['minimum_body_gap_lower_m'] <= result['minimum_sampled_body_gap_m']
    assert not result['driving_authorized']
    assert not result['observation_verified']
    assert not result['tracking_verified']


def test_geometry_cannot_identify_invisible_low_car():
    path = sample_path(-.10, .9)
    assert assess(path, TruthBox(1.5, .24, .06)) == assess(path, TruthBox(1.5, .24, .12))


def test_mirror_symmetry():
    a = assess(sample_path(-.10, .9), TruthBox(1.5, .24, .12, yaw_rad=.3))
    b = assess(sample_path(.10, .9), TruthBox(1.5, -.24, .12, yaw_rad=-.3))
    for key in ('minimum_body_gap_lower_m', 'minimum_road_edge_lower_m', 'steering_upper_rad'):
        assert a[key] == pytest.approx(b[key], abs=1e-12)
    assert a['checks'] == b['checks']


@pytest.mark.parametrize('kwargs', [{'step_m': .03}, {'width_m': float('nan')},
                                   {'length_m': True}, {'max_steering_rad': math.pi/2}])
def test_invalid_config(kwargs):
    with pytest.raises(ValueError):
        ProbeConfig(**kwargs)


@pytest.mark.parametrize('offset,length', [(float('nan'), .9), (.4, .9), (.1, .1), (.1, 2), (True, .9)])
def test_invalid_path(offset, length):
    with pytest.raises(ValueError):
        sample_path(offset, length)
