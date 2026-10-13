import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from probe_gapped_boundary import fragment_evidence


def geometry(intervals, offset=.425, slope=0.):
    return np.vstack([np.column_stack((xs, offset+slope*xs))
                      for xs in (np.linspace(a, b, n) for a,b,n in intervals)])


def reference():
    return np.column_stack((np.linspace(-.25, 1.8, 36), np.zeros(36)))


def test_separated_actual_fragments_not_gap_count_as_support():
    points = geometry([(-.25, .3, 31), (.9, 1.7, 31)])
    before = points.copy()
    result = fragment_evidence(points, reference(), 'left', FollowConfig())
    assert result['supported']
    assert len(result['fragments']) == 2
    assert result['unknown_gaps_m'] == pytest.approx([.6])
    assert .95 < result['observed_ahead_sum_m'] < 1.2
    assert result['occupied_points'] == len(points) and result['added_sensor_points'] == 0
    assert not result['is_drive_permission']
    np.testing.assert_array_equal(points, before)


def test_large_span_of_two_short_fragments_is_not_enough():
    result = fragment_evidence(geometry([(-.1, .12, 20), (1.5, 1.63, 12)]), reference(), 'left', FollowConfig())
    assert not result['supported']
    assert result['observed_ahead_sum_m'] < .4


@pytest.mark.parametrize('points', [geometry([(0., .20, 20)]),  # short parallel vehicle
                                  geometry([(.8, 1.8, 30)]),  # no near anchor
                                  geometry([(-.25, 1.8, 50)], offset=.20),  # vehicle side, not expected wall
                                  np.empty((0, 2)), np.array([[np.nan, .425]])])
def test_inadequate_support_never_confirms(points):
    assert not fragment_evidence(points, reference(), 'left', FollowConfig())['supported']


def test_occupancy_keeps_protruding_vehicle_even_when_wall_fragments_confirm():
    wall = geometry([(-.25, .3, 31), (.9, 1.7, 31)])
    vehicle = geometry([(.30, .50, 20)], offset=.20)
    result = fragment_evidence(np.vstack((wall, vehicle)), reference(), 'left', FollowConfig())
    assert result['supported']
    assert result['occupied_points'] == 82
    assert all(i < len(wall) for f in result['fragments'] for i in f['source_indices'])


def test_wrong_direction_reference_and_sparse_distant_points_rejected():
    sparse = geometry([(-.25, .3, 31), (.9, 1.7, 4)])
    assert not fragment_evidence(sparse, reference(), 'left', FollowConfig())['supported']
    backwards = reference()[::-1]
    assert not fragment_evidence(geometry([(-.25, 1.7, 60)]), backwards, 'left', FollowConfig())['supported']


def test_mirrored_geometry_has_equivalent_evidence():
    points = geometry([(-.25, .3, 31), (.9, 1.7, 31)])
    left = fragment_evidence(points, reference(), 'left', FollowConfig())
    right = fragment_evidence(points*[1, -1], reference(), 'right', FollowConfig())
    assert left['supported'] == right['supported']
    assert left['observed_ahead_sum_m'] == pytest.approx(right['observed_ahead_sum_m'])
