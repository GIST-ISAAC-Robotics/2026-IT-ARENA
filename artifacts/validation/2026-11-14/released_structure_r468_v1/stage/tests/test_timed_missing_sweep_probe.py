"""시각을 빠뜨리면 가능한 이동을 배제하는 반례와 기본 입력 계약."""
import math

import numpy as np
import pytest

from probe_timed_missing_sweep import timed_missing_sweep
from probe_joint_missing_sweep import joint_missing_sweep


def run(points, acquired=(0.,), missing=(.1,), now=.2, **kw):
    return timed_missing_sweep(points, acquired, [[0., 0.]], [0.], missing,
                               now, math.radians(.5), 1., 0., .1, 0., 0., **kw)


def test_motion_after_measurement_can_reach_body():
    # p=.32 -> q=.22 at t=.1 -> x=.17 at t=.2. Both steps fit 1m/s.
    # x is inside the buffered stopping sweep, whose front reaches .19m.
    # Ignoring the latter step incorrectly excludes the old point.
    distance, _ = joint_missing_sweep([[.32, 0.]], [[0., 0.]], [0.],
        math.radians(.5), 1., 0., .1, 0., 0.)
    assert distance[0] > .1
    out = run([[.32, 0.]], motion_bound=1., radial_margin=0.)
    assert out['possible'] == [True]
    assert out['after_radius_m'] == pytest.approx([.1])


def test_unreachable_old_point_remains_excluded():
    assert run([[1., 1.]], motion_bound=1.)['possible'] == [False]


def test_zero_motion_matches_static_intersection():
    points = np.array([[.2, .1], [.1, 0.], [1., 1.]])
    out = run(points, acquired=[0.]*3, motion_bound=0.)
    distance, _ = joint_missing_sweep(points, [[0., 0.]], [0.], math.radians(.5),
                                     1., 0., .1, 0., 0.)
    assert out['possible'] == (distance <= .03 + 1e-10).tolist()


@pytest.mark.parametrize('acquired,missing,now', [([.15],[.1],.2), ([0.],[.3],.2),
                                               ([float('nan')],[.1],.2)])
def test_noncausal_times_rejected(acquired, missing, now):
    with pytest.raises(ValueError):
        run([[0., 0.]], acquired, missing, now)


def test_larger_motion_bound_never_discards_possible_points():
    points = np.array([[x,y] for x in np.linspace(-.3,.7,11)
                       for y in np.linspace(-.3,.3,7)])
    low = run(points, [0.]*len(points), motion_bound=.5)['possible']
    high = run(points, [0.]*len(points), motion_bound=2.)['possible']
    assert all(not a or b for a,b in zip(low,high))
