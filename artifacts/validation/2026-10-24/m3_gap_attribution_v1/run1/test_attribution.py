"""간격 분해 수식과 결합 계약의 독립 단위 검사. 실제 주행 증명이 아니다."""
import math

import pytest

from analyze import join_poses, polygon, split_change


@pytest.mark.parametrize('angle', [0., .2, -.3, 1.0])
def test_sum_and_exchange_symmetry(angle):
    e0, e1 = (0, 0., 0., 0.), (1, .05, .02, angle)
    a0, a1 = (0, .3, .2, -.1), (1, .27, .21, .1)
    ep0, ep1, ap0, ap1 = map(polygon, (e0, e1, a0, a1))
    d00, d10, d01, d11 = ep0.distance(ap0), ep1.distance(ap0), ep0.distance(ap1), ep1.distance(ap1)
    de, da = split_change(d00, d10, d01, d11)
    ae, ea = split_change(d00, d01, d10, d11)
    assert de+da == pytest.approx(d11-d00, abs=1e-12)
    assert de == pytest.approx(ea, abs=1e-12)
    assert da == pytest.approx(ae, abs=1e-12)


def test_stationary_ego_has_zero_contribution():
    de, da = split_change(.3, .3, .2, .2)
    assert de == 0 and da == pytest.approx(-.1)


def test_overlap_is_not_signed_penetration():
    assert polygon((0, 0, 0, 0)).distance(polygon((0, .05, 0, 0))) == 0
    assert split_change(0., 0., 0., 0.) == (0., 0.)


@pytest.mark.parametrize('bad', [math.nan, math.inf, -.01])
def test_invalid_distance_rejected(bad):
    with pytest.raises(ValueError):
        split_change(.1, bad, .2, .1)


def test_asof_does_not_use_future_actor_pose():
    joined = join_poses([(1., 0., 0., 0.), (1.01, 0., 0., 0.)],
                        [(.99, 1., 0., 0.), (1.005, 2., 0., 0.)], 1., 1.01)
    assert [a[1] for _, a in joined] == [1., 2.]


def test_stale_actor_is_not_silently_joined():
    with pytest.raises(ValueError, match='missing as-of'):
        join_poses([(1., 0., 0., 0.), (1.01, 0., 0., 0.)],
                   [(.90, 1., 0., 0.), (1.005, 2., 0., 0.)], 1., 1.01)
