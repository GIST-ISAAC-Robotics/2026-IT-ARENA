"""진단 사본의 실제 광선→점 취득 시각 대응과 소실/가림 반례."""
import math
import numpy as np
import pytest
from probe_timed_policy import policy_class
from test_local_follow import history, scan


def points(h, raw, now):
    first=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
    return h.project(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,
        raw.range_max,first,raw.time_increment,now,mode='both',timing_verified=True)[0]


def warm(index=None, distance=.15):
    cls,_,_=policy_class();p,h=cls(allow_narrow_forward=True),history()
    raw=scan(.4,[8.]*500)
    if index is not None:raw.ranges[index]=distance
    assert p.evaluate(raw,points(h,raw,.5),h,.5,.15)['allowed']
    return p,h


def test_close_hidden_point_is_not_forgotten():
    p,h=warm(375);raw=scan(.5,[8.]*500);raw.ranges[375]=math.inf
    r=p.evaluate(raw,points(h,raw,.6),h,.6,.15)
    assert not r['allowed'] and r['latch_reason']=='previous_point_in_missing_sweep'
    assert r['timed_missing_diagnostic']['possible_pairs']>0
    assert not r['unknown_filled']


def test_current_close_neighbor_still_blocks():
    p,h=warm();raw=scan(.5,[8.]*500);raw.ranges[375]=math.inf;raw.ranges[374]=.15
    assert p.evaluate(raw,points(h,raw,.6),h,.6,.15)['reason']=='near_gap_boundary'


@pytest.mark.parametrize('missing',[range(295,300),range(500)])
def test_wide_missing_stays_blocked(missing):
    p,h=warm();raw=scan(.5,[8.]*500)
    for i in missing:raw.ranges[i]=math.inf
    assert not p.evaluate(raw,points(h,raw,.6),h,.6,.15)['allowed']


def test_point_order_mismatch_is_rejected():
    p,h=warm();raw=scan(.5,[8.]*500)
    with pytest.raises(ValueError,match='order/coordinate'):
        p.evaluate(raw,points(h,raw,.6)[::-1],h,.6,.15)


def test_two_fresh_clear_scans_release_latch_not_same_scan_twice():
    p,h=warm(375);raw=scan(.5,[8.]*500);raw.ranges[375]=math.inf
    assert not p.evaluate(raw,points(h,raw,.6),h,.6,.15)['allowed']
    raw=scan(.6,[8.]*500)
    for now in [.7,.71]:assert not p.evaluate(raw,points(h,raw,now),h,now,.15)['allowed']
    raw=scan(.7,[8.]*500)
    assert p.evaluate(raw,points(h,raw,.8),h,.8,.15)['allowed']


def test_retained_times_follow_original_point_filter():
    p,h=warm(321,.55)
    assert list(p.point_times)==[(.4,.5)]
    np.testing.assert_allclose(p.point_times[(.4,.5)],[.4+321*.0002],rtol=0,atol=1e-12)
    p.reset();assert p.point_times=={}
