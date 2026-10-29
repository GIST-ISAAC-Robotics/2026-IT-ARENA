"""새 소실·현재 점유는 유지하고 같은 스캔 시간 팽창만 일시 대기로 나눈다."""
import math
import numpy as np
import pytest
from fresh_loss_candidate import FreshLossCandidate
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from test_local_follow import history,scan
from test_timed_policy_probe import points


def warm(cls=FreshLossCandidate,distance=.4):
    policy=cls(True,True,True);h=history();raw=scan(.4,[2.]*500)
    raw.ranges[375]=distance
    assert policy.evaluate(raw,points(h,raw,.5),h,.5,.15)['allowed']
    return policy,h


def missing(first=.5):
    raw=scan(first,[2.]*500);raw.ranges[375]=math.inf
    return raw


def test_same_scan_potential_stops_this_call_without_persistent_latch():
    p,h=warm();raw=missing()
    assert p.evaluate(raw,points(h,raw,.6),h,.6,.15)['allowed']
    result=p.evaluate(raw,points(h,raw,.68),h,.68,.15)
    assert not result['allowed'] and result['reason']=='same_scan_missing_wait'
    assert result['same_scan_wait']['possible_pairs']>0
    assert not p.latched and p.anchor_latch_time is None
    assert result['unknown_filled'] is False
    # 다음 실측에서도 좁은 결측은 그대로다. 장기 숨은 객체 부재 증명이 아니다.
    raw=missing(.6)
    result=p.evaluate(raw,points(h,raw,.7),h,.7,.15)
    assert result['allowed'] and result['reason']=='bounded_side_missing'


def test_original_model_latches_the_same_aged_scan():
    p,h=warm(RecoveryPolicy);raw=missing()
    assert p.evaluate(raw,points(h,raw,.6),h,.6,.15)['allowed']
    result=p.evaluate(raw,points(h,raw,.68),h,.68,.15)
    assert not result['allowed'] and p.latched


def test_current_reobserved_point_does_not_create_unnecessary_wait():
    p,h=warm();raw=missing();raw.ranges[374]=.4
    assert p.evaluate(raw,points(h,raw,.6),h,.6,.15)['allowed']
    result=p.evaluate(raw,points(h,raw,.68),h,.68,.15)
    assert result['timed_missing_diagnostic']['possible_pairs']>0
    assert result['reobserved_previous_points']>0
    assert result['allowed'] and not p.latched and p.same_scan_wait is None


def test_fresh_nearby_disappearance_still_latches():
    p,h=warm(distance=.15);raw=missing()
    result=p.evaluate(raw,points(h,raw,.6),h,.6,.15)
    assert not result['allowed'] and p.latched
    assert p.latch_reason=='previous_point_in_missing_sweep'
    before=(p.latch_time,p.latch_reason)
    assert not p.evaluate(raw,points(h,raw,.68),h,.68,.15)['allowed']
    assert p.latched and (p.latch_time,p.latch_reason)==before


@pytest.mark.parametrize('new_source',[True,False])
def test_actual_close_gap_boundary_is_not_deferred(new_source):
    p,h=warm();raw=missing()
    assert p.evaluate(raw,points(h,raw,.6),h,.6,.15)['allowed']
    raw=missing(.6 if new_source else .5);raw.ranges[374]=.15
    now=.7 if new_source else .68
    result=p.evaluate(raw,points(h,raw,now),h,now,.15)
    assert not result['allowed'] and p.latched and p.latch_reason=='near_gap_boundary'


@pytest.mark.parametrize('fault',['all_missing','near_invalid','stale','order'])
def test_faults_remain_stopped(fault):
    p,h=warm();raw=missing();now=.6
    if fault=='all_missing':raw.ranges=[math.inf]*500
    if fault=='near_invalid':raw.ranges[374]=.01
    if fault=='stale':now=.81
    projected=points(h,raw,now)
    if fault=='order':projected=projected[::-1]
    result=p.evaluate(raw,projected,h,now,.15)
    assert not result['allowed']


def test_two_independent_clear_scans_still_required_for_permanent_latch():
    p,h=warm(distance=.15);raw=missing()
    assert not p.evaluate(raw,points(h,raw,.6),h,.6,.15)['allowed']
    raw=scan(.6,[2.]*500)
    for now in (.7,.71):assert not p.evaluate(raw,points(h,raw,now),h,now,.15)['allowed']
    raw=scan(.7,[2.]*500)
    assert p.evaluate(raw,points(h,raw,.8),h,.8,.15)['allowed']


def test_default_and_independent_state_unchanged():
    from arena_autonomy.local_follow import FollowConfig
    assert not FollowConfig().traffic_recovery_trial
    a,h=warm();b,_=warm();raw=missing()
    a.evaluate(raw,points(h,raw,.6),h,.6,.15)
    a.evaluate(raw,points(h,raw,.68),h,.68,.15)
    assert a.same_scan_wait is not None and b.same_scan_wait is None
    a.reset();assert a.same_scan_wait is None and not a.assessment_fresh
