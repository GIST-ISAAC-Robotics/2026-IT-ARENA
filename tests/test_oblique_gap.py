"""사선 공백의 저속 후보와 유지할 실제 고장/가까운 소실 반례."""
import math
import numpy as np
import pytest
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_partial_observation import sample
from test_local_follow import history


def ready(oblique=False):
    p=PartialObservationPolicy(True,True,True,oblique_metric_enabled=oblique);h=history()
    p.evaluate(sample(.4),[],h,.5,.5)
    return p,h


def test_original_remains_rejecting_the_oblique_gap():
    p,h=ready()
    assert p.evaluate(sample(.5,range(301,306)),[],h,.6,.5)['reason']=='lateral_metric_outside_trial'


def test_oblique_gap_keeps_raw_unknown_and_only_allows_point_two():
    p,h=ready(True);raw=sample(.5,range(301,306));before=raw.ranges.copy()
    r=p.evaluate(raw,[],h,.6,.5)
    assert r['allowed'] and r['speed_cap_mps']==.2 and not r['unknown_filled']
    assert r['residual_hidden_object_risk'] and r['lateral_metric']['missing_chord_m']<.05
    np.testing.assert_equal(raw.ranges,before)


@pytest.mark.parametrize('case',['central','wide','high_speed','stale','near_flank','under_range','all_missing'])
def test_direct_front_or_unsupported_input_still_rejected(case):
    p,h=ready(True);raw=sample(.5,range(301,306));speed=.5;now=.6
    if case=='central':raw=sample(.5,range(246,255))  # 단일 저속 후보의 총6도 범위도 넘음
    if case=='wide':raw=sample(.5,range(301,310))  # 6도 초과; 7광선은 실제 반경으로 별도 검사
    if case=='high_speed':speed=.61
    if case=='stale':now=.81
    if case=='near_flank':raw.ranges[300]=.9
    if case=='under_range':raw.ranges[304]=.01
    if case=='all_missing':raw=sample(.5,range(500))
    assert not p.evaluate(raw,[],h,now,speed)['allowed']


def test_near_disappearing_return_is_still_latched():
    p,h=ready(True);angle=math.radians(38.)
    p._remember([[.06+.16*math.cos(angle),.16*math.sin(angle)]],.5,.4,True)
    r=p.evaluate(sample(.5,range(301,306)),[],h,.6,.5)
    assert not r['allowed'] and r['reason']=='known_obstacle_latched'


def test_rotation_to_central_uses_same_bounded_single_gap():
    p,h=ready(True);h=history(w=4.)
    result=p.evaluate(sample(.5,range(301,306)),[],h,.7,.5)
    assert result['allowed'] and result['speed_cap_mps']==.2
    assert not result['unknown_filled']
    assert result['lateral_metric']['missing_chord_m']<=.05


def test_rotation_outside_front_observation_domain_still_rejected():
    p,h=ready(True);h=history(w=-8.)
    assert not p.evaluate(sample(.5,range(301,306)),[],h,.7,.5)['allowed']



def test_explicit_policy_prerequisites():
    for args in [(False,False,False),(True,False,True),(True,True,False)]:
        with pytest.raises(ValueError,match='complete v4'):
            PartialObservationPolicy(*args,oblique_metric_enabled=True)
    with pytest.raises(ValueError):
        PartialObservationPolicy(True,True,True,oblique_metric_enabled='true')


def test_follow_keeps_default_off_and_only_selects_solo_v4():
    from dataclasses import replace
    from arena_autonomy.local_follow import FollowConfig,LocalFollow
    assert not FollowConfig().oblique_gap_trial
    with pytest.raises(ValueError,match='solo v4'):
        FollowConfig(oblique_gap_trial=True)
    c=FollowConfig(solo_route_only=True,partial_observation_policy='bounded_local_v4',oblique_gap_trial=True)
    assert LocalFollow(c).partial_policy.oblique_metric_enabled
    assert not LocalFollow(replace(c,oblique_gap_trial=False)).partial_policy.oblique_metric_enabled
