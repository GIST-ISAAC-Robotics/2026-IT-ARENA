"""사선 공백의 저속 후보와 유지할 실제 고장/가까운 소실 반례."""
import math
import numpy as np
import pytest
from arena_autonomy.partial_observation import PartialObservationPolicy
from candidate import install
from test_partial_observation import sample
from test_local_follow import history


@pytest.fixture
def candidate():
    original=install()
    try:yield
    finally:PartialObservationPolicy.evaluate=original


def ready():
    p=PartialObservationPolicy(True,True,True);h=history()
    p.evaluate(sample(.4),[],h,.5,.5)
    return p,h


def test_original_remains_rejecting_the_oblique_gap():
    p,h=ready()
    assert p.evaluate(sample(.5,range(301,306)),[],h,.6,.5)['reason']=='lateral_metric_outside_trial'


def test_oblique_gap_keeps_raw_unknown_and_only_allows_point_two(candidate):
    p,h=ready();raw=sample(.5,range(301,306));before=raw.ranges.copy()
    r=p.evaluate(raw,[],h,.6,.5)
    assert r['allowed'] and r['speed_cap_mps']==.2 and not r['unknown_filled']
    assert r['residual_hidden_object_risk'] and r['lateral_metric']['missing_chord_m']<.05
    np.testing.assert_equal(raw.ranges,before)


@pytest.mark.parametrize('case',['central','wide','high_speed','stale','near_flank','under_range','all_missing'])
def test_direct_front_or_unsupported_input_still_rejected(candidate,case):
    p,h=ready();raw=sample(.5,range(301,306));speed=.5;now=.6
    if case=='central':raw=sample(.5,range(249,254))
    if case=='wide':raw=sample(.5,range(301,308))
    if case=='high_speed':speed=.61
    if case=='stale':now=.81
    if case=='near_flank':raw.ranges[300]=.9
    if case=='under_range':raw.ranges[304]=.01
    if case=='all_missing':raw=sample(.5,range(500))
    assert not p.evaluate(raw,[],h,now,speed)['allowed']


def test_near_disappearing_return_is_still_latched(candidate):
    p,h=ready();angle=math.radians(38.)
    p._remember([[.06+.16*math.cos(angle),.16*math.sin(angle)]],.5,.4,True)
    r=p.evaluate(sample(.5,range(301,306)),[],h,.6,.5)
    assert not r['allowed'] and r['reason']=='known_obstacle_latched'


def test_rotation_that_transports_the_gap_to_central_still_rejected(candidate):
    p,h=ready();h=history(w=4.)
    assert not p.evaluate(sample(.5,range(301,306)),[],h,.7,.5)['allowed']

