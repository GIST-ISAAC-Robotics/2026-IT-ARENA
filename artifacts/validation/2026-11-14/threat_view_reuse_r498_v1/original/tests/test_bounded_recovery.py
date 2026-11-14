import math
from dataclasses import replace

import numpy as np
import pytest

from arena_autonomy.bounded_recovery import PulseBudget, quiet_feedback, recovery_evidence, path_prefix_supported
from arena_autonomy.lidar_motion import MotionHistory
from arena_autonomy.local_follow import FollowConfig
from test_corridor_visibility import scene
from test_linear_observation import unknown_at


def stationary(now=.4):
    h=MotionHistory()
    for t in np.arange(0.,now+.001,.01):
        h.wheels.add(float(t),0.)
        h.add_gyro(float(t),0.)
    return h


def straight():
    return np.column_stack((np.arange(0.,.61,.02),np.zeros(31)))


def test_path_prefix_does_not_require_straight_driving_through_standstill_gap():
    p=np.column_stack((np.arange(0.,.31,.02),2*np.arange(0.,.31,.02)**2))
    assert path_prefix_supported(p,.16)
    assert not path_prefix_supported(p,.27)


@pytest.mark.parametrize('p',[np.array([[0.,0.],[.4,0.]]),straight()[::-1],straight()+[.2,0.],straight()+[0.,.081]])
def test_path_gaps_short_unordered_offset_rejected(p):
    assert not path_prefix_supported(p,.16)


def test_stationary_front_observed_side_unknown_produces_evidence_only():
    r=recovery_evidence(unknown_at(90),stationary(),0.,0.,straight(),FollowConfig())
    assert r['eligible']
    assert r['observation']['required_distance_m'] > r['path_prefix_distance_m']
    assert not r['actual_body_stop_verified']


@pytest.mark.parametrize('speed,steer',[(-.001,0.),(.06,0.),(0.,.002),(float('nan'),0.)])
def test_start_needs_current_zeroish_positive_feedback(speed,steer):
    assert not recovery_evidence(scene(),stationary(),speed,steer,straight(),FollowConfig())['eligible']


def test_recent_movement_missing_gyro_and_stale_feedback_rejected():
    h=stationary()
    assert quiet_feedback(h,.4)
    assert not quiet_feedback(h,.45)
    h.wheels.values[-10]=.1
    assert not quiet_feedback(h,.4)
    h=stationary()
    h.gyro.times.clear(); h.gyro.values.clear()
    assert not quiet_feedback(h,.4)


def test_front_unknown_and_close_return_rejected():
    for s in (unknown_at(0),scene(.25)):
        assert not recovery_evidence(s,stationary(),0.,0.,straight(),FollowConfig())['eligible']


def test_pulse_budget_once_only_even_after_fresh_permission():
    b=PulseBudget()
    assert b.step(1.,0.,True)
    for t in (1.05,1.1,1.15,1.2): assert b.step(t,.1,True)
    assert not b.step(1.25,.1,True)
    assert not b.step(2.,0.,True)
    assert not b.step(2.02,0.,False)


def test_distance_budget_and_loss_of_evidence_end_pulse():
    b=PulseBudget()
    assert b.step(1.,.25,True)
    assert b.step(1.09,.25,True)
    assert b.step(1.18,.25,True)
    assert not b.step(1.20,.25,True)
    b=PulseBudget()
    assert b.step(1.,0.,True)
    assert not b.step(1.02,0.,False)
    assert not b.step(1.04,0.,True)


@pytest.mark.parametrize('now,speed',[(.9,0.),(1.2,0.),(1.02,-.01),(1.02,.26),(math.nan,0.)])
def test_time_reverse_gap_negative_overspeed_fail_closed(now,speed):
    b=PulseBudget(); assert b.step(1.,0.,True)
    assert not b.step(now,speed,True)
