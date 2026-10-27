"""v4 저속 측방 공백 후보. 기존 v3·정면·고장·소실 장애물을 완화하지 않는다."""
import math
import numpy as np
import pytest
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_partial_observation import sample
from test_local_follow import history


def ready():
    p=PartialObservationPolicy(True,True,True);h=history()
    p.evaluate(sample(.4),[],h,.5,.5)
    return p,h


@pytest.mark.parametrize('indices',[range(361,366),range(135,140)])
def test_low_speed_lateral_chord_candidate_preserves_unknown_and_caps(indices):
    p,h=ready();raw=sample(.5,indices);before=raw.ranges.copy()
    result=p.evaluate(raw,[],h,.6,.5)
    assert result['allowed'] and result['reason']=='bounded_lateral_metric_missing'
    assert result['speed_cap_mps']==.2 and not result['unknown_filled']
    assert result['lateral_metric']['missing_chord_m']<.05
    np.testing.assert_array_equal(raw.ranges,before)
    old=PartialObservationPolicy(True,True);old.evaluate(sample(.4),[],h,.5,.5)
    assert old.evaluate(raw,[],h,.6,.5)['reason']=='wide_or_many_missing'


@pytest.mark.parametrize('fault',['speed','wide','forward','near_neighbor','missing_all','stale','rotated_forward'])
def test_candidate_does_not_authorize_outside_its_observation_contract(fault):
    p,h=ready();raw=sample(.5,range(361,366));speed=.5;now=.6
    if fault=='speed':speed=.61
    if fault=='wide':raw=sample(.5,range(360,368))
    if fault=='forward':raw=sample(.5,range(249,254))
    if fault=='near_neighbor':raw.ranges[360]=.9
    if fault=='missing_all':raw=sample(.5,range(500))
    if fault=='stale':now=1.
    if fault=='rotated_forward':h=history(w=4.);now=.7
    assert not p.evaluate(raw,[],h,now,speed)['allowed']


def test_widened_lateral_candidate_still_latches_disappearing_close_object():
    p,h=ready()
    angle=math.radians(81.)
    p._remember([[.06+.16*math.cos(angle),.16*math.sin(angle)]],.5,.4,True)
    result=p.evaluate(sample(.5,range(361,366)),[],h,.6,.5)
    assert not result['allowed'] and result['reason']=='known_obstacle_latched'
    assert result['latch_evidence']['pairs']>0
