"""단일 무반환을 저속 이동 범위로 검사한다. 자유 공간 인증은 아니다."""
import math
from dataclasses import replace
import json
import numpy as np
import pytest
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.lidar_safety import LidarSafety
from arena_autonomy.scan_input import scan_angles,project_scan
from test_local_follow import history
from test_partial_observation import sample,candidate_node
from test_native_scan_pipeline import native


def scan_with_gap(first,representation,angle=0.,width=5.):
    s=sample(first) if representation=='uniform' else native(first=first,clockwise=representation=='cw')
    values=list(s.ranges)
    for i in np.flatnonzero(abs(scan_angles(s)-math.radians(angle))<math.radians(width/2)):
        values[i]=math.inf
    if representation=='uniform':s.ranges=values;return s
    return replace(s,ranges_m=tuple(values))


@pytest.mark.parametrize('representation',['uniform','cw','ccw'])
@pytest.mark.parametrize('angle',[0.,29.,31.])
def test_same_bounded_wedge_either_side_of_old_angle_cutoff(representation,angle):
    p=PartialObservationPolicy(True,True,True,True);h=history()
    warm=scan_with_gap(.4,representation,width=0.)
    pts,_,_=project_scan(warm,h,.52,timing_verified=True)
    assert p.evaluate(warm,pts,h,.52,.2)['allowed']
    raw=scan_with_gap(.5,representation,angle=angle)
    original=tuple(raw.ranges);pts,_,_=project_scan(raw,h,.62,timing_verified=True)
    requested=p.requested_speed(raw,0.,1.)
    assert requested==.2
    r=p.evaluate(raw,pts,h,.62,requested)
    assert r['allowed'] and r['speed_cap_mps']==.2
    assert r['lateral_metric']['missing_chord_m']<=.05
    assert not r['unknown_filled'] and tuple(raw.ranges)==original


def test_medium_front_gap_uses_same_bound_without_width_discontinuity():
    p=PartialObservationPolicy(True,True,True,True);h=history()
    p.evaluate(sample(.4),[],h,.5,.2)
    raw=scan_with_gap(.5,'uniform',width=2.3)
    assert p.requested_speed(raw,0.,1.)==.2
    r=p.evaluate(raw,[],h,.6,.2)
    assert r['allowed'] and r['speed_cap_mps']==.2


@pytest.mark.parametrize('fault',['wide','multiple','near','lost','fast','stale'])
def test_known_threats_and_missing_support_are_not_excused(fault):
    p=PartialObservationPolicy(True,True,True,True);h=history()
    p.evaluate(sample(.4),[[.15,0.]] if fault=='lost' else [],h,.5,.2)
    raw=scan_with_gap(.5,'uniform',width=8. if fault=='wide' else 5.)
    if fault=='multiple':raw.ranges[300]=math.inf
    if fault=='near':raw.ranges[np.flatnonzero(~np.isfinite(raw.ranges))[0]-1]=.5
    assert not p.evaluate(raw,[],h,.9 if fault=='stale' else .6,.61 if fault=='fast' else .2)['allowed']


def test_original_non_oblique_policy_still_rejects_forward_wedge():
    p=PartialObservationPolicy(True,True,True);h=history()
    p.evaluate(sample(.4),[],h,.5,.2)
    assert not p.evaluate(scan_with_gap(.5,'uniform'),[],h,.6,.2)['allowed']


def test_independent_gate_still_brakes_for_current_front_return():
    node,commands,statuses=candidate_node(forward=True)
    node.partial_policy=PartialObservationPolicy(True,True,True,True)
    node.partial_policy.evaluate(sample(.4),[],node.motion.history,.5,.2)
    node.scan=scan_with_gap(.5,'uniform',angle=29.)
    node.command.drive.speed=.2
    node.motion.project=lambda _: (np.tile([.1,0.],(60,1)),60)
    LidarSafety.control(node)
    assert commands[-1].drive.speed==0.
    assert json.loads(statuses[-1].data)['reason']!='clear'
