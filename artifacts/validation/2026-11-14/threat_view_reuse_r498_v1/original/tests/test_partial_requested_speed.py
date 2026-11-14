"""요청 감속으로 부분 관측의 허용 속도를 찾되 측정속도/장애물을 숨기지 않는다."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.scan_input import scan_angles
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import history
from test_partial_observation import sample
from test_native_scan_pipeline import native


def partial_scan(first, representation, gap=True, center=81.):
    if representation=='uniform':
        raw=sample(first)
        ids=np.flatnonzero(abs(scan_angles(raw)-math.radians(center))<math.radians(2.6))
        if gap:
            for index in ids:raw.ranges[index]=math.inf
        return raw
    raw=native(first=first,clockwise=representation=='cw')
    values=list(raw.ranges)
    if gap:
        for index in np.flatnonzero(abs(scan_angles(raw)-math.radians(center))<math.radians(2.6)):
            values[index]=math.inf
    return replace(raw,ranges_m=tuple(values))


@pytest.mark.parametrize('representation',['uniform','cw','ccw'])
def test_proposal_is_stateless_and_changes_only_requested_speed(representation):
    p=PartialObservationPolicy(True,True,True,True)
    raw=partial_scan(.5,representation);before=tuple(raw.ranges)
    assert p.requested_speed(raw,0.,.99)==.2
    assert p.requested_speed(raw,0.,0.)==0.
    assert p.requested_speed(raw,0.,.1)==.1
    assert tuple(raw.ranges)==before and p.previous is None and not p.latched
    assert p.requested_speed(partial_scan(.5,representation,False),0.,.99)==.99
    assert PartialObservationPolicy().requested_speed(raw,0.,.99)==.99
    ids=frozenset(np.flatnonzero(~np.isfinite(raw.ranges)).tolist())
    assert p.requested_speed(raw,0.,.99,ids)==.99


@pytest.mark.parametrize('representation',['uniform','cw','ccw'])
@pytest.mark.parametrize('measured',[0.,.25,.61])
def test_command_evaluates_actual_speed_even_when_request_is_capped(representation,measured):
    f=LocalFollow(FollowConfig(solo_route_only=True,partial_observation_policy='bounded_local_v4',oblique_gap_trial=True))
    h=history(v=measured)
    f._path_command=lambda *args,**kw:(.99,.1,{'reason':'local_follow'})
    warm=partial_scan(.4,representation,False)
    scene=motion_scene(warm,h,.52,timing_verified=True,rear_half_angle_deg=0.)
    f._partial_command(scene,h,measured,'left',.1,.02,warm)
    raw=partial_scan(.5,representation)
    scene=motion_scene(raw,h,.62,timing_verified=True,rear_half_angle_deg=0.)
    output=f._partial_command(scene,h,measured,'left',.1,.02,raw)
    assert output[2]['partial_requested_speed_mps']==.2
    if measured>.6:
        assert output[0]==0. and not output[2]['partial_observation']['allowed']
    else:
        assert output[0]==.2 and output[2]['partial_observation']['allowed']
        assert not output[2]['partial_observation']['unknown_filled']


@pytest.mark.parametrize('fault',['central','near','missing_all','lost_obstacle'])
def test_slow_request_is_not_itself_permission(fault):
    p=PartialObservationPolicy(True,True,True,True);h=history()
    old=[[.06+.16*math.cos(math.radians(81)),.16*math.sin(math.radians(81))]] if fault=='lost_obstacle' else []
    p.evaluate(sample(.4),old,h,.5,.2)
    raw=partial_scan(.5,'uniform',center=0. if fault=='central' else 81.)
    if fault=='central':
        for index in range(246,255):raw.ranges[index]=math.inf
    if fault=='missing_all':raw.ranges=[math.inf]*500
    if fault=='near':
        missing=np.flatnonzero(~np.isfinite(raw.ranges));raw.ranges[missing[0]-1]=.15
    requested=p.requested_speed(raw,0.,.99)
    assert requested==.2
    result=p.evaluate(raw,[],h,.6,requested)
    assert not result['allowed']
