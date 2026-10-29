"""기본 측방 증거와 같은 명령의 범위를 맞추되 정면/넓은 공백은 거절한다."""
import math
import numpy as np
import pytest
from arena_autonomy.same_path_gap_trial import bounded_same_path_gap,scope_check
from arena_autonomy.scene_motion import motion_scene
from test_same_path_gap_probe import fixture
from test_local_follow import history


def case(degrees,missing_count=1):
    raw,_,proof,command=fixture()
    raw.ranges[375]=8.
    i=round((math.radians(degrees)-raw.angle_min)/raw.angle_increment)
    for j in range(i,i+missing_count):raw.ranges[j]=math.inf
    scene=motion_scene(raw,history(),.6,timing_verified=True)
    angle=raw.angle_min+i*raw.angle_increment
    point=np.array([[.06+.08*math.cos(angle),.08*math.sin(angle)]])
    return raw,scene,proof,command,point


@pytest.mark.parametrize('degrees',[-90,-60,-54,-45,45,54,59.43,60,90])
def test_same_checked_path_accepts_oblique_side_gap(degrees):
    raw,scene,p,c,q=case(degrees)
    r=bounded_same_path_gap(scene,raw,q,np.full(500,.1),.2,.15,p,c)
    assert r['accepted'],r
    assert not r['unknown_filled'] and r['residual_hidden_object_risk']


@pytest.mark.parametrize('degrees',[-101,-30,0,30,101])
def test_forward_or_rear_gap_does_not_borrow_side_proof(degrees):
    raw,scene,p,c,q=case(degrees)
    r=bounded_same_path_gap(scene,raw,q,np.full(500,.1),.2,.15,p,c)
    assert not r['accepted'] and r['reason']=='unknown_outside_side_scope'


def test_wide_oblique_gap_is_still_rejected():
    raw,scene,p,c,q=case(54,7)
    assert not bounded_same_path_gap(scene,raw,q,np.full(500,.1),.2,.15,p,c)['accepted']


def test_oblique_gap_cannot_borrow_a_different_steering():
    raw,scene,p,c,q=case(54);c['steering']+=.01
    assert scope_check(scene,q,.2,.15,p,c)=='different_or_larger_motion'


def test_oblique_gap_near_occupied_return_is_not_ignored():
    raw,scene,p,c,q=case(54)
    i=round((math.radians(54)-raw.angle_min)/raw.angle_increment)
    raw.ranges[i-1]=.08
    scene=motion_scene(raw,history(),.6,timing_verified=True)
    assert bounded_same_path_gap(scene,raw,q,np.full(500,.1),.2,.15,p,c)['reason']=='known_occupied_padding'
