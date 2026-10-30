from pathlib import Path
import sys
import numpy as np
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from geometry import compare
from arena_autonomy.local_follow import FollowConfig

C=FollowConfig()


@pytest.mark.parametrize('side',[-1,1])
def test_parallel_uncertainty_overlap_stays_explicit(side):
    r=compare([[0.,side*.2]],[.13],0.,.035,C)
    assert r['sampled_geometry_candidate'] and r['side_only_overlap_points']==1
    assert r['current_uncertainty_overlap_points']==1 and not r['unknown_filled']
    assert r['minimum_base_gap_m']>0 and not r['control_authorized']


@pytest.mark.parametrize('side',[-1,1])
def test_turn_toward_side_cannot_reuse_parallel_permission(side):
    r=compare([[0.,side*.2]],[.13],side*.2,.08,C)
    assert not r['sampled_geometry_candidate'] and r['worsening_side_points']


@pytest.mark.parametrize('point,padding',[
    ([.2,0.],.13),([0.,.12],.13),([.25,0.],.1),([.25,.3],.13)])
def test_front_base_new_overlap_or_turns_are_not_whitelisted(point,padding):
    # 마지막은 회전으로 새로운 측방 위험을 만드는 예다. 모든 물체 점을 유지한다.
    steer=.37 if point[1]==.3 else 0.
    r=compare([point],[padding],steer,.25,C)
    assert not r['sampled_geometry_candidate']


def test_reflection_preserves_all_counts_and_gap():
    pts=np.array([[0.,.2],[.5,-.4],[1.,.2]])
    a=compare(pts,[.13,.055,.1],-.08,.07,C)
    b=compare(pts*np.array([1.,-1.]),[.13,.055,.1],.08,.07,C)
    assert a==b


@pytest.mark.parametrize('points,padding,steer,distance',[
    ([],[],0.,.1),([[float('nan'),0.]],[.1],0.,.1),([[0.,.2]],[.01],0.,.1),
    ([[0.,.2]],[.1],float('nan'),.1),([[0.,.2]],[.1],0.,-1.),([[0.,.2]],[.1],.9,.1)])
def test_invalid_input_is_not_permission(points,padding,steer,distance):
    with pytest.raises(ValueError):compare(points,padding,steer,distance,C)
