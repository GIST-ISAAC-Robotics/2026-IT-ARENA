"""Synthetic geometry contracts only, not sampled sensor or physical driving proof."""
import numpy as np
from arena_autonomy.fragmented_wall_hypothesis import propose


def cloud(*,shift=0.,witness=True,anchor=True,exit_length=1.5):
    a=np.column_stack((np.linspace(-.3,.3,45),np.full(45,-.505)))
    b=np.column_stack((np.linspace(1.,1.6,30),np.full(30,-.505+shift)))
    c=np.column_stack((np.full(40,1.6),np.linspace(-.505+shift,-.505+shift+exit_length,40)))
    occ=np.column_stack((np.linspace(.4,.6,20),np.full(20,-.25)))
    return np.vstack(([a] if anchor else [])+[b,c]+([occ] if witness else []))


def test_collinear_fragments_require_actual_nearer_returns():
    p=cloud();before=p.copy()
    path,d=propose(p,'right',.505,.348)
    assert path is not None and d['accepted'] and d['gap_m']>.1
    assert d['free_space_certified'] is False and d['added_sensor_points']==0
    assert len(d['occluder_indices'])>=3
    np.testing.assert_array_equal(p,before)
    assert propose(cloud(witness=False),'right',.505,.348)[0] is None


def test_other_line_missing_anchor_and_short_exit_are_rejected():
    for p in (cloud(shift=.16),cloud(anchor=False),cloud(exit_length=.10)):
        assert propose(p,'right',.505,.348)[0] is None


def test_mirror_left_is_equivalent():
    right,_=propose(cloud(),'right',.505,.348)
    pts=cloud()*[1,-1]
    left,_=propose(pts,'left',.505,.348)
    assert left is not None
    np.testing.assert_allclose(left,right*[1,-1],atol=1e-10)


def test_invalid_and_below_steering_radius_are_rejected():
    assert propose(cloud(),'right',.30,.348)[0] is None
    p=cloud();p[0,0]=float('nan')
    assert propose(p,'right',.505,.348)[0] is None
    assert propose(cloud(),'unknown',.505,.348)[0] is None


def test_nearby_car_tail_may_share_the_wall_component_but_is_not_erased():
    p=cloud()
    tail=np.column_stack((np.linspace(.28,.35,8),np.linspace(-.48,-.40,8)))
    p=np.vstack((p,tail));before=p.copy()
    path,d=propose(p,'right',.505,.348)
    assert path is not None and d['free_space_certified'] is False
    np.testing.assert_array_equal(p,before)


def test_wrong_offset_continuation_does_not_become_the_near_line():
    p=cloud(shift=.12)
    assert propose(p,'right',.505,.348)[0] is None
