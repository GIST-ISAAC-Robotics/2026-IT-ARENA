import numpy as np
import pytest
from candidate import propose,straight_path
from test_fragmented_wall_hypothesis import cloud

def straight(witness=True):
    a=np.column_stack((np.linspace(-.3,.3,45),np.full(45,-.505)))
    b=np.column_stack((np.linspace(.8,1.8,60),np.full(60,-.505)))
    occ=np.column_stack((np.linspace(.4,.6,20),np.full(20,-.25)))
    return np.vstack([a,b,*([occ] if witness else [])])

def test_actual_straight_fragments_and_witness_only():
    points=straight();before=points.copy();path,d=propose(points,'right',.505,.348)
    assert path is not None and d['shape']=='straight'
    assert d['added_sensor_points']==0 and not d['free_space_certified']
    assert np.max(abs(path[:,1]))<1e-12 and path[:,0].max()<=1.8+1e-12
    np.testing.assert_array_equal(before,points)
    assert propose(straight(False),'right',.505,.348)[0] is None

def test_left_mirror():
    right,_=propose(straight(),'right',.505,.348)
    left,_=propose(straight()*[1,-1],'left',.505,.348)
    np.testing.assert_allclose(left,right*[1,-1],atol=1e-10)

def test_bent_or_missing_near_wall_not_flattened():
    assert straight_path(cloud(),'right',.505) is None
    assert propose(straight()[45:],'right',.505,.348)[0] is None
    p=straight();p[45:105,1]+=.12
    assert propose(p,'right',.505,.348)[0] is None

def test_original_corner_and_short_bad_exit():
    p,d=propose(cloud(),'right',.505,.348)
    assert p is not None and d['shape']=='corner'
    assert propose(cloud(exit_length=.10),'right',.505,.348)[0] is None
