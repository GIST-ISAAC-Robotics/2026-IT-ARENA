import numpy as np
import pytest
from continuous_prefix import capsule_intervals,common_intervals,refine


def capsule(a,b,c,d,r):
    return capsule_intervals(*[np.array(v,dtype=float) for v in (a,b,c,d)],r)


def test_perpendicular_band():
    assert np.asarray(capsule((0,-1),(0,1),(-1,0),(1,0),.2))==pytest.approx(np.array([[.4,.6]]))


def test_inside_and_outside_parallel_segments():
    assert capsule((0,.05),(1,.05),(0,0),(1,0),.08)==[(0.,1.)]
    assert capsule((0,.081),(1,.081),(0,0),(1,0),.08)==[]


def test_endpoint_circle_not_infinite_line():
    result=capsule((-1,0),(2,0),(0,0),(1,0),.1)
    assert np.asarray(result)==pytest.approx(np.array([[.3,.7]]))


@pytest.mark.parametrize('count',[2,36,101])
def test_representation_density_preserves_continuous_distance(count):
    own=np.column_stack([np.linspace(0,1,count),np.zeros(count)])
    other=np.array([[.25,.06],[1,.06]])
    intervals,arc=common_intervals(own,other,.1)
    assert np.asarray(intervals)==pytest.approx(np.array([[.17,1.]]))
    assert arc[-1]==pytest.approx(1.)


def test_disconnected_support_islands_not_joined():
    own=np.array([[0,0],[1,0]])
    other=np.array([[0,0],[.2,0],[.3,1],[.7,1],[.8,0],[1,0]])
    intervals,_=common_intervals(own,other,.01)
    assert len(intervals)==2
    assert intervals[0][1]<.22 and intervals[1][0]>.78


@pytest.mark.parametrize('result',[{'supported':True,'reason':'short_observed_prefix_geometry_only'},
                                  {'supported':False,'reason':'missing_path'},
                                  {'supported':False,'reason':'preferred_anchor_unconfirmed'}])
def test_other_successes_and_rejections_are_not_changed(result):
    assert refine(result,None) is result
