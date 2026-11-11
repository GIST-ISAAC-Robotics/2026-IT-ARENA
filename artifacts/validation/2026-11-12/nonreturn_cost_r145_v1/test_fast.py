import importlib.util
import math
from pathlib import Path
import numpy as np
import pytest

import arena_autonomy.rear_fringe_trial as old
from test_rear_fringe_trial import case
import fast_nonreturn as fast


@pytest.mark.parametrize('rays,queries',[(500,1),(504,31),(508,32),(512,33),(8192,67)])
def test_exact_brackets_and_radii_across_chunks(rays,queries):
    rng=np.random.default_rng(145+rays+queries)
    origins=rng.normal(0,.015,(rays,2));a=rng.uniform(-math.pi,math.pi,rays)
    directions=np.column_stack((np.cos(a),np.sin(a)))
    q=rng.normal(0,.3,(queries,2))
    actual=list(fast._query_brackets(q,origins,directions))
    assert len(actual)==len(q)
    for point,(radius,angles,lo,hi) in zip(q,actual):
        delta=point-origins
        expected_radius=np.linalg.norm(delta,axis=1)
        expected_angles=np.arctan2(directions[:,0]*delta[:,1]-directions[:,1]*delta[:,0],
                                  np.sum(directions*delta,axis=1))
        assert np.array_equal(radius,expected_radius)
        assert np.array_equal(angles,expected_angles)
        assert lo==int(np.argmin(np.where(expected_angles>=-1e-10,np.maximum(expected_angles,0.),np.inf)))
        assert hi==int(np.argmin(np.where(expected_angles<=1e-10,np.maximum(-expected_angles,0.),np.inf)))


@pytest.mark.parametrize('a',[-1.0000001e-10,-1e-10,-.9999999e-10,-0.,0.,.9999999e-10,1e-10,1.0000001e-10])
def test_threshold_and_tie_order(a):
    directions=np.array([[math.cos(a),math.sin(a)],[math.cos(a),math.sin(a)],[1.,0.]])
    origins=np.zeros((3,2));q=np.tile([1.,0.],(33,1))
    rows=list(fast._query_brackets(q,origins,directions))
    delta=q[0]-origins
    expected=np.arctan2(directions[:,0]*delta[:,1]-directions[:,1]*delta[:,0],np.sum(directions*delta,axis=1))
    for _,angles,lo,hi in rows:
        assert np.array_equal(angles,expected)
        assert lo==np.argmin(np.where(expected>=-1e-10,np.maximum(expected,0.),np.inf))
        assert hi==np.argmin(np.where(expected<=1e-10,np.maximum(-expected,0.),np.inf))


@pytest.mark.parametrize('value',[math.inf,math.nan,-math.inf,.02])
@pytest.mark.parametrize('extra',[(-.047,.0753),(.8,.2),(.2,0.)])
def test_failure_order_and_group_order(value,extra):
    result,scan,scene=case(value=value)
    baseline=dict(accepted=False,reason='invalid_input',unknown_filled=False,
                  maximum_gap_rad=math.radians(3),minimum_neighbor_range_m=.6)
    for q in (np.array([[-.047,.0753],extra]),np.array([extra,[-.047,.0753]])):
        pad=np.full(500,.1)
        a=old._bounded_nonreturn(scene,scan,q,pad,baseline)
        b=fast._bounded_nonreturn(scene,scan,q,pad,baseline)
        assert a==b


def test_sensor_origin_still_rejected():
    _,scan,scene=case()
    baseline=dict(accepted=False,unknown_filled=False,maximum_gap_rad=math.radians(3),minimum_neighbor_range_m=.6)
    q=np.array([scene.rays[0].ray_origin_xy_m])
    assert old._bounded_nonreturn(scene,scan,q,np.zeros(500),baseline)==fast._bounded_nonreturn(scene,scan,q,np.zeros(500),baseline)
