"""취득 순서/동률/부동소수 경계가 묶음 연산으로 달라지지 않는지 확인한다."""
import math
import numpy as np
import pytest

from arena_autonomy.rear_fringe_trial import _query_brackets


def scalar_bracket(point, origins, directions):
    delta=point-origins
    radius=np.linalg.norm(delta,axis=1)
    angles=np.arctan2(directions[:,0]*delta[:,1]-directions[:,1]*delta[:,0],
                     np.sum(directions*delta,axis=1))
    lo=int(np.argmin(np.where(angles>=-1e-10,np.maximum(angles,0.),np.inf)))
    hi=int(np.argmin(np.where(angles<=1e-10,np.maximum(-angles,0.),np.inf)))
    return radius,angles,lo,hi


@pytest.mark.parametrize('rays,queries',[(500,1),(504,31),(508,32),(512,33),(8192,67)])
def test_native_counts_and_chunk_boundaries(rays,queries):
    rng=np.random.default_rng(145+rays+queries)
    origins=rng.normal(0,.015,(rays,2));a=rng.uniform(-math.pi,math.pi,rays)
    directions=np.column_stack((np.cos(a),np.sin(a)))
    q=rng.normal(0,.3,(queries,2))
    actual=list(_query_brackets(q,origins,directions))
    assert len(actual)==len(q)
    for point,got in zip(q,actual):
        expected=scalar_bracket(point,origins,directions)
        assert np.array_equal(got[0],expected[0])
        assert np.array_equal(got[1],expected[1])
        assert got[2:]==expected[2:]


@pytest.mark.parametrize('a',[-1.0000001e-10,-1e-10,-.9999999e-10,-0.,0.,.9999999e-10,1e-10,1.0000001e-10])
def test_threshold_and_tie_order(a):
    directions=np.array([[math.cos(a),math.sin(a)],[math.cos(a),math.sin(a)],[1.,0.]])
    origins=np.zeros((3,2));q=np.tile([1.,0.],(33,1))
    expected=scalar_bracket(q[0],origins,directions)
    for got in _query_brackets(q,origins,directions):
        assert np.array_equal(got[0],expected[0])
        assert np.array_equal(got[1],expected[1])
        assert got[2:]==expected[2:]


def test_empty_queries_do_not_allocate_rays():
    assert list(_query_brackets(np.empty((0,2)),np.empty((0,2)),np.empty((0,2))))==[]
