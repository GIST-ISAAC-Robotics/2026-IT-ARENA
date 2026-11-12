import inspect, math
from pathlib import Path
import numpy as np
import pytest
import candidate
import arena_autonomy.local_path as lp

# 의미 비교 기준: 동일 기존 판정/탐색 순서를 데이터 길이만큼 반복한다.
namespace={}
exec(inspect.getsource(candidate.ORIGINAL).replace('range(12)','range(len(points))'),lp.__dict__,namespace)
reference=namespace['break_adjacent_occluders']

def broken(p):
    return np.linalg.norm(np.diff(p,axis=0),axis=1)>.10+.035*np.minimum(np.linalg.norm(p[:-1],axis=1),np.linalg.norm(p[1:],axis=1))

@pytest.mark.parametrize('seed',range(60))
def test_same_as_complete_sequential_search(seed):
    rng=np.random.default_rng(seed)
    angles=np.sort(rng.uniform(-math.pi,math.pi,512))
    ranges=rng.choice([.3,.5,1.],512)+rng.normal(0,.003,512)
    points=ranges[:,None]*np.column_stack([np.cos(angles),np.sin(angles)])
    before=points.copy();edges=broken(points)
    a,aj=candidate.break_adjacent_occluders(points,edges)
    b,bj=reference(points,edges)
    np.testing.assert_array_equal(a,b);assert aj==bj
    np.testing.assert_array_equal(before,points)

def test_r308_stall_still_uses_real_wall_and_full_raw_points():
    data=np.load(Path(__file__).with_name('stall_sensor.npz'))
    points=data['points'];order=np.argsort(np.arctan2(points[:,1],points[:,0]));p=points[order]
    old,oj=candidate.ORIGINAL(p,broken(p));new,nj=candidate.break_adjacent_occluders(p,broken(p))
    ref,rj=reference(p,broken(p))
    assert len(oj)==12 and len(nj)>12
    np.testing.assert_array_equal(new,ref);assert nj==rj
    before=points.copy()
    candidate.install()
    try:
        path,info=lp.local_path(points,'right',float(data['offset']),horizon=float(data['horizon']),
            rays=data['rays'],piecewise_corner=True,minimum_radius=float(data['radius']),narrow_occluder_bridge=True)
        assert path is not None and info['path_anchor_error_m']<.02
    finally:candidate.uninstall()
    np.testing.assert_array_equal(before,points)

@pytest.mark.parametrize('seed',range(12))
def test_less_than_cap_retains_existing_semantics(seed):
    rng=np.random.default_rng(seed)
    points=np.column_stack([np.linspace(-.5,2.,100),np.full(100,-.5)])
    points[45:48,1]+=.16+rng.uniform(-.01,.01,3)
    a,aj=candidate.break_adjacent_occluders(points,broken(points));b,bj=candidate.ORIGINAL(points,broken(points))
    np.testing.assert_array_equal(a,b);assert aj==bj

def test_wide_foreground_and_missing_end_witness_remain():
    for p in (np.array([[-.2,-.5],[-.1,-.3],[.1,-.3],[.2,-.5]]),np.array([[-.1,-.3],[0.,-.3],[.1,-.5]])):
        mask,joins=candidate.break_adjacent_occluders(p,broken(p))
        assert not mask.any() and not joins
