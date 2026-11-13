"""위협 기록의 계산 범위 최적화가 거리/광선 판정을 바꾸지 않는지 검사한다."""
from dataclasses import replace
import numpy as np
import pytest
import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import car
from test_m3_observed_follow import timed_scene


def brute_within(cells, centers, radius):
    if not len(centers):
        return np.zeros(len(cells),dtype=bool)
    d = cells[:,None,:]-centers[None,:,:]
    return np.min(d[:,:,0]**2+d[:,:,1]**2,axis=1) <= radius*radius


@pytest.mark.parametrize('radius',[0.,.02,.0782842712474619,.25,2.])
def test_windowed_disks_equal_full_distance_on_random_and_boundary_points(radius):
    rng=np.random.default_rng(725)
    centers=rng.uniform(-3.,3.,(271,2))
    cells=rng.uniform(-4.,4.,(319,2))
    # 原 판정이 포함/제외하는 부동소수점 경계를 그대로 유지한다.
    edge=centers[:19]+[radius,0.]
    cells=np.concatenate((cells,centers[:5],edge,
        np.nextafter(edge,np.inf),np.nextafter(edge,-np.inf)))
    assert np.array_equal(mx._within(cells,centers,radius),brute_within(cells,centers,radius))
    assert np.array_equal(mx._within(cells,centers[::-1],radius),brute_within(cells,centers,radius))


def test_disk_empty_inputs_and_distant_singleton():
    empty=np.empty((0,2))
    assert not len(mx._within(empty,np.array([[0.,0.]]),.1))
    assert not mx._within(np.array([[0.,0.]]),empty,.1).any()
    assert not mx._within(np.array([[5.,5.]]),np.array([[-5.,-5.]]),.1).any()


def scalar_returned(scene,points):
    origins=np.array([r.ray_origin_xy_m for r in scene.rays])
    directions=np.array([r.direction_xy for r in scene.rays])
    status=np.array([r.status for r in scene.rays])
    indices=np.array([r.source_index for r in scene.rays])
    out=[]
    for point in points:
        d=point-origins
        angle=np.arctan2(directions[:,0]*d[:,1]-directions[:,1]*d[:,0],
                        directions[:,0]*d[:,0]+directions[:,1]*d[:,1])
        low=np.where(angle>=-1e-10,np.maximum(angle,0.),np.inf)
        high=np.where(angle<=1e-10,np.maximum(-angle,0.),np.inf)
        lo,hi=int(np.argmin(low)),int(np.argmin(high))
        out.append(bool(np.isfinite(low[lo]) and np.isfinite(high[hi]) and low[lo]+high[hi]<=.025
            and abs(int(indices[lo])-int(indices[hi]))<=1 and status[lo]==status[hi]=='return'))
    return np.array(out,dtype=bool)


@pytest.mark.parametrize('moving_origins',[False,True])
def test_batched_return_brackets_preserve_unknown_masked_ties_and_source_order(moving_origins):
    scene,_=timed_scene(1.,-.09,extra=[car(.3,.13,-.09)],
        dropout=lambda deg:(deg>20)&(deg<26))
    if moving_origins:
        scene=replace(scene,rays=tuple(replace(r,ray_origin_xy_m=(.06+.02*k/500,.001*np.sin(k)))
                                      for k,r in enumerate(scene.rays)))
    rng=np.random.default_rng(912)
    points=rng.uniform(-1.,1.,(391,2))
    origins=np.array([r.ray_origin_xy_m for r in scene.rays])
    directions=np.array([r.direction_xy for r in scene.rays])
    points=np.concatenate((points,origins,origins+.2*directions,origins-.2*directions))
    assert np.array_equal(mx._both_sides_returned(scene,points),scalar_returned(scene,points))
    assert not len(mx._both_sides_returned(scene,np.empty((0,2))))
