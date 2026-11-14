"""잡음 지주 분리 후보: 실제 근거리 반환은 벽 적합 밖에서도 보존돼야 한다."""
import numpy as np
import pytest
from arena_autonomy.local_path import break_adjacent_occluders, narrow_occluders


def breaks(points):
    r=np.linalg.norm(points,axis=1)
    return np.linalg.norm(np.diff(points,axis=0),axis=1) > .10+.035*np.minimum(r[:-1],r[1:])


@pytest.mark.parametrize('mirror',[1,-1])
@pytest.mark.parametrize('reverse',[False,True])
def test_one_attached_side_keeps_actual_neighbors_and_raw_points(mirror,reverse):
    p=np.array([[.40,.48],[.45,.47],[.45,.37],[.47,.53],[.53,.52]])
    p[:,1]*=mirror
    if reverse:p=p[::-1].copy()
    before=p.copy();b=breaks(p)
    assert sum(b)==1
    old,_=narrow_occluders(p,b)
    mask,joins=break_adjacent_occluders(p,b)
    assert not old.any()
    assert mask.tolist()==[False,False,True,False,False]
    assert joins==[(1,3)]
    np.testing.assert_array_equal(p,before)


def test_already_joined_ray_continuity_is_not_reopened():
    p=np.array([[.40,.48],[.45,.47],[.45,.37],[.47,.53],[.53,.52]])
    mask,joins=break_adjacent_occluders(p,np.zeros(len(p)-1,dtype=bool))
    assert not mask.any() and not joins


@pytest.mark.parametrize('mirror',[1,-1])
def test_wide_vehicle_front_does_not_become_a_narrow_bridge(mirror):
    # 15 cm 면 양끝까지 관측된 차를 작은 조각으로 반복 제거하지 않는다.
    cluster=np.c_[np.linspace(.375,.525,12),np.full(12,.37)]
    p=np.vstack(([.33,.53],[.36,.51],cluster,[.55,.52],[.58,.53]))
    p[:,1]*=mirror
    mask,joins=break_adjacent_occluders(p,breaks(p))
    assert not mask.any() and not joins


def test_range_noise_alone_on_a_wall_is_not_a_radial_occluder():
    theta=np.linspace(.6,1.2,80)
    radius=.5/np.sin(theta)
    radius+=.03*np.where(np.arange(len(theta))%2,1,-1)
    p=radius[:,None]*np.c_[np.cos(theta),np.sin(theta)]
    mask,joins=break_adjacent_occluders(p,breaks(p))
    assert not mask.any() and not joins


@pytest.mark.parametrize('p',[
    [[.45,.37],[.47,.53],[.53,.52]],
    [[.4,.48],[.45,.47],[.45,.37]],
    [[-.2,.5],[0.,.35],[.2,.5]],
])
def test_missing_neighbor_or_large_shadow_does_not_extend(p):
    p=np.array(p)
    mask,joins=break_adjacent_occluders(p,breaks(p))
    assert not mask.any() and not joins


def test_two_separate_objects_remain_two_bridges():
    p=np.array([[.4,.5],[.45,.5],[.45,.36],[.47,.52],
                [.55,.52],[.60,.53],[.60,.38],[.62,.55],[.67,.55]])
    mask,joins=break_adjacent_occluders(p,breaks(p))
    assert np.flatnonzero(mask).tolist()==[2,6]
    assert joins==[(1,3),(5,7)]
