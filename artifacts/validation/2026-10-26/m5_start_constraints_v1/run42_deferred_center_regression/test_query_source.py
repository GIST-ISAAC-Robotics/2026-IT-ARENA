"""M3 외곽 질의 최적화와 10/26 run33 이전 전체 차체 사전 검사의 정확 대조."""
import math
import numpy as np
import pytest

from arena_autonomy.linear_observation import clip_halfplane
from arena_autonomy.lateral_intent import rectangle_corners, GEOMETRY_EPS_M
from arena_autonomy.observation_support import outside_body
from arena_autonomy.observed_lateral_trial import path_observation_queries
from arena_autonomy.m3_observed_follow import command_arc
from arena_autonomy.scene_motion import motion_scene
from test_lateral_intent import GRASS
from test_local_follow import history, scan


def reference(scene, poses, cfg):
    """개선 전 순서와 float 연산을 동결한다. 새 exterior 필요조건을 공유하지 않는다."""
    lx, wy = cfg.length_m/2, cfg.width_m/2
    border = np.unique(np.vstack(([(x,y) for x in np.linspace(-lx,lx,11) for y in (-wy,wy)],
        [(x,y) for x in (-lx,lx) for y in np.linspace(-wy,wy,9)])),axis=0)
    unknown = [r for r in scene.rays if r.status != 'return']
    co, si = math.cos(math.pi/500), math.sin(math.pi/500)
    queries, intersections = [], 0
    corners = rectangle_corners(poses,cfg.length_m,cfg.width_m)
    possible = np.zeros((len(poses),len(unknown)),dtype=bool)
    lower = [(co*r.direction_xy[0]+si*r.direction_xy[1],-si*r.direction_xy[0]+co*r.direction_xy[1]) for r in unknown]
    upper = [(co*r.direction_xy[0]-si*r.direction_xy[1],si*r.direction_xy[0]+co*r.direction_xy[1]) for r in unknown]
    if unknown:
        origins=np.array([r.ray_origin_xy_m for r in unknown]); directions=np.array([r.direction_xy for r in unknown])
        delta=corners[:,None,:,:]-origins[None,:,None,:];low,high=np.array(lower),np.array(upper)
        cl=low[None,:,None,0]*delta[...,1]-low[None,:,None,1]*delta[...,0]
        ch=high[None,:,None,0]*delta[...,1]-high[None,:,None,1]*delta[...,0]
        f=delta[...,0]*directions[None,:,None,0]+delta[...,1]*directions[None,:,None,1]
        possible=~(np.all(cl<0,axis=2)|np.all(ch>0,axis=2)|np.all(f<0,axis=2))
    for i,(pose,polygon) in enumerate(zip(poses,corners)):
        c,s=math.cos(pose[2]),math.sin(pose[2]);queries.extend(border@np.array([[c,s],[-s,c]])+pose[:2])
        for j in np.flatnonzero(possible[i]):
            origin=unknown[j].ray_origin_xy_m
            piece=clip_halfplane(clip_halfplane(polygon.tolist(),origin,lower[j],1),origin,upper[j],-1)
            for exterior in outside_body(piece,cfg.length_m,cfg.width_m):
                queries.extend(exterior);queries.append(np.mean(exterior,axis=0));intersections+=1
    q=np.asarray(queries,dtype=float).reshape(-1,2)
    outside=(abs(q[:,0])>lx+GEOMETRY_EPS_M)|(abs(q[:,1])>wy+GEOMETRY_EPS_M)
    return np.unique(q[outside],axis=0),intersections


@pytest.mark.parametrize('speed,yaw',[(0.,0.),(.35,.8),(.9,-1.3)])
@pytest.mark.parametrize('angle',[0.,-1e-9,1e-9,-.0003,.0003,-.175,.175,-.37,.37])
@pytest.mark.parametrize('distance',[0.,.08,.28,.65])
def test_query_coordinates_and_intersection_count_are_exact(speed,yaw,angle,distance):
    rng=np.random.default_rng(1026)
    raw=scan(.5,rng.uniform(.2,12.,500).tolist())
    for i in rng.choice(500,40,replace=False):raw.ranges[i]=math.inf
    scene=motion_scene(raw,history(speed,yaw),.635,timing_verified=True,rear_half_angle_deg=30.)
    poses=command_arc(angle,distance,GRASS)
    expected,n=reference(scene,poses,GRASS);actual,m=path_observation_queries(scene,poses,GRASS)
    assert m==n
    np.testing.assert_array_equal(actual,expected)


@pytest.mark.parametrize('missing',[False,True])
@pytest.mark.parametrize('offset',[0.,-1e-11,1e-11,-1e-7,1e-7,-.0001,.0001])
def test_all_missing_and_tiny_new_body_strips_are_preserved(missing,offset):
    raw=scan(.5,[math.inf if missing else 3.]*500)
    scene=motion_scene(raw,history(),.635,timing_verified=True,rear_half_angle_deg=0.)
    poses=np.array([[0.,offset,0.],[.02,offset,offset],[.2,-offset,-offset]])
    expected,n=reference(scene,poses,GRASS);actual,m=path_observation_queries(scene,poses,GRASS)
    assert m==n
    np.testing.assert_array_equal(actual,expected)
