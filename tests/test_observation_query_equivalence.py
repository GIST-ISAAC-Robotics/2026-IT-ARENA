"""정방향 필요조건 최적화와 이전 cross-only 질의의 정확 대조."""
import math

import numpy as np
import pytest

from arena_autonomy.linear_observation import clip_halfplane
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import outside_body, sweep_queries
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import history, scan


def reference_queries(scene, steer, distance, config):
    """2026-10-17 cost_r0의 cross-only 판정. 새 필요조건을 공유하지 않는다."""
    curve=math.tan(steer)/config.wheelbase_m
    lx,wy=config.length_m/2,config.width_m/2
    corner=np.array([[-lx,-wy],[lx,-wy],[lx,wy],[-lx,wy]])
    border=np.unique(np.vstack(([(x,y) for x in np.linspace(-lx,lx,11) for y in (-wy,wy)],
                                [(x,y) for x in (-lx,lx) for y in np.linspace(-wy,wy,9)])),axis=0)
    unknown=[r for r in scene.rays if r.status!='return']
    queries=[];thin=0
    co,si=math.cos(math.pi/500),math.sin(math.pi/500)
    origins=np.array([r.ray_origin_xy_m for r in unknown]).reshape(-1,2)
    directions=np.array([r.direction_xy for r in unknown]).reshape(-1,2)
    lower=directions@np.array([[co,-si],[si,co]])
    upper=directions@np.array([[co,si],[-si,co]])
    for travel in np.r_[np.arange(.02,distance,.02),distance]:
        yaw=curve*travel;c,s=math.cos(yaw),math.sin(yaw)
        x=travel if curve==0 else math.sin(yaw)/curve
        y=0. if curve==0 else 2*math.sin(yaw*.5)**2/curve
        center=np.array([x-config.wheelbase_m/2+config.wheelbase_m/2*c,y+config.wheelbase_m/2*s])
        rotation=np.array([[c,s],[-s,c]])
        b=border@rotation+center
        queries.extend(b[(abs(b[:,0])>lx+1e-10)|(abs(b[:,1])>wy+1e-10)])
        polygon=(corner@rotation+center).tolist()
        possible=np.zeros(len(unknown),dtype=bool)
        for exterior in outside_body(polygon,config.length_m,config.width_m):
            delta=np.array(exterior)[None]-origins[:,None]
            low=lower[:,None,0]*delta[:,:,1]-lower[:,None,1]*delta[:,:,0]
            high=upper[:,None,0]*delta[:,:,1]-upper[:,None,1]*delta[:,:,0]
            possible|=~(np.all(low<0,axis=1)|np.all(high>0,axis=1))
        for i in np.flatnonzero(possible):
            r=unknown[i]
            cut=clip_halfplane(clip_halfplane(polygon,r.ray_origin_xy_m,lower[i].tolist(),1),
                               r.ray_origin_xy_m,upper[i].tolist(),-1)
            for piece in outside_body(cut,config.length_m,config.width_m):
                queries.extend(piece);queries.append(np.mean(piece,axis=0));thin+=1
    queries=np.asarray(queries).reshape(-1,2)
    exterior=(abs(queries[:,0])>lx+1e-10)|(abs(queries[:,1])>wy+1e-10)
    return np.unique(queries[exterior],axis=0),thin


@pytest.mark.parametrize('velocity,yaw',[(0.,0.),(.2,.7),(.9,-1.3),(.3,2.)])
@pytest.mark.parametrize('angle',[0.,-9.6137665e-9,9.6137665e-9,-.001,.001,-.175,.175,-.37,.37])
@pytest.mark.parametrize('distance',[.12,.27,.55,.9])
def test_wedge_forward_prefilter_preserves_every_query_and_thin_intersection(velocity,yaw,angle,distance):
    rng=np.random.default_rng(1017)
    raw=scan(.5,rng.uniform(.2,12.,500).tolist())
    for i in rng.choice(500,30,replace=False):raw.ranges[i]=math.inf
    scene=motion_scene(raw,history(velocity,yaw),.635,timing_verified=True,rear_half_angle_deg=30.)
    cfg=FollowConfig(wheelbase_m=.135)
    expected,n=reference_queries(scene,angle,distance,cfg)
    actual,m=sweep_queries(scene,angle,distance,cfg)
    assert m==n
    np.testing.assert_array_equal(actual,expected)


@pytest.mark.parametrize('missing',[False,True])
def test_all_or_no_unknown_is_not_skipped_as_free_space(missing):
    raw=scan(.5,[math.inf if missing else 3.]*500)
    scene=motion_scene(raw,history(.2,.5),.635,timing_verified=True,rear_half_angle_deg=0.)
    cfg=FollowConfig(wheelbase_m=.135)
    expected,n=reference_queries(scene,.21,.3,cfg)
    actual,m=sweep_queries(scene,.21,.3,cfg)
    assert m==n
    np.testing.assert_array_equal(actual,expected)
