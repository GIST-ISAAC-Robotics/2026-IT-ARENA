"""네이티브 연산만의 동등성. 기본 source/window/미관측 계약과 비교한다."""
from dataclasses import replace
import math
from pathlib import Path
import sys

import numpy as np
import pytest

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'tests')]
import candidate
from arena_autonomy.observation_support import depth_support
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_topology import ScanCycle
from test_local_follow import history,scan


@pytest.mark.parametrize('speed,yaw',[(0.,0.),(.2,.7),(.9,-1.3),(1.,2.)])
@pytest.mark.parametrize('seed',[19,73,1019])
def test_moving_origins_nan_unknown_and_boundaries(speed,yaw,seed):
    rng=np.random.default_rng(seed)
    raw=scan(.5,rng.uniform(.06,12.,500).tolist())
    for i in rng.choice(500,30,replace=False):raw.ranges[i]=math.inf
    scene=motion_scene(raw,history(speed,yaw),.635,timing_verified=True,rear_half_angle_deg=30.)
    q=list(rng.uniform([-.3,-.7],[2.,.7],(600,2)))
    for ray in [*scene.rays[:3],*scene.rays[248:253],*scene.rays[-3:]]:
        o,d=np.array(ray.ray_origin_xy_m),np.array(ray.direction_xy)
        q.append(o)
        for angle in (0.,-1e-11,1e-11,-1e-9,1e-9,-.025,.025):
            c,s=math.cos(angle),math.sin(angle)
            q.append(o+.2*np.array([c*d[0]-s*d[1],s*d[0]+c*d[1]]))
        for delta in (0.,-1e-12,1e-12):q.append(o+(1e-9+delta)*d)
    q.extend([(math.nan,0.),(math.inf,0.)])
    pad=rng.uniform(0.,.2,500)
    with np.errstate(invalid='ignore'):
        np.testing.assert_array_equal(candidate.depth_support(scene,q,pad),depth_support(scene,q,pad))


@pytest.mark.parametrize('count',[0,1,127,512,513,1701])
def test_full_ray_fallback_chunk_and_no_mutation(count):
    rng=np.random.default_rng(19)
    scene=motion_scene(scan(.5),history(.9,-1.3),.635,timing_verified=True)
    q=rng.uniform([-.04,-.03],[.08,.03],(count,2));before=q.copy();pad=np.full(500,.05)
    np.testing.assert_array_equal(candidate.depth_support(scene,q,pad),depth_support(scene,q,pad))
    np.testing.assert_array_equal(q,before)


@pytest.mark.parametrize('cycle_enabled',[False,True])
def test_source_seam_depth_equality_ties_and_changed_indices(cycle_enabled):
    scene=motion_scene(scan(.5,[2.]*500),history(0.,0.),.635,timing_verified=True,rear_half_angle_deg=0.)
    rays=list(scene.rays)
    # 동일 방향과 부적합 반환이 겹쳐도 첫 argmin 후보가 바뀌면 안 된다.
    rays[2]=replace(rays[2],direction_xy=rays[1].direction_xy,hit_xy_m=rays[1].hit_xy_m)
    scene=replace(scene,rays=tuple(rays))
    cycle=ScanCycle(scene,-math.pi,math.tau/500,500,True,None) if cycle_enabled else None
    q=[]
    for ray in (rays[0],rays[1],rays[2],rays[-1]):
        o,d=np.array(ray.ray_origin_xy_m),np.array(ray.direction_xy)
        depth=math.dist(ray.ray_origin_xy_m,ray.hit_xy_m)
        for dist in (.2,depth-.05,np.nextafter(depth-.05,0.),np.nextafter(depth-.05,math.inf)):
            q.append(o+dist*d)
    pad=np.full(500,.05)
    np.testing.assert_array_equal(candidate.depth_support(scene,q,pad,cycle=cycle),depth_support(scene,q,pad,cycle=cycle))
