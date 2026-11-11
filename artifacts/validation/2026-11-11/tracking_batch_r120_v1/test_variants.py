import math
from dataclasses import replace
import numpy as np
import pytest
from variants import depth_variant, border_variant
from arena_autonomy.observation_support import depth_support
from arena_autonomy.observed_lateral_trial import path_observation_queries
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.m3_observed_follow import command_arc
from test_lateral_intent import GRASS
from test_local_follow import scan, history


@pytest.mark.parametrize('chunk',[512,1024])
@pytest.mark.parametrize('speed,yaw',[(0.,0.),(.35,.8),(.9,-1.3)])
def test_depth_batch_does_not_change_random_origins_gaps_or_boundaries(chunk,speed,yaw):
    rng=np.random.default_rng(1111)
    ranges=rng.uniform(.15,12.,500)
    ranges[rng.choice(500,40,replace=False)]=math.inf
    scene=motion_scene(scan(.5,ranges.tolist()),history(speed,yaw),.635,
        timing_verified=True,rear_half_angle_deg=30.)
    origins=np.asarray([r.ray_origin_xy_m for r in scene.rays])
    directions=np.asarray([r.direction_xy for r in scene.rays])
    q=np.vstack((rng.uniform(-2.,2.,(6000,2)),origins,origins+directions*1e-9,
                 origins+directions*.5))
    padding=np.full(500,.055)
    np.testing.assert_array_equal(depth_variant(chunk)(scene,q,padding),depth_support(scene,q,padding))
    assert depth_variant(chunk)(scene,[],padding).shape==(0,)


@pytest.mark.parametrize('angle',[0.,-1e-9,1e-9,-.0003,.175,-.37])
@pytest.mark.parametrize('distance',[.01,.28,.65])
def test_border_batch_preserves_every_query_and_thin_intersection(angle,distance):
    ranges=[3. if i%13 else math.inf for i in range(500)]
    scene=motion_scene(scan(.5,ranges),history(.7,.8),.635,
        timing_verified=True,rear_half_angle_deg=30.)
    poses=command_arc(angle,distance,GRASS)
    a,n=path_observation_queries(scene,poses,GRASS)
    b,m=border_variant()(scene,poses,GRASS)
    assert n==m
    np.testing.assert_array_equal(a,b)
