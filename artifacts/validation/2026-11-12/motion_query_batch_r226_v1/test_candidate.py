"""두 원호를 묶어도 unknown 교차와 외곽 표본이 같은지 기존 함수와 대조."""
import math
import numpy as np
import pytest
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history
from candidate import motion_queries


@pytest.mark.parametrize('missing',['none','all','thin','mixed'])
@pytest.mark.parametrize('angles',[(0.,0.),(.21,.21),(-.27,.19),(-1e-9,1e-9),(.4,0.),(0.,-.4)])
@pytest.mark.parametrize('speed,target',[(0.,.2),(-.2,.3),(.9,0.)])
def test_exact_query_set(missing,angles,speed,target):
    rng=np.random.default_rng(226)
    ranges=rng.uniform(.2,6.,500)
    if missing=='all':ranges[:]=math.inf
    elif missing=='thin':ranges[[0,1,249,250,498,499]]=math.inf
    elif missing=='mixed':ranges[rng.choice(500,110,replace=False)]=math.inf
    scene=motion_scene(scan(.5,ranges.tolist()),history(speed,.6),.635,
        timing_verified=True,rear_half_angle_deg=20.)
    cfg=FollowConfig(wheelbase_m=.135)
    args=(scene,speed,target,*angles,.02,cfg,ps.lateral_config(cfg))
    distance,expected=ps.motion_queries(*args)
    other,actual=motion_queries(*args)
    assert distance==other
    np.testing.assert_array_equal(actual,expected)
