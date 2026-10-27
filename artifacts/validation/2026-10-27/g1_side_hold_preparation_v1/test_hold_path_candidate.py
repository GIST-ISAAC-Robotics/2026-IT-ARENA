import math
import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from test_g1_partial_policy import corridor
from test_local_follow import history
from hold_path_candidate import candidate


def base(offset=.11, slope=0.):
    x=np.linspace(-.1,1.2,40)
    return (0.,.06,{'reason':'follow_gap_hold','path_points':np.c_[x,slope*x+offset].tolist()})


def scene(obstacle=None):
    return motion_scene(corridor(.5,obstacle=obstacle),history(),.6,timing_verified=True)


@pytest.mark.parametrize('offset',[-.11,.11])
def test_parallel_path_preserves_current_offset_instead_of_recentering(offset):
    s=scene();original=np.array(s.occupied_endpoints_xy_m)
    r=candidate(s,base(offset),0.,0.,.02,FollowConfig(occupancy_timing_mode='per_ray_candidate'))
    assert r['eligible'] and r['target_speed_mps']==.35
    assert abs(r['steering_rad'])<1e-12 and r['maximum_body_corner_lateral_m']<1e-12
    assert not r['unknown_filled'] and not r['runtime_authorized']
    np.testing.assert_array_equal(s.occupied_endpoints_xy_m,original)


def test_front_obstacle_keeps_original_gap_hold():
    r=candidate(scene(.25),base(),0.,0.,.02,FollowConfig())
    assert not r['eligible'] and r['reason']=='unchanged_gap_hold' and r['target_speed_mps']==0.


def test_close_side_return_is_not_removed_to_make_parallel_motion_pass():
    raw=corridor(.5)
    raw.ranges[375]=.12
    s=motion_scene(raw,history(),.6,timing_verified=True)
    r=candidate(s,base(),0.,0.,.02,FollowConfig())
    assert not r['eligible'] and r['reason']=='unchanged_gap_hold'
    assert r['unfiltered_occupancy'] and r['occupied_points']==len(s.occupied_endpoints_xy_m)


@pytest.mark.parametrize('speed,previous',[(1.,.2),(.35,.37)])
def test_existing_motion_and_previous_steering_cannot_be_erased(speed,previous):
    r=candidate(scene(),base(),speed,previous,.02,FollowConfig())
    assert not r['eligible'] and r['reason']=='lateral_motion_not_small'


@pytest.mark.parametrize('fault',['curve','missing_path','missing_scan','reverse','invalid_speed'])
def test_unconfirmed_inputs_never_become_proposals(fault):
    b=base();speed=0.
    if fault=='curve':b=base(slope=math.tan(math.radians(8)))
    elif fault=='missing_path':b[2]['path_points']=[]
    elif fault=='missing_scan':b[2]['reason']='follow_front_unobserved'
    elif fault=='reverse':speed=-.1
    else:speed=math.nan
    assert not candidate(scene(),b,speed,0.,.02,FollowConfig())['eligible']
