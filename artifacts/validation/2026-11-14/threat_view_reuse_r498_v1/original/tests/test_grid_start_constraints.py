import math

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from analyze_grid_start_constraints import constraints, decode_ranges, stationary_scene, translated_peers, placement_checks
from probe_grid_follow import stationary_ranges
from g0_geometry import Footprint, TruthBox

SENSOR = dict(xyz_m=[.06,0.,.095], rate_hz=10., masked_rear_half_angle_deg=30.)
LIDAR = dict(range_min_m=.05,range_max_m=12.)
ROOM = np.array([[[-2.,-.425],[3.,-.425]], [[3.,-.425],[3.,.425]],
                 [[3.,.425],[-2.,.425]], [[-2.,.425],[-2.,-.425]]])
EGO = dict(x=0.,y=0.,yaw_rad=0.,index=0)


def test_close_side_is_blocked_at_zero_sweep_even_with_straight_steering():
    ranges,_ = stationary_ranges(EGO,[TruthBox(-.2,.18,.12,Footprint(.2,.15))],ROOM,SENSOR,500,.05,12.)
    result=constraints(stationary_scene(ranges,SENSOR,LIDAR,0),FollowConfig())
    assert result['initial_inflated_overlap_count'] > 0
    assert result['swept_components'][2]['steering_clearance_m']['0.0']==0.
    assert not result['driving_authorized']


def test_raw_component_does_not_claim_permission_and_age_increases_existing_padding():
    ranges,_ = stationary_ranges(EGO,[TruthBox(-.2,.18,.12,Footprint(.2,.15))],ROOM,SENSOR,500,.05,12.)
    fresh=constraints(stationary_scene(ranges,SENSOR,LIDAR,0),FollowConfig())
    late=constraints(stationary_scene(ranges,SENSOR,LIDAR,4),FollowConfig())
    assert late['minimum_existing_margin_residual_m'] == pytest.approx(fresh['minimum_existing_margin_residual_m']-.04)
    assert all(not r['policy_alternative'] for r in fresh['swept_components'])


def test_clear_room_has_no_initial_overlap_and_both_observed_boundaries():
    ranges,_=stationary_ranges(EGO,[],ROOM,SENSOR,500,.05,12.)
    result=constraints(stationary_scene(ranges,SENSOR,LIDAR,0),FollowConfig())
    assert result['initial_inflated_overlap_count']==0
    assert all(b['length_ok'] and b['anchor_ok'] for b in result['boundary'].values())


def test_unknown_returns_no_free_space_conclusion():
    r=constraints(stationary_scene(np.full(500,np.nan),SENSOR,LIDAR,0),FollowConfig())
    assert r==dict(reason='no_returns',driving_authorized=False)
    assert math.isnan(decode_ranges([{'nonfinite':'nan'}])[0])


def test_peer_translation_is_in_heading_and_does_not_delete_rear_or_original():
    ego=dict(EGO,yaw_rad=math.pi/2)
    peers=[dict(x=1.,y=1.,yaw_rad=math.pi/2,index=1),dict(x=-1.,y=-1.,yaw_rad=math.pi/2,index=2)]
    after=translated_peers(ego,peers,.5,'ahead_only')
    assert after[0]['x']==pytest.approx(1.) and after[0]['y']==1.5
    assert after[1]==peers[1] and peers[0]['y']==1.
    assert [p['index'] for p in after]==[1,2]


@pytest.mark.parametrize('distance,selection',[(-.1,'all'),(math.nan,'all'),(.1,'erase')])
def test_invalid_displacement_rejected(distance,selection):
    with pytest.raises(ValueError):translated_peers(EGO,[],distance,selection)


def test_unregistered_repeat_rejected():
    with pytest.raises(ValueError):stationary_scene(np.ones(500),SENSOR,LIDAR,3)


def test_overlap_is_not_a_usable_departure_case():
    peer=dict(EGO,index=1,x=.1)
    result=placement_checks(EGO,[peer],Footprint(.2,.15))
    assert not result['nonoverlapping'] and result['overlaps'][0]['overlap_area_m2']==pytest.approx(.015)
    assert not result['trajectory_validated'] and result['evaluation_only']
    assert placement_checks(EGO,[dict(peer,x=.5)],Footprint(.2,.15))['nonoverlapping']
