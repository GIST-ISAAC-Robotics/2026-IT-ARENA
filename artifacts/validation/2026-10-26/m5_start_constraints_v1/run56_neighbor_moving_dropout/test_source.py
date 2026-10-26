"""평가 actor 각본 변경을 센서/운동 감사가 동일하게 해석하는지 검사한다."""
import math
import numpy as np
import pytest
from shapely.geometry import MultiLineString,box
from simulate_grid_departure import grid_peer_velocities,simulate
from audit_grid_departure_motion import audit,audited_peer_moving
from surface_contact_eval import VehicleGeometry


def slot(index,x,y,yaw=0.):return dict(index=index,x=x,y=y,yaw_rad=yaw)


@pytest.mark.parametrize('yaw',[0.,.37,math.pi/2,-2.1])
def test_peer_choreography_is_relative_to_ego_pose_not_world_axis(yaw):
    def placed(index,x,y):
        return slot(index,2+x*math.cos(yaw)-y*math.sin(yaw),-3+x*math.sin(yaw)+y*math.cos(yaw),yaw)
    ego=placed(2,0,0)
    peers=[placed(1,.4,0),placed(4,-.4,0),placed(5,-.2,.18),placed(0,-.6,-.18)]
    ahead=grid_peer_velocities(ego,peers,.15,'ahead_only')
    neighbor=grid_peer_velocities(ego,peers,.15,'ahead_or_other_lane')
    moving=(.5*math.cos(yaw),.5*math.sin(yaw))
    np.testing.assert_allclose(ahead,[moving,(0.,0.),(0.,0.),(0.,0.)],atol=1e-14)
    np.testing.assert_allclose(neighbor,[moving,(0.,0.),moving,moving],atol=1e-14)
    assert audited_peer_moving(ego,peers,.15,{'peer_motion_policy':'ahead_only'})==[True,False,False,False]
    assert audited_peer_moving(ego,peers,.15,{'peer_motion_policy':'ahead_or_other_lane'})==[True,False,True,True]


def test_old_missing_policy_preserves_ahead_only_and_same_lane_rear_stays_fixed():
    ego=slot(2,0,0);peers=[slot(5,-.2,.18),slot(1,.4,0),slot(4,-.4,0)]
    assert grid_peer_velocities(ego,peers,.15)==[(0.,0.),(.5,0.),(0.,0.)]
    assert audited_peer_moving(ego,peers,.15,{})==[False,True,False]
    with pytest.raises(ValueError,match='policy'):grid_peer_velocities(ego,peers,.15,'all')
    with pytest.raises(ValueError,match='policy'):audited_peer_moving(ego,peers,.15,{'peer_motion_policy':'all'})


def test_audit_tracks_rear_adjacent_actor_passing_stationary_ego():
    slots=[slot(2,0,0),slot(5,-.6,.18)]
    walls=MultiLineString([[[-1,-1],[4,-1]],[[-1,1],[4,1]]]);road=box(-2,-2,5,2)
    case=dict(summary=dict(rounded=True,fault=False),calls=[dict(t_s=5.99,truth_pose=[0.,0.,0.],safe_mps=0.,safe_steering=0.)])
    old=audit(case,slots,walls,road,VehicleGeometry(),ego_index=2)
    case['summary']['peer_motion_policy']='ahead_or_other_lane'
    new=audit(case,slots,walls,road,VehicleGeometry(),ego_index=2)
    assert old['minimum_peer_gap_lower_m']>.38
    assert .0294<new['minimum_peer_gap_lower_m']<.0301
    assert new['passed']
    assert new['minimum_peer_location']['t_s']>.5


@pytest.mark.parametrize('frames',[(34,30),(-1,3),(0,56),(1.,3),(True,3),(3,),None])
def test_fault_frame_typo_rejected_before_any_plant_or_input_access(frames):
    with pytest.raises(ValueError,match='fault frames'):
        simulate(None,None,None,None,None,None,rounded=True,fault_frames=frames)
