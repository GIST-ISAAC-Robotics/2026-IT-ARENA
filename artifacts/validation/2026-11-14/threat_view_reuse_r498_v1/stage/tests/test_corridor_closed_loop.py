import math
import numpy as np
import pytest

import compare_corridor_closed_loop as sim
from arena_autonomy.local_follow import FollowConfig
from probe_corner_observability import annular_scan
from test_local_follow import history


def test_straight_integrate_and_constant_twist_composition():
    np.testing.assert_allclose(sim.integrate([0,0,0],1.,0.,.1),[.1,0,0])
    full=sim.integrate([0,0,0],.6,1.,.1)
    half=sim.integrate(sim.integrate([0,0,0],.6,1.,.05),.6,1.,.05)
    np.testing.assert_allclose(full,half,atol=1e-15)


def test_ray_acquisition_uses_past_zoh_not_future_interpolation():
    truth=np.array([[0.,0,0,0,1.,0],[.1,99,99,0,0,0]])
    np.testing.assert_allclose(sim.poses_at(truth,np.array([.025,.05]))[:,0],[.025,.05])
    with pytest.raises(ValueError):sim.poses_at(truth,np.array([.11]))


@pytest.mark.parametrize('sign',[1,-1])
def test_stationary_rolling_scan_matches_existing_annulus(sign):
    truth=np.array([[t,0,0,0,0,0] for t in np.arange(0,.111,.01)])
    scan=sim.scan_at(.1,truth,sign,'open')
    expected=annular_scan(.15,sign)[0]
    np.testing.assert_allclose(scan.ranges,expected,atol=1e-12)
    assert scan.header.stamp.sec==0 and scan.header.stamp.nanosec==0
    assert scan.time_increment==.0002


def test_first_box_intersection_and_rotation():
    origins=np.array([[0.,0],[.48,0],[0,1.]])
    directions=np.array([[1.,0],[1.,0],[1.,0]])
    hits=sim.box_hits(origins,directions,[.5,0],0.)
    np.testing.assert_allclose(hits[:2],[.4,.12])
    assert math.isinf(hits[2])
    assert sim.box_hits(origins[:1],directions[:1],[.5,0],math.pi/2)[0]==pytest.approx(.425)


@pytest.mark.parametrize('sign',[1,-1])
def test_footprint_road_and_obstacle_checks(sign):
    assert sim.footprint_audit([0,0,0],sign,'open')['inside_road']
    _,box=sim.environment(sign,'blocked')
    result=sim.footprint_audit([*box[0],box[1]],sign,'blocked')
    assert result['collision'] and result['body_gap_m']==0
    assert not sim.footprint_audit([0,-sign*.3,0],sign,'open')['inside_road']


def test_handoff_is_diagnostic_and_rejects_displaced_path():
    path=np.column_stack((np.linspace(0,1,30),np.zeros(30)))
    cfg=FollowConfig()
    assert sim.handoff_agreement((.5,path),path,.52,history(),cfg)
    assert not sim.handoff_agreement((.5,path),path+[0,.2],.52,history(),cfg)
    assert sim.handoff_agreement((.5,path),path,.9,history(),cfg) is None


def test_short_run_cannot_claim_half_turn_or_real_vehicle_success():
    result=sim.run_case(1,'open','baseline',duration=.12)
    assert not result['checks']['half_course_progress']
    assert not result['passed'] and not result['ros_connected'] and not result['real_vehicle_verified']
