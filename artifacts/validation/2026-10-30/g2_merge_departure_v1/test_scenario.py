import math
import pytest
from scenario import Scenario


def test_unstarted_and_old_queries_stay_still():
    s=Scenario('s_merge',.95)
    assert s.pose_at(100)==pytest.approx((1.2,0.,0.),abs=1e-12)
    s.observe_ego(10,(.95,0.,0.))
    assert s.pose_at(9)==pytest.approx((1.2,0.,0.),abs=1e-12)
    s.observe_ego(11,(1.,0.,0.))
    assert s.start==10


def test_distance_velocity_and_acceleration_schedule():
    s=Scenario('s_merge',.95)
    assert s.distance_at(.5)==pytest.approx(.125)
    assert s.distance_at(5)==pytest.approx(2.375)
    assert s.distance_at(5.5)==pytest.approx(2.5)
    assert s.distance_at(100)==2.5
    for t in (.1,.499,.501,2.,4.999,5.001,5.4):
        v=(s.distance_at(t+1e-5)-s.distance_at(t-1e-5))/2e-5
        assert 0<=v<=.500001


def test_s_bends_are_continuous_and_finish_straight():
    s=Scenario('s_merge',.95);s.start=0.
    for distance in (s.radius*s.angle,2*s.radius*s.angle):
        at=.5+(distance-.125)/.5
        before,after=s.pose_at(at-1e-7),s.pose_at(at+1e-7)
        assert max(abs(a-b) for a,b in zip(before,after))<1e-6
    x,y,yaw=s.pose_at(20)
    assert (x,y,yaw)==pytest.approx((1.2+2*s.radius*math.sin(s.angle)+2.5-2*s.radius*s.angle,
        -2*s.radius*(1-math.cos(s.angle)),0.))


def test_body_remains_within_track_walls():
    s=Scenario('s_merge',.95);s.start=0.
    for k in range(601):
        target=s.boxes(k*.01)[-1]
        polygon=target.footprint.polygon(target.x_m,target.y_m,target.yaw_rad)
        assert all(-.425<y<.425 and -2<x<6 for x,y in polygon)
    assert math.atan(.135/s.radius)<.4
