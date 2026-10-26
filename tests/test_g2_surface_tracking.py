import math
import pytest

from g0_geometry import TruthBox
from g2_surface_tracking import inspect_rollout
from g2_tracking_probe import bicycle_step


def rows_at(y=0., steering=.0, speed=.4, n=100):
    pose = (-.0675,y,0.)
    result=[]
    for i in range(n):
        result.append(dict(t_s=i*.02,rear_x_m=pose[0],rear_y_m=pose[1],yaw_rad=pose[2],
                           applied_steering_rad=steering,applied_speed_mps=speed))
        pose=bicycle_step(*pose,speed,steering,.02,.135)
    return result


def test_center_straight_keeps_four_wheels_and_never_authorizes_motion():
    r=inspect_rollout(rows_at(),TruthBox(1.5,.3,.12))
    assert r['maximum_outside_wheels_upper']==0
    assert r['maximum_grass_duration_upper_s']==0
    assert r['checks']['two_wheel_surface']
    assert not r['driving_authorized']


def test_one_side_grass_count_and_three_second_team_limit_are_separate():
    r=inspect_rollout(rows_at(y=-.16,speed=0.,n=160),TruthBox(1.5,.15,.12))
    assert r['maximum_outside_wheels_upper']==2
    assert r['checks']['two_wheel_surface']
    assert r['maximum_grass_duration_upper_s']==pytest.approx(3.2)
    assert not r['checks']['grass_duration_within_team_limit']


def test_short_hidden_between_tick_collision_is_not_missed():
    # One 20 ms segment approaches a very close target; lower bound must fail even if first sample is separated.
    r=inspect_rollout(rows_at(speed=.8,n=1),TruthBox(.21,0.,.12),subdivisions=1)
    assert r['minimum_gap_lower_m']==0.
    assert not r['checks']['body_and_wheel_gap']


def test_mirrored_steering_paths_have_same_gap_and_surface():
    a=inspect_rollout(rows_at(y=-.1,steering=.05,n=30),TruthBox(1.5,.15,.12))
    b=inspect_rollout(rows_at(y=.1,steering=-.05,n=30),TruthBox(1.5,-.15,.12))
    assert a['checks']==b['checks']
    assert a['minimum_gap_lower_m']==pytest.approx(b['minimum_gap_lower_m'])
    assert a['minimum_inner_patch_edge_lower_m']['left']==pytest.approx(b['minimum_inner_patch_edge_lower_m']['right'])


@pytest.mark.parametrize('kind',['empty','nonuniform','nonfinite'])
def test_incomplete_or_invalid_time_series_is_rejected(kind):
    rows=rows_at(n=3)
    if kind=='empty': rows=[]
    elif kind=='nonuniform': rows[1]['t_s']=.1
    else: rows[1]['rear_x_m']=math.nan
    with pytest.raises(ValueError): inspect_rollout(rows,TruthBox(1.5,.15,.12))
