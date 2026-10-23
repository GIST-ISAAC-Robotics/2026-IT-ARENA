"""각 실제 취득 시각에 같은 운동/거리 상한을 적용한다. 점 삭제 없음."""
from types import SimpleNamespace
import numpy as np
import pytest

from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import swept_limit


def scene(now=1.2, times=(1., 1.08, 1.1), statuses=('return', 'masked', 'return')):
    return SimpleNamespace(t_ref_s=now, rays=[SimpleNamespace(acquired_at_s=t, status=s)
                                             for t, s in zip(times, statuses)])


def test_return_order_and_actual_age_preserved_without_new_points():
    assert per_ray_padding(scene(), .055, .5) == pytest.approx([.155, .105])
    assert len(per_ray_padding(scene(), .055, .5)) == 2
    assert FollowConfig().occupancy_timing_mode == 'oldest'
    with pytest.raises(ValueError): FollowConfig(occupancy_timing_mode='newest_for_all')


@pytest.mark.parametrize('fixed,bound,now,times', [(-1,.5,1.2,(1.,)), (.055,-1,1.2,(1.,)),
    (.055,.5,1.2,(1.3,)), (.055,.5,1.2,(float('nan'),)), (True,.5,1.2,(1.,)),
    (.055,.5,float('nan'),(1.,))])
def test_invalid_contract_or_future_point_rejected(fixed, bound, now, times):
    with pytest.raises(ValueError): per_ray_padding(scene(now, times, ('return',)), fixed, bound)


def test_equal_acquisition_times_exactly_match_old_scalar_sweep():
    points = np.array([[.6,.15],[.7,-.3],[1.,.4]])
    margins = per_ray_padding(scene(times=(1.,1.,1.), statuses=('return',)*3), .055, .5)
    for angle in (-.2,0.,.2):
        assert swept_limit(points, angle, margin=margins) == swept_limit(points, angle, margin=float(margins[0]))


def test_newer_unrelated_ray_never_refreshes_old_obstacle():
    points = np.array([[.30,.20],[2.,1.]])
    margins = per_ray_padding(scene(times=(1.,1.19), statuses=('return','return')), .055, .5)
    assert margins[0] == pytest.approx(.155)
    clearance = swept_limit(points, 0., margin=margins)
    assert clearance == swept_limit(points, 0., margin=float(margins[0]))
    assert clearance <= FollowConfig().hold_clearance_m
    assert swept_limit(points, 0., margin=float(margins[1])) > clearance


def test_each_rectangle_contains_same_bounded_motion_and_radial_error():
    rng = np.random.default_rng(22)
    ages = rng.uniform(0., .3, 1000)
    sample = scene(1., tuple(1.-ages), ('return',)*len(ages))
    margins = per_ray_padding(sample, .025+.03, .5)
    direction = rng.uniform(-np.pi,np.pi,len(ages))
    radial = rng.uniform(-.03,.03,len(ages))
    error = np.column_stack((np.cos(direction),np.sin(direction)))*radial[:,None]
    direction = rng.uniform(-np.pi,np.pi,len(ages))
    motion = np.column_stack((np.cos(direction),np.sin(direction)))*(.5*ages)[:,None]
    assert np.all(np.abs(error+motion) <= margins[:,None])
    assert np.all(margins <= .055+.5*max(ages)+1e-12)


def test_zero_motion_keeps_same_fixed_padding_for_every_return():
    assert per_ray_padding(scene(), .055, 0.) == pytest.approx([.055,.055])


@pytest.mark.parametrize('kwargs', [{'near':False}, {'far':False}, {'block':True}, {'unknown':True}])
def test_candidate_preserves_fragment_boundary_and_unknown_and_blocked_stops(kwargs):
    from test_fragmented_follow import observed, history
    from arena_autonomy.local_follow import LocalFollow
    follow = LocalFollow(FollowConfig(boundary_evidence_mode='fragmented_candidate',
                                     occupancy_timing_mode='per_ray_candidate'))
    for t in (.4,.5,.6,.7):
        assert follow.command(observed(t, **kwargs), history(), 0., 'left')[0] == 0


def test_candidate_keeps_all_occupied_points_and_reverse_stop():
    from test_fragmented_follow import observed, history
    from arena_autonomy.local_follow import LocalFollow
    follow = LocalFollow(FollowConfig(boundary_evidence_mode='fragmented_candidate',
                                     occupancy_timing_mode='per_ray_candidate'))
    for t in (.4,.5,.6,.7):
        result = follow.command(observed(t), history(), 0., 'left')
    assert result[0] > 0
    assert result[2]['unfiltered_occupancy']
    assert result[2]['occupancy_padding_points'] == result[2]['occupied_points'] == 270
    assert follow.command(observed(.8), history(), -.01, 'left')[0] == 0


@pytest.mark.parametrize('yaw_rate', [-1., 0., 1.])
def test_motion_projection_preserves_padding_order_across_masks_and_no_returns(yaw_rate):
    import math
    from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
    from arena_autonomy.scene_motion import motion_scene
    count, first, dt = 500, .2, .0002
    ranges = [1.+i/1000 for i in range(count)]
    ranges[90], ranges[250] = math.inf, math.nan
    scan = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=200000000),
                                                  frame_id='laser_frame'),
                           ranges=ranges, angle_min=-math.pi, angle_increment=math.tau/count,
                           range_min=.05, range_max=12., time_increment=dt)
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
    for i in range(41):
        history.add_mean_wheel(i*.01, .8/.0325)
        history.add_gyro(i*.01, yaw_rate)
    earlier = motion_scene(scan, history, .32, timing_verified=True)
    later = motion_scene(scan, history, .38, timing_verified=True)
    returns = [r for r in earlier.rays if r.status == 'return']
    assert len(returns) == len(earlier.occupied_endpoints_xy_m) < count-2
    assert tuple(r.hit_xy_m for r in returns) == earlier.occupied_endpoints_xy_m
    assert [r.source_index for r in returns] == [r.source_index for r in later.rays if r.status == 'return']
    assert [r.acquired_at_s for r in returns] == pytest.approx([first+r.source_index*dt for r in returns])
    expected = [.055+.5*(.32-first-r.source_index*dt) for r in returns]
    assert per_ray_padding(earlier, .055, .5) == pytest.approx(expected)
    assert per_ray_padding(later, .055, .5)-per_ray_padding(earlier, .055, .5) == pytest.approx([.03]*len(returns))
    # 재투영은 좌표만 바꾸며 원래 취득 시각이나 가림/무반사 상태를 갱신하지 않는다.
    assert not np.allclose(earlier.occupied_endpoints_xy_m, later.occupied_endpoints_xy_m)
