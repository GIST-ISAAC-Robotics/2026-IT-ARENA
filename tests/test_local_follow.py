"""G1-a의 순수 계약/판정 회귀. 합성 관측을 차량 물리 검증으로 부르지 않는다."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import json
import math

import numpy as np
import pytest

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.local_scene import SensorRay, LocalScene, scene_from_scan
from arena_autonomy.scene_motion import motion_scene, transport_points
from arena_autonomy.local_follow import FollowConfig, LocalFollow, pursue_path, boundary_support, drop_protrusions
from arena_autonomy.local_path import pursuit_command, local_path, mask_scan


def history(v=0., w=0., end=3.):
    result = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675,
                                       lidar_x_m=.06, history_s=5.))
    for time in np.arange(0., end+.0001, .01):
        result.add_mean_wheel(float(time), v/.0325)
        result.add_gyro(float(time), w)
    return result


def scan(first=.5, ranges=None):
    return SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame', stamp=SimpleNamespace(
        sec=int(first), nanosec=round((first-int(first))*1e9))),
        ranges=[1.]*500 if ranges is None else ranges, angle_min=-math.pi,
        angle_increment=math.tau/500, time_increment=.0002, range_min=.05, range_max=12.)


def scene(now, obstacle_x=None, *, epoch=0, side_car=False, missing_wall=False, source=None):
    # 임의 점을 사용하는 계약 단위 시험이며 실제 센서 분해능 시험이 아니다.
    xs = np.linspace(-.25, 2.0, 140)
    left = np.column_stack((xs, xs*0+.425)) if not missing_wall else np.empty((0, 2))
    right = np.column_stack((xs, xs*0-.425))
    points = [left, right]
    if obstacle_x is not None:
        points.append(np.column_stack((np.full(20, obstacle_x), np.linspace(-.075, .075, 20))))
    if side_car:
        points.append(np.column_stack((np.linspace(-.1, .1, 30), np.full(30, .18))))
    points = np.vstack(points)
    first = now-.1 if source is None else source
    rays = []
    for i, point in enumerate(points):
        d = point-np.array([.06, 0.])
        d /= np.linalg.norm(d)
        rays.append(SensorRay((.06, 0.), tuple(point), first, 'return', 'laser_frame', epoch, i, tuple(d)))
    return LocalScene(now, first, first, 'laser_frame', epoch, tuple(rays))


def settle(controller, h, **kwargs):
    result = None
    for now in (.4, .5, .6, .7):
        result = controller.command(scene(now, **kwargs), h, 0., 'left')
    return result


@pytest.mark.parametrize('v,w', [(0., 0.), (1., 0.), (1., .8), (.2, -1.)])
def test_scene_endpoints_match_existing_both_projection(v, w):
    h = history(v, w)
    raw = scan()
    new = motion_scene(raw, h, .7, timing_verified=True, rear_half_angle_deg=0.)
    old, indices, _ = h.project(raw.ranges, raw.angle_min, raw.angle_increment, .05, 12.,
                               .5, .0002, .7, 'both', True)
    np.testing.assert_allclose(new.occupied_endpoints_xy_m, old, atol=1e-12)
    assert new.source_first_s == .5 and new.source_last_s == pytest.approx(.5998)
    assert new.t_ref_s == .7 and len(indices) == 500


def test_unknown_ray_origin_and_direction_are_transported_without_return():
    raw = scan(ranges=[float('inf')]*500)
    new = motion_scene(raw, history(1., 1.), .7, timing_verified=True, rear_half_angle_deg=0.)
    assert all(ray.hit_xy_m is None and ray.status == 'unknown' for ray in new.rays)
    assert new.rays[0].ray_origin_xy_m != new.rays[-1].ray_origin_xy_m
    assert new.rays[0].acquired_at_s == .5


@pytest.mark.parametrize('kind', ['unverified', 'frame', 'zero_step', 'future', 'stale', 'gap'])
def test_scene_refuses_unverified_or_missing_motion(kind):
    raw, h, now, verified = scan(), history(), .7, True
    if kind == 'unverified': verified = False
    if kind == 'frame': raw.header.frame_id = 'world'
    if kind == 'zero_step': raw.time_increment = 0.
    if kind == 'future': now = .55
    if kind == 'stale': now = .81
    if kind == 'gap': h = MotionHistory()
    with pytest.raises(MotionUnavailable):
        motion_scene(raw, h, now, timing_verified=verified)


def test_transport_translation_and_yaw_not_commanded_motion():
    np.testing.assert_allclose(transport_points([[1., 0.]], history(1.), .4, .5), [[.9, 0.]], atol=1e-12)
    h = history(1., .8)
    point = np.array([[.5, .4]])
    moved = transport_points(point, h, .4, .6)
    back = transport_points(moved, h, .6, .6)
    np.testing.assert_allclose(back, moved)
    assert not np.allclose(moved, point)


def test_same_scan_cannot_confirm_or_refresh_boundary():
    ctl, h = LocalFollow(), history()
    for now in (.4, .42, .44, .46):
        speed, _, meta = ctl.command(scene(now, source=.3), h, 0., 'left')
        assert speed == 0 and meta['behavior'] == 'HOLD'
    assert ctl.pending_count == 1


def test_clean_corridor_cruises_and_all_occupied_points_remain():
    speed, steering, meta = settle(LocalFollow(), history())
    assert speed == 1. and abs(steering) < 1e-8 and meta['behavior'] == 'CRUISE'
    assert meta['occupied_points'] == 280 and meta['unfiltered_occupancy']


def test_short_nearby_side_vehicle_cannot_become_wall():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    _, steering, meta = ctl.command(scene(.8, side_car=True, missing_wall=True), h, .5, 'left')
    assert abs(steering) < 1e-6
    assert meta['boundary_from_memory'] and meta['occupied_points'] == 170
    assert meta['path_source_s'] == pytest.approx(.6)
    speed, _, meta = ctl.command(scene(1.0, side_car=True, missing_wall=True), h, 0., 'left')
    assert speed == 0 and meta['reason'] == 'follow_boundary_unconfirmed'


def test_one_wall_only_cannot_initialize_new_path():
    speed, _, meta = settle(LocalFollow(), history(), missing_wall=True)
    assert speed == 0 and meta['behavior'] == 'HOLD'


@pytest.mark.parametrize('side', ['left', 'right'])
def test_side_occlusion_memory_expires_and_recovers_without_using_car_as_wall(side):
    ctl, h = LocalFollow(), history()
    def observed(now, hidden=False):
        result = scene(now, missing_wall=hidden, side_car=hidden)
        if side == 'right':
            result = replace(result, rays=tuple(replace(ray,
                ray_origin_xy_m=(ray.ray_origin_xy_m[0], -ray.ray_origin_xy_m[1]),
                hit_xy_m=(ray.hit_xy_m[0], -ray.hit_xy_m[1]),
                direction_xy=(ray.direction_xy[0], -ray.direction_xy[1])) for ray in result.rays))
        return result
    for now in (.4, .5, .6, .7):
        speed, _, meta = ctl.command(observed(now), h, 0., side)
    assert speed > 0
    for now in (.8, .9):
        _, steering, meta = ctl.command(observed(now, True), h, 0., side)
        assert abs(steering) < 1e-6
    speed, _, meta = ctl.command(observed(1., True), h, 0., side)
    assert speed == 0 and meta['reason'] == 'follow_boundary_unconfirmed'
    assert ctl.path is None
    for now in (1.1, 1.2, 1.3, 1.4):
        speed, steering, meta = ctl.command(observed(now), h, 0., side)
    assert speed > 0 and abs(steering) < 1e-6
    assert meta['unfiltered_occupancy'] and meta['occupied_points'] == 280


def test_cold_start_occlusion_requires_fresh_bilateral_recovery():
    ctl, h = LocalFollow(), history()
    # 실제 광선 가림을 재현한 시험이 아니라 관측 계약 시험이다.
    # 앞 물체와 한쪽 벽만 보이는 동안 기억 경로를 새로 만들면 안 된다.
    for now in (.4, .5, .6, .7):
        speed, _, meta = ctl.command(scene(now, obstacle_x=.45, missing_wall=True), h, 0., 'left')
        assert speed == 0 and meta['behavior'] == 'HOLD'
        assert ctl.path is None
    speed, _, _ = ctl.command(scene(.8), h, 0., 'left')
    assert speed == 0
    speed, _, _ = ctl.command(scene(.82, source=.7), h, 0., 'left')
    assert speed == 0  # 같은 프레임 재사용은 확인 횟수를 늘리지 않는다.
    for now in (.9, 1., 1.1):
        speed, _, meta = ctl.command(scene(now), h, 0., 'left')
    assert speed > 0 and meta['behavior'] == 'CRUISE'


def test_stopped_front_object_hold_and_two_fresh_observations_to_resume():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    speed, _, meta = ctl.command(scene(.8, obstacle_x=.40), h, .5, 'left')
    assert speed == 0 and meta['reason'] == 'follow_gap_hold'
    speed, _, meta = ctl.command(scene(.9, obstacle_x=1.), h, 0., 'left')
    assert speed == 0 and meta['reason'] == 'follow_release_wait'
    speed, _, _ = ctl.command(scene(.92, obstacle_x=1., source=.8), h, 0., 'left')
    assert speed == 0
    speed, steering, meta = ctl.command(scene(1., obstacle_x=1.), h, 0., 'left')
    assert 0 < speed < 1 and abs(steering) < 1e-6 and meta['behavior'] == 'FOLLOW'


def test_occupancy_never_comes_from_boundary_memory():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    speed, _, meta = ctl.command(scene(.8, obstacle_x=.35, missing_wall=True), h, .5, 'left')
    assert speed == 0 and meta['boundary_from_memory']
    assert meta['occupied_points'] == 160


def test_unknown_front_holds_but_rear_unknown_does_not_cancel_cruise():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    base = scene(.8)
    unknown = SensorRay((.06, 0.), None, base.source_first_s, 'unknown', 'laser_frame', 0, 999, (1., 0.))
    speed, _, meta = ctl.command(replace(base, rays=base.rays+(unknown,)), h, .5, 'left')
    assert speed == 0 and meta['reason'] == 'follow_front_unobserved'
    ctl = LocalFollow()
    settle(ctl, h)
    unknown = replace(unknown, status='masked', direction_xy=(-1., 0.))
    speed, _, meta = ctl.command(replace(base, rays=base.rays+(unknown,)), h, .5, 'left')
    assert speed > 0


@pytest.mark.parametrize('change', ['epoch', 'side', 'time_rewind'])
def test_new_epoch_side_or_time_rewind_drops_previous_path(change):
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    raw = scene(.8, epoch=1 if change == 'epoch' else 0)
    if change == 'time_rewind': raw = scene(.3)
    speed, _, meta = ctl.command(raw, h, 0., 'right' if change == 'side' else 'left')
    assert speed == 0 and meta['behavior'] == 'HOLD'


def test_path_follower_matches_existing_pure_pursuit_on_straight_and_curve():
    t = np.linspace(-.2, 1.5, 180)
    for wall in (np.column_stack((np.linspace(-.1, 2., 200), np.full(200, .425))),
                 np.column_stack((.575*np.sin(t), 1.-.575*np.cos(t)))):
        path, _ = local_path(wall, 'left')
        cfg = FollowConfig(max_speed_mps=2.)
        old_speed, old_steer, _ = pursuit_command(wall, 'left', .5, wheelbase=.135, max_steering=.37, max_speed=2.)
        new_speed, new_steer, _ = pursue_path(path, .5, cfg, .2)
        assert new_speed == pytest.approx(old_speed)
        assert new_steer == pytest.approx(old_steer)


@pytest.mark.parametrize('key,value', [('memory_ttl_s', .31), ('max_speed_mps', 3.),
    ('headway_s', 0.), ('measurement_padding_m', float('nan')), ('confirmations', 1)])
def test_bad_configuration_rejected(key, value):
    with pytest.raises(ValueError): FollowConfig(**{key: value})


def test_partial_scan_geometry_cannot_claim_a_complete_scene():
    raw = scan()
    raw.angle_increment = math.pi/500
    with pytest.raises(MotionUnavailable, match='scan_invalid'):
        motion_scene(raw, history(), .7, timing_verified=True)


def test_out_of_range_point_remains_unknown_not_free_space():
    raw = scan()
    raw.ranges[250] = 50.
    new = motion_scene(raw, history(), .7, timing_verified=True)
    assert new.rays[250].status == 'unknown' and new.rays[250].hit_xy_m is None


def test_existing_node_command_is_unchanged_when_opt_in_is_off():
    from arena_autonomy.local_pursuit import LocalPursuit
    h = history()
    node = SimpleNamespace(follow=None, motion=SimpleNamespace(history=h, last_meta={}), side='left',
        settings={'wheelbase_m': .135, 'max_steering_angle_rad': .37, 'max_speed_mps': 2.5,
                  'lateral_acceleration_limit_mps2': 3.5, 'target_wall_distance_m': .425})
    points = np.asarray(scene(.5).occupied_endpoints_xy_m)
    actual = LocalPursuit.command(node, points)
    expected = pursuit_command(points, 'left', 0., .135, .37, 2.5, 3.5, .425, .2)
    assert actual == expected


def test_node_g1_input_failure_cannot_fall_back_to_r0():
    from arena_autonomy.local_pursuit import LocalPursuit
    h = history()
    node = SimpleNamespace(follow=LocalFollow(), scan=scan(), last_steering=0., side='left',
        motion=SimpleNamespace(history=h, last_meta={'reference_stamp_s': .7}, counts={'resets': 0}, verified=False),
        settings={'control_rate_hz': 50.}, rear_blind_half_angle=30.)
    speed, _, info = LocalPursuit.command(node, np.asarray(scene(.7).occupied_endpoints_xy_m))
    assert speed == 0 and info['reason'] == 'follow_input_stop'


def test_boundary_memory_does_not_turn_same_source_into_fresh_observation():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    _, _, first = ctl.command(scene(.8), h, .5, 'left')
    _, _, repeated = ctl.command(scene(.82, source=first['source_first_s']), h, .5, 'left')
    assert first['path_source_s'] == pytest.approx(.7)
    assert repeated['path_source_s'] == pytest.approx(.7)
    assert repeated['boundary_age_s'] > first['boundary_age_s']


def test_missing_motion_for_memory_is_hold_not_static_assumption():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    speed, _, meta = ctl.command(scene(.8, missing_wall=True), MotionHistory(), .5, 'left')
    assert speed == 0 and meta['reason'] == 'follow_motion_waiting'


def test_saved_gazebo_short_occluders_use_current_wall_support_not_false_free_space():
    import json
    from pathlib import Path
    sample = Path(__file__).resolve().parents[1]/'artifacts/validation/2026-10-05/g1_gazebo_solo_v1/last_scan.json'
    raw = json.loads(sample.read_text())
    observed = scene_from_scan(raw['ranges'], angle_min=raw['angle_min'],
        angle_increment=raw['angle_increment'], range_min=raw['range_min'], range_max=raw['range_max'],
        acquired_at_s=[.7]*len(raw['ranges']), t_ref_s=.8)
    points = np.asarray(observed.occupied_endpoints_xy_m)
    _, old_info = local_path(points, 'left')
    assert old_info['boundary_observed_ahead_m'] < .3
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    speed, _, detail = ctl.command(observed, h, 0., 'left')
    assert speed > 0 and detail['fresh_boundary_accepted']
    assert detail['boundary_selection'] == 'observed_history_support'
    assert detail['occupied_points'] == len(points)  # 기둥도 보호 검사에는 전부 남는다.
    assert detail['path_source_s'] == .7
    assert detail['boundary_candidate_ahead_m'] >= .65


def test_boundary_support_cannot_fill_large_missing_wall_segment():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    for now in (.8, 1.):
        observed = scene(now)
        rays = tuple(ray for ray in observed.rays if not (ray.hit_xy_m[1] > 0 and .19 < ray.hit_xy_m[0] < .65))
        speed, _, detail = ctl.command(replace(observed, rays=rays), h, .5, 'left')
    assert speed == 0 and detail['reason'] == 'follow_boundary_unconfirmed'


def test_boundary_support_retains_observed_points_beyond_previous_path_end():
    path = np.column_stack((np.linspace(-.25, 1., 36), np.zeros(36)))
    points = np.array([[.3, .425], [.3, .2], [1.5, .5]])
    kept = boundary_support(points, path, 'left', FollowConfig())
    np.testing.assert_allclose(kept, points[[0, 2]])


def test_saved_corner_requires_bilateral_agreement_before_using_long_outer_wall():
    import json
    from pathlib import Path
    raw = json.loads((Path(__file__).resolve().parents[1]/
        'artifacts/validation/2026-10-05/g1_gazebo_solo_v2/last_scan.json').read_text())
    ctl, h = LocalFollow(), history()
    for now in (.4, .5, .6, .7):
        observed = scene_from_scan(raw['ranges'], angle_min=raw['angle_min'], angle_increment=raw['angle_increment'],
            range_min=raw['range_min'], range_max=raw['range_max'],
            acquired_at_s=[now-.1]*len(raw['ranges']), t_ref_s=now)
        speed, _, detail = ctl.command(observed, h, 0., 'left')
    assert speed > 0 and detail['path_wall'] == 'right'
    assert detail['bilateral_alternative_confirmed']
    assert detail['boundary_candidate_ahead_m'] >= .65
    assert detail['occupied_points'] == len(observed.occupied_endpoints_xy_m)


def test_long_opposite_wall_with_inconsistent_short_preferred_boundary_is_not_a_fallback():
    ctl, h = LocalFollow(), history()
    for now in (.4, .5, .6, .7):
        raw = scene(now)
        # 양쪽 벽에서 만든 중심 경로가 서로 다른 회랑이다. 왼쪽 짧은 반환이
        # 허용 앵커 오차 안에 있더라도 형상 일치 없이는 오른쪽만 따르지 않는다.
        rays = tuple(replace(ray, hit_xy_m=(ray.hit_xy_m[0], .315))
            if ray.hit_xy_m[1] > 0 else ray for ray in raw.rays
            if ray.hit_xy_m[1] < 0 or ray.hit_xy_m[0] < .38)
        speed, _, detail = ctl.command(replace(raw, rays=rays), h, 0., 'left')
    assert speed == 0 and detail['behavior'] == 'HOLD'


def test_single_negative_motor_count_keeps_current_boundary_and_occupancy():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    one_count_speed = -math.pi*.065/(28*15)/.01
    speed, _, detail = ctl.command(scene(.8, obstacle_x=1.), h, one_count_speed, 'left')
    assert speed == 0 and detail['reason'] == 'follow_reverse_motion'
    assert detail['measured_speed_mps'] == one_count_speed and detail['reverse_motion']
    assert detail['fresh_boundary_accepted'] and ctl.path is not None
    assert detail['path_source_s'] == scene(.8).source_first_s
    assert detail['occupied_points'] == 300 and detail['unfiltered_occupancy']
    assert detail['swept_clearance_m'] < 1.
    assert ctl.last_source == detail['source_first_s'] and ctl.release_count == 0


def test_continuous_signed_reverse_updates_observations_but_never_drives_forward():
    ctl, h = LocalFollow(), history(-.05)
    settle(ctl, h)
    for now in (.8, .9, 1., 1.1, 1.2):
        speed, _, detail = ctl.command(scene(now, obstacle_x=1.), h, -.05, 'left')
        assert speed == 0 and detail['reason'] == 'follow_reverse_motion'
        assert detail['fresh_boundary_accepted'] and not detail['boundary_from_memory']
        assert detail['occupied_points'] == 300 and ctl.release_count == 0
    assert ctl.path_source == scene(1.2).source_first_s


def test_reverse_recovery_requires_two_distinct_fresh_release_observations():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    ctl.command(scene(.8), h, -.049, 'left')
    speed, _, detail = ctl.command(scene(.9), h, 0., 'left')
    assert speed == 0 and detail['reason'] == 'follow_release_wait' and ctl.release_count == 1
    same_source = detail['source_first_s']
    speed, _, _ = ctl.command(scene(.92, source=same_source), h, 0., 'left')
    assert speed == 0 and ctl.release_count == 1
    speed, _, detail = ctl.command(scene(1.), h, 0., 'left')
    assert speed > 0 and detail['behavior'] == 'CRUISE'


@pytest.mark.parametrize('speed', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_speed_remains_invalid_and_resets_boundary(speed):
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    output, _, detail = ctl.command(scene(.8), h, speed, 'left')
    assert output == 0 and detail['reason'] == 'follow_invalid_state'
    assert ctl.path is None and ctl.last_source is None


@pytest.mark.parametrize('change', ['epoch', 'reference_rewind', 'source_rewind'])
def test_reverse_does_not_preserve_path_across_invalid_time_or_epoch(change):
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    raw = scene(.8, epoch=1) if change == 'epoch' else scene(.5 if change == 'reference_rewind' else .8, source=.3)
    output, _, detail = ctl.command(raw, h, -.049, 'left')
    assert output == 0 and detail['reason'] in ('follow_boundary_unconfirmed', 'follow_scene_stale')
    assert ctl.path is None


def test_reverse_with_stale_scene_resets_and_does_not_refresh_source_time():
    ctl, h = LocalFollow(), history()
    settle(ctl, h)
    output, _, detail = ctl.command(scene(1., source=.6), h, -.049, 'left')
    assert output == 0 and detail['reason'] == 'follow_scene_stale'
    assert ctl.path is None and ctl.last_source is None


def test_reverse_with_occlusion_can_only_use_unexpired_observed_boundary_memory():
    ctl, h = LocalFollow(), history(-.05)
    settle(ctl, h)
    source = ctl.path_source
    output, _, detail = ctl.command(scene(.8, obstacle_x=.4, missing_wall=True), h, -.049, 'left')
    assert output == 0 and detail['reason'] == 'follow_reverse_motion'
    assert detail['boundary_from_memory'] and detail['path_source_s'] == source
    assert detail['occupied_points'] == 160 and detail['swept_clearance_m'] <= .2
    output, _, detail = ctl.command(scene(1., obstacle_x=.4, missing_wall=True), h, -.049, 'left')
    assert output == 0 and detail['reason'] == 'follow_boundary_unconfirmed' and ctl.path is None


def test_signed_reverse_history_is_integrated_without_zero_clamping():
    count_displacement = math.pi*.065/(28*15)
    h = history(-count_displacement/.01)
    moved = transport_points([[1., 0.]], h, .4, .41)
    np.testing.assert_allclose(moved, [[1.+count_displacement, 0.]], atol=1e-12)
    assert moved[0, 0] > 1.


# --- G1-b 재개: 벽 바로 앞 얇은 물체·무기억 재획득(10/7 등속 seed 0 실패) ---
FAILURE_WINDOW = (Path(__file__).resolve().parents[1]/
                  'artifacts/validation/2026-10-07/g1b_resume_analysis_v1/failure_window.json')


def box(cx, cy, sx, sy):
    x0, x1, y0, y1 = cx-sx/2, cx+sx/2, cy-sy/2, cy+sy/2
    corners = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return [(corners[i], corners[(i+1) % 4]) for i in range(4)]


def corridor(left=.425, right=-.425, start=-3., end=4.):
    return [((start, left), (end, left)), ((start, right), (end, right)), ((end, left), (end, right))]


def post(x, inset=.07, side='left'):
    # 2.5 cm 기둥의 벽 쪽 면이 벽면에서 inset만큼 도로 쪽에 있다. tl_post는 약 0.078 m.
    sign = 1 if side == 'left' else -1
    return box(x, sign*(.425-inset-.0125), .025, .025)


def cast_scene(now, segments, *, source=None, epoch=0, samples=500):
    """정지 차체의 이상적 2D 광선 투사. Gazebo 렌더링 재현이 아닌 보존 기하 근사다."""
    segments = np.asarray(segments, dtype=float)
    start, edge = segments[:, 0], segments[:, 1]-segments[:, 0]
    offset = start-np.array([.06, 0.])
    angles = -math.pi+np.arange(samples)*math.tau/samples
    direction = np.column_stack((np.cos(angles), np.sin(angles)))
    denominator = direction[:, None, 0]*edge[None, :, 1]-direction[:, None, 1]*edge[None, :, 0]
    with np.errstate(divide='ignore', invalid='ignore'):
        distance = (offset[None, :, 0]*edge[None, :, 1]-offset[None, :, 1]*edge[None, :, 0])/denominator
        along = (offset[None, :, 0]*direction[:, None, 1]-offset[None, :, 1]*direction[:, None, 0])/denominator
    hit = (np.abs(denominator) > 1e-12) & (distance > .05) & (along >= 0) & (along <= 1)
    ranges = np.where(hit, distance, np.inf).min(axis=1)
    first = now-.1 if source is None else source
    return scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=math.tau/samples, range_min=.05,
                           range_max=12., acquired_at_s=[first]*samples, t_ref_s=now, source_epoch=epoch)


def settle_cast(controller, h, world=None, times=(.4, .5, .6, .7), speed=0.):
    return [controller.command(cast_scene(now, corridor() if world is None else world), h, speed, 'left')
            for now in times]


def test_protrusion_filter_uses_median_offset_and_keeps_points_beyond_reference_ends():
    path = np.column_stack((np.linspace(-.25, 1., 36), np.zeros(36)))
    wall = np.column_stack((np.linspace(0., .9, 40), np.full(40, .425)))
    points = np.vstack((wall, [[.35, .349], [1.4, .30], [-.6, .30]]))
    kept = drop_protrusions(points, path, 'left', FollowConfig())
    np.testing.assert_allclose(kept, np.vstack((wall, [[1.4, .30], [-.6, .30]])))
    # 기준 경로가 일정하게 5 cm 치우쳐도 벽은 남고 벽 앞 7.6 cm 모서리만 빠진다.
    shifted = drop_protrusions(points-[0., .05], path, 'left', FollowConfig())
    np.testing.assert_allclose(shifted, np.vstack((wall, [[1.4, .30], [-.6, .30]]))-[0., .05])


@pytest.mark.parametrize('inset,x', [(.06, .36), (.07, .30), (.07, .42), (.075, .36)])
def test_thin_object_just_inside_wall_does_not_break_observed_boundary_memory(inset, x):
    ctl, h = LocalFollow(), history()
    settle_cast(ctl, h)
    observed = cast_scene(.8, corridor()+post(x, inset))
    points = np.asarray(observed.occupied_endpoints_xy_m)
    selected = boundary_support(points, transport_points(ctl.path, h, ctl.path_ref, .8), 'left', ctl.config)
    _, first_pass = local_path(selected, 'left')
    assert first_pass['boundary_observed_ahead_m'] < .65  # 기존 1차 지원 선택은 이 모서리로 끊긴다.
    speed, _, detail = ctl.command(observed, h, 0., 'left')
    assert speed > 0 and detail['fresh_boundary_accepted'] and not detail['boundary_from_memory']
    assert detail['boundary_selection'] == 'observed_history_support'
    assert detail['boundary_protrusions_removed'] >= 1 and detail['boundary_candidate_ahead_m'] >= .65
    # 경계 지원에서 뺀 기둥도 점유 검사에는 모두 남는다.
    assert detail['occupied_points'] == len(points) and detail['unfiltered_occupancy']
    assert np.any((np.abs(points[:, 0]-x) <= .0126) & (points[:, 1] < .425-inset+1e-6) & (points[:, 1] > .2))


@pytest.mark.parametrize('x', [.25, .30, .36, .42])
def test_cold_reacquisition_reselects_preferred_wall_from_long_opposite_wall(x):
    world = corridor()+post(x)
    points = np.asarray(cast_scene(.4, world).occupied_endpoints_xy_m)
    _, preferred = local_path(points, 'left')
    assert preferred['boundary_observed_ahead_m'] < .65  # 기존 무기억 재획득은 여기서 막힌다.
    ctl, h = LocalFollow(), history()
    results = settle_cast(ctl, h, world, times=(.4,))
    first = results[0][2]
    assert results[0][0] == 0 and first['reason'] == 'follow_boundary_unconfirmed'
    assert first['boundary_selection'] == 'bilateral_reference_support' and ctl.pending_count == 1
    results += settle_cast(ctl, h, world, times=(.5, .6, .7))
    assert results[1][0] == 0 and results[1][2]['reason'] == 'follow_release_wait'
    speed, _, detail = results[-1]
    assert speed > 0 and detail['path_wall'] == 'left' and not detail['wall_fallback']
    assert detail['occupied_points'] == len(points)


def test_cold_reacquisition_with_thin_object_inside_opposite_wall_is_symmetric():
    ctl, h = LocalFollow(), history()
    results = settle_cast(ctl, h, corridor()+post(.36, side='right'))
    assert results[0][2]['bilateral_reference'] == 'opposite_support'
    assert results[-1][0] > 0 and results[-1][2]['path_wall'] == 'left'


@pytest.mark.parametrize('length', [.15, .30])
def test_long_object_shadow_is_not_bridged_into_a_boundary(length):
    # 벽 앞 6.5 cm의 긴 물체 뒤 벽은 연결 한도보다 길게 가려진다. 2차 정제도 그 그림자를 잇지 않는다.
    world = corridor()+box(.25+length/2, .425-.065-.0125, length, .025)
    ctl, h = LocalFollow(), history()
    assert all(speed == 0 for speed, _, _ in settle_cast(ctl, h, world))
    ctl, h = LocalFollow(), history()
    settle_cast(ctl, h)
    _, _, detail = ctl.command(cast_scene(.8, world), h, 0., 'left')
    assert detail['boundary_from_memory'] and not detail['fresh_boundary_accepted']
    speed, _, detail = ctl.command(cast_scene(.9, world), h, 0., 'left')
    assert speed == 0 and detail['reason'] == 'follow_boundary_unconfirmed' and ctl.path is None


def branch(divergence):
    return [((-3, .425), (4, .425)), ((-3, -.425), (divergence, -.425)), ((divergence, -.425), (2., -1.2)),
            ((2., -1.2), (4, -1.5)), ((4, .425), (4, -1.5))]


@pytest.mark.parametrize('world', [corridor(right=-.32)+post(.36), branch(.3)+post(.36), branch(.5)+post(.36)],
                         ids=['width_mismatch', 'branch_0.3', 'branch_0.5'])
def test_mismatched_or_diverging_opposite_wall_cannot_reselect_preferred_wall(world):
    ctl, h = LocalFollow(), history()
    results = settle_cast(ctl, h, world)
    assert all(speed == 0 and detail['behavior'] == 'HOLD' for speed, _, detail in results)
    assert ctl.path is None


def test_bilateral_reference_that_disagrees_with_unexpired_memory_is_not_adopted():
    # 두 벽은 서로 맞지만 기억 경계와 10 cm 어긋난 장면. 기억은 TTL 안에서만 쓰고 새 경계로 바꾸지 않는다.
    shifted = corridor(left=.525, right=-.325)+post(.36, side='left')
    ctl, h = LocalFollow(), history()
    settle_cast(ctl, h)
    _, _, detail = ctl.command(cast_scene(.8, shifted), h, 0., 'left')
    assert detail['boundary_from_memory'] and not detail['fresh_boundary_accepted']
    speed, _, detail = ctl.command(cast_scene(.9, shifted), h, 0., 'left')
    assert speed == 0 and detail['reason'] == 'follow_boundary_unconfirmed' and ctl.path is None


def test_bilateral_reference_support_needs_two_distinct_fresh_scans_and_resets_on_epoch():
    world = corridor()+post(.36)
    ctl, h = LocalFollow(), history()
    for now in (.4, .42, .44):
        speed, _, _ = ctl.command(cast_scene(now, world, source=.3), h, 0., 'left')
        assert speed == 0
    assert ctl.pending_count == 1 and ctl.path is None
    speed, _, _ = ctl.command(cast_scene(.5, world, epoch=1), h, 0., 'left')
    assert speed == 0 and ctl.pending_count == 1 and ctl.path is None
    speed, _, detail = ctl.command(cast_scene(.9, world, source=.5, epoch=1), h, 0., 'left')
    assert speed == 0 and detail['reason'] == 'follow_scene_stale' and ctl.path is None


def test_reacquired_boundary_still_holds_for_signed_reverse_and_unobserved_front():
    world = corridor()+post(.36)
    ctl, h = LocalFollow(), history()
    settle_cast(ctl, h, world)
    speed, _, detail = ctl.command(cast_scene(.8, world), h, -.0486, 'left')
    assert speed == 0 and detail['reason'] == 'follow_reverse_motion' and detail['fresh_boundary_accepted']
    speed, _, detail = ctl.command(cast_scene(.9, world), h, 0., 'left')
    assert speed == 0 and detail['reason'] == 'follow_release_wait'
    base = cast_scene(1., world)
    unknown = SensorRay((.06, 0.), None, base.source_first_s, 'unknown', 'laser_frame', 0, 999, (1., 0.))
    speed, _, detail = ctl.command(replace(base, rays=base.rays+(unknown,)), h, 0., 'left')
    assert speed == 0 and detail['reason'] == 'follow_front_unobserved' and ctl.path is None


def failure_window(name='failure_window.json'):
    return json.loads((FAILURE_WINDOW.parent/name).read_text())


def preserved_scene(row, h, now):
    nanos = round(row['stamp_s']*1e9)
    raw = SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame', stamp=SimpleNamespace(
        sec=nanos//10**9, nanosec=nanos % 10**9)), ranges=[math.inf if r is None else r for r in row['ranges']],
        angle_min=row['angle_min'], angle_increment=row['angle_increment'], range_min=.05, range_max=12.,
        time_increment=row['time_increment'])
    return motion_scene(mask_scan(raw), h, now, timing_verified=True)


def replay_preserved_moving(data, critical):
    """보존 scan·MCU 평균 바퀴 속도를 scan마다 한 번 공급한다. critical scan의 기억 지원 1차 결과도 돌려준다."""
    h = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
    feedback = data['feedback_capture_s_mean_wheel_mps']
    for stamp in np.arange(feedback[0][0], feedback[-1][0]+.05, .005):
        h.add_gyro(float(stamp), 0.)  # 자이로 미기록. 직선 출발 구간의 0 근사.
    rows = {round(r['stamp_s'], 3): r for r in data['scans']}
    ctl, used, details, first_pass, points = LocalFollow(), 0, {}, None, None
    for stamp in data['moving_scan_stamps_s']:
        now = round(stamp+.118, 3)
        while used < len(feedback) and feedback[used][0] <= now-.005:
            h.add_mean_wheel(feedback[used][0], feedback[used][1]/.0325)
            used += 1
        observed = preserved_scene(rows[stamp], h, now)
        if stamp == critical:
            points = np.asarray(observed.occupied_endpoints_xy_m)
            carried = transport_points(ctl.path, h, ctl.path_ref, now)
            first_pass = local_path(boundary_support(points, carried, 'left', ctl.config), 'left')[1]
        details[stamp] = ctl.command(observed, h, h.wheels.values[-1], 'left', 0., .02)
    return ctl, details, first_pass, points


def test_preserved_slow_lead_post_corner_keeps_boundary_instead_of_ttl_expiry():
    ctl, details, first_pass, points = replay_preserved_moving(failure_window(), 7.802)
    assert first_pass['boundary_observed_ahead_m'] < .65  # 보존 실패의 기둥 모서리 1차 선택
    corner = details[7.802][2]
    assert corner['boundary_selection'] == 'observed_history_support' and corner['fresh_boundary_accepted']
    assert corner['boundary_protrusions_removed'] >= 1 and corner['boundary_candidate_ahead_m'] >= .65
    assert corner['occupied_points'] == len(points)
    speed, _, last = details[7.902]
    assert speed > 0 and last['reason'] == 'local_follow' and last['boundary_age_s'] < ctl.config.memory_ttl_s


def test_preserved_post_behind_memory_path_start_uses_same_scan_opposite_wall_with_memory_agreement():
    # 1차 수정 실행 8.62초: 직전 경로가 기둥 앞에서 시작해 기둥 점이 기억 범위 밖으로 남았다.
    ctl, details, first_pass, points = replay_preserved_moving(failure_window('transient_window.json'), 8.502)
    assert first_pass['reason'] == 'wall_shape_uncertain'
    speed, _, detail = details[8.502]
    assert speed > 0 and detail['boundary_selection'] == 'bilateral_reference_support'
    assert detail['fresh_boundary_accepted'] and not detail['boundary_from_memory']
    assert detail['path_wall'] == 'left' and detail['occupied_points'] == len(points)
    speed, _, detail = details[8.602]
    assert speed > 0 and detail['reason'] == 'local_follow' and detail['boundary_age_s'] < ctl.config.memory_ttl_s


def test_preserved_stationary_post_geometry_reacquires_with_bilateral_support():
    data = failure_window()
    rows = {round(r['stamp_s'], 3): r for r in data['scans']}
    h = history(end=10.2)
    ctl, results = LocalFollow(), []
    for stamp in data['stationary_scan_stamps_s']:
        observed = preserved_scene(rows[stamp], h, round(stamp+.118, 3))
        points = np.asarray(observed.occupied_endpoints_xy_m)
        _, left = local_path(points, 'left')
        _, right = local_path(points, 'right')
        assert left['boundary_observed_ahead_m'] < .65 <= right['boundary_observed_ahead_m']
        results.append(ctl.command(observed, h, 0., 'left', 0., .02))
    assert results[0][2]['boundary_selection'] == 'bilateral_reference_support'
    assert results[0][0] == 0 and results[1][0] == 0
    speed, _, detail = results[-1]
    assert speed > 0 and detail['path_wall'] == 'left' and detail['occupied_points'] == len(points)
