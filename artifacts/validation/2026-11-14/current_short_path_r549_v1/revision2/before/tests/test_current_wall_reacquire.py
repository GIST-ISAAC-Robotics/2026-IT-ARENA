"""현재 벽 재획득의 공통 제어 연결. 합성 센서/상태 시험이며 주행 증명은 아니다."""
from dataclasses import replace
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest

from arena_autonomy import current_wall_reacquire as reacquire
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
from arena_autonomy.bypass_entry_trial import entry_path, straight_reference
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import scene_from_scan


# r513/r521에서 실제 선택한 설정. artifact/ROS/원시 trace를 읽지 않는다.
STRUCTURES = json.dumps(dict(sectors_deg=[
    [168.14344658741254, 3.026954460405105], [-167.94189267100495, 3.0286330838020525],
    [123.152834481159, 7.259987043244574], [-123.15283448115898, 7.259987043244585]],
    range_limits_m=[[.1546521350185877, .1791643871601953],
                    [.1547781919363293, .17928620601910233],
                    [.05794965910877819, .07998629412484672],
                    [.0579496591087782, .07998629412484673]]))
CFG = FollowConfig(signed_motion_policy='forward_drive', piecewise_wall_trial=True,
    oblique_gap_trial=True, partial_observation_policy='bounded_local_v4',
    known_structure_policy='structure_aware_v1', known_structure_sectors_json=STRUCTURES,
    parallel_hold_trial=True, current_lane_preference_trial=True,
    current_lane_headroom_mps=.061225042014404256, side_departure_trial=True,
    pass_release_trial=True, guard_curve_trial=True, guard_observation_policy='selected_r181',
    traffic_recovery_trial=True, released_rear_sweep_trial=True, released_body_gap_trial=True,
    static_side_release_trial=True, traffic_path_recovery_trial=True, corridor_fallback_enabled=True,
    short_boundary_prefix_enabled=True, boundary_evidence_mode='corner_distance_candidate',
    occupancy_timing_mode='per_ray_candidate', wheelbase_m=.128, length_m=.207,
    wall_offset_m=.505, road_half_m=.305, max_steering_rad=.4578436017036438,
    open_route_speed_mps=1.5, standing_wall_reacquire=True, narrow_occluder_bridge=True,
    junction_continuity=True, expiry_route_reference_trial=True,
    lateral_acceleration_mps2=3., track_m=.1239, wheel_width_m=.0261, crop_edge_component_trial=True)
EXECUTION = HandoffExecutionConfig(
    bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold',
                                steerable_transition=True, rear_wall_witness=True),
    preparation_policy='observed_entry_space_v1', persistence_window_policy='trim_before_compare_v1',
    follow_entry_space_enabled=True, moving_follow_reserve=True, closing_follow_reserve=True,
    low_motion_reserve=True, moving_pace_floor=True, observed_forward_pace=True,
    continuous_forward_pace=True, moving_pass_trial=True, moving_pass_profile='medium_v1',
    reacquire_policy='observed_rejoin_v2', entry_policy='corridor_local_v1',
    preparation_hold_policy='bounded_hold_v1', initial_reference_policy='ready_directional_pair_v2',
    motion_pause_policy='bounded_motion_pause_v1', relative_yield_policy='observed_progress_yield_v3',
    entry_motion_gate=True)


def history():
    result = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675,
                                       lidar_x_m=.06, history_s=5.))
    for stamp in np.arange(0., 3.0001, .01):
        result.add_mean_wheel(float(stamp), 0.)
        result.add_gyro(float(stamp), 0.)
    return result


def determinant(a, b):
    """로컬 ray 교차 연산. NumPy/공통 함수에는 patch를 설치하지 않는다."""
    return a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]


def sensor(now=.7, shift=0., end=2., curved=False, obstacle=False, closed=False, actor=None,
           left_end=None):
    # 임의 시험 벽과 actor는 교차거리 생성에만 사용하고 제어기에는 label을 전달하지 않는다.
    xs = np.linspace(-2., end, 160)
    center = shift + (.05*np.sin(6*xs) if curved else xs*0)
    segments = []
    for sign in (-1., 1.):
        points = np.c_[xs, center+sign*CFG.wall_offset_m]
        if sign > 0 and left_end is not None:
            points = points[xs <= left_end]
        segments.extend(zip(points[:-1], points[1:]))
    if obstacle:
        segments.append((np.array([.18, -.22]), np.array([.18, .22])))
    if actor is not None:
        corners = np.array([[actor-.1, -.06], [actor+.1, -.06], [actor+.1, .06],
                            [actor-.1, .06], [actor-.1, -.06]])
        segments.extend(zip(corners[:-1], corners[1:]))
    if closed:
        for index in (0, -1):
            segments.append((np.array([xs[index], center[index]-CFG.wall_offset_m]),
                             np.array([xs[index], center[index]+CFG.wall_offset_m])))
    origin = np.array([.06, 0.])
    starts = np.array([s[0] for s in segments])
    deltas = np.array([s[1]-s[0] for s in segments])
    ranges = []
    for angle in -math.pi+np.arange(500)*math.tau/500:
        direction = np.array([math.cos(angle), math.sin(angle)])
        denom = determinant(direction, deltas)
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = determinant(starts-origin, deltas)/denom
            proportion = determinant(starts-origin, direction)/denom
        valid = ((distance >= .05) & (distance <= 12.) & (proportion >= 0.)
                 & (proportion <= 1.) & (abs(denom) > 1e-12))
        ranges.append(float(np.min(distance[valid])) if np.any(valid) else math.inf)
    first_sec = int(now-.1)
    first_nanosec = round((now-.1-first_sec)*1e9)
    first = first_sec+first_nanosec*1e-9
    scene = scene_from_scan(ranges, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., acquired_at_s=[first+i*.0002 for i in range(500)],
        t_ref_s=now, source_epoch=0, rear_half_angle_deg=0.)
    raw = SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., time_increment=.0002,
        header=SimpleNamespace(frame_id='laser_frame',
            stamp=SimpleNamespace(sec=first_sec, nanosec=first_nanosec)))
    return scene, raw


def baseline_command(owner, scene):
    return owner._baseline_command(scene, history(), 0., 'left', 0., .1, allow_partial=True)


def legacy_command(monkeypatch, owner, scene):
    with monkeypatch.context() as scoped:
        scoped.setattr(reacquire, 'current_wall_candidate', lambda *args, **kwargs: None)
        return baseline_command(owner, scene)


def active_owner_with_old_wall(old_shift=0., actor=1.2):
    owner = BypassHandoffFollow(CFG, EXECUTION, allow_simulation_trial=True)
    adapter = owner.moving_pass
    old, _ = sensor(.5, shift=old_shift, closed=True, actor=actor)
    _, reference = straight_reference(old, CFG)
    wall = reference['walls']['left']
    fit = (wall['slope'], wall['intercept_m'], *wall['observed_x_m'])
    guide = adapter.guide
    guide.path = entry_path(fit, .16, CFG, guide.entry, 1.2)
    guide.side = 'left'
    guide.offset, guide.transition, guide.phase = .16, 1.2, 'entry'
    guide.reference_s, guide.last_source = old.t_ref_s, old.source_first_s
    guide.epoch, guide.wall_confirmations = old.source_epoch, 2
    x = np.asarray(wall['observed_x_m'])
    guide.wall_line = np.c_[x, wall['slope']*x+wall['intercept_m']]
    points = np.asarray(old.occupied_endpoints_xy_m)
    target = points[(points[:, 0] >= actor-.11) & (points[:, 0] <= actor+.11)
                    & (abs(points[:, 1]) <= .061)]
    assert len(target) >= 3
    adapter.tracker.begin(old, target, reference['walls'])
    adapter.active = True
    return owner


def moving_command(owner, now, shift=.08, actor=1.2, obstacle=False):
    scene, raw = sensor(now, shift=shift, closed=True, actor=actor, obstacle=obstacle)
    return owner.command(scene, history(), 0., 'left', 0., .1, raw_scan=raw)


def test_shifted_wall_reacquires_current_points_without_old_anchor(monkeypatch):
    old, current = LocalFollow(CFG), LocalFollow(CFG)
    for now in (.4, .5, .6):
        scene, _ = sensor(now)
        legacy_command(monkeypatch, old, scene)
        baseline_command(current, scene)
    shifted, _ = sensor(1.1, shift=.16)
    before = legacy_command(monkeypatch, old, shifted)
    after = baseline_command(current, shifted)
    assert before[0] == 0 and before[2]['reason'] == 'follow_boundary_unconfirmed'
    assert 0 < after[0] <= .10
    assert after[2]['current_wall_reacquire']['anchor_error_m'] > CFG.boundary_anchor_error_m
    assert after[2]['occupied_points'] == len(shifted.occupied_endpoints_xy_m)
    assert after[2]['boundary_from_memory'] is False


def test_short_curved_current_path_positive_and_real_obstacle_zero(monkeypatch):
    free, _ = sensor(shift=.08, end=.50, curved=True)
    blocked, _ = sensor(shift=.08, end=.50, curved=True, obstacle=True)
    assert legacy_command(monkeypatch, LocalFollow(CFG), free)[0] == 0
    after = baseline_command(LocalFollow(CFG), free)
    assert 0 < after[0] <= .10
    assert after[2]['current_wall_reacquire']['current_boundary_ahead_m'] < CFG.boundary_min_ahead_m
    assert baseline_command(LocalFollow(CFG), blocked)[0] == 0


def test_short_preferred_wall_uses_current_opposite_wall_with_measured_steering():
    cfg = replace(CFG, max_steering_rad=.28475436568260193)
    scene, _ = sensor(left_end=.20)
    out = reacquire.current_wall_candidate(scene, .1, 'left', 0., .1, cfg)
    assert out is not None and 0 < out[0] <= .1
    assert out[2]['wall_fallback'] and out[2]['path_wall'] == 'right'
    assert out[2]['current_wall_reacquire']['preferred_wall_failure'] == 'wall_segment_short'
    assert out[2]['path_source_s'] == scene.source_first_s
    assert out[2]['occupied_points'] == len(scene.occupied_endpoints_xy_m)
    assert abs(out[1]) <= cfg.max_steering_rad


def test_opposite_reacquisition_preserves_actual_front_obstacle_stop():
    scene, _ = sensor(left_end=.20, obstacle=True)
    assert reacquire.current_wall_candidate(scene, .1, 'left', 0., .1, CFG) is None


def test_no_wall_is_not_free_path():
    scene, _ = sensor(end=.15)
    assert reacquire.current_wall_candidate(scene, .1, 'left', 0., .1, CFG) is None


def test_occupied_preferred_path_does_not_trigger_opposite_path_search(monkeypatch):
    scene, _ = sensor()
    calls = []
    original = reacquire.local_path
    def record(points, side, *args, **kwargs):
        calls.append(side)
        return original(points, side, *args, **kwargs)
    monkeypatch.setattr(reacquire, 'local_path', record)
    monkeypatch.setattr(reacquire, 'swept_limit', lambda *args, **kwargs: 0.)
    assert reacquire.current_wall_candidate(scene, .1, 'left', 0., .1, CFG) is None
    assert calls == ['left']


def test_normal_positive_and_actual_occupied_stop_unchanged(monkeypatch):
    old, current = LocalFollow(CFG), LocalFollow(CFG)
    for now in (.4, .5, .6, .7):
        scene, _ = sensor(now)
        before = legacy_command(monkeypatch, old, scene)
        after = baseline_command(current, scene)
        if before[0] > 0:
            assert before == after
    assert before[0] > .10
    blocked, _ = sensor(.8, obstacle=True)
    assert legacy_command(monkeypatch, old, blocked) == baseline_command(current, blocked)


@pytest.mark.parametrize('invalid', ['stale', 'empty'])
def test_invalid_current_source_cannot_reacquire(invalid):
    scene, _ = sensor()
    scene = (replace(scene, t_ref_s=1.1) if invalid == 'stale' else
             replace(scene, rays=(), source_first_s=None, source_last_s=None))
    assert reacquire.current_wall_candidate(scene, 0., 'left', 0., .1, CFG) is None
    assert baseline_command(LocalFollow(CFG), scene)[0] == 0


def test_previous_steering_sweep_still_rejects_candidate(monkeypatch):
    scene, _ = sensor(shift=.16)
    original = reacquire.swept_limit
    calls = []
    def record(points, angle, *args, **kwargs):
        calls.append(angle)
        return 0. if angle == -.2 else original(points, angle, *args, **kwargs)
    monkeypatch.setattr(reacquire, 'swept_limit', record)
    assert reacquire.current_wall_candidate(scene, 0., 'left', -.2, .1, CFG) is None
    assert -.2 in calls and len(calls) == 2


def test_real_moving_pass_failure_directly_connects_current_reacquisition(monkeypatch):
    old, current = active_owner_with_old_wall(), active_owner_with_old_wall()
    with monkeypatch.context() as scoped:
        scoped.setattr(reacquire, 'connect_moving_pass', lambda adapter, baseline, *args, **kwargs: baseline)
        before = moving_command(old, .7, shift=.16)
    after = moving_command(current, .7, shift=.16)
    assert before[0] == 0 and before[2]['reason'] == 'moving_pass_current_path_unavailable'
    assert after[0] == pytest.approx(.10)
    assert after[2]['handoff_state'] == 'MOVING_PASS'
    assert after[2]['reacquire_original_reason'] in ('entry_guide_current_outer_wall_unavailable',
                                                   'entry_guide_current_wall_disagrees')


def test_prepared_low_speed_fallback_is_not_normal_follow_handoff():
    owner = active_owner_with_old_wall(old_shift=-.20)
    for now in (.7, .8):
        result = moving_command(owner, now, shift=.16)
        assert result[0] == pytest.approx(.10)
        assert owner.moving_pass.active and result[2]['handoff_state'] == 'MOVING_PASS'
    prepared = owner.moving_pass.handoff_follow
    scene, raw = sensor(.8, shift=.16, closed=True, actor=1.2)
    candidate = prepared.command(scene, history(), 0., 'left', 0., .1, raw_scan=raw)
    assert candidate[0] > 0 and candidate[2].get('current_wall_reacquire') is not None


def test_raw_scene_timestamp_mismatch_does_not_authorize_reacquisition():
    owner = active_owner_with_old_wall()
    scene, raw = sensor(.7, shift=.16, closed=True, actor=1.2)
    raw.header.stamp.nanosec += 10
    result = owner.command(scene, history(), 0., 'left', 0., .1, raw_scan=raw)
    assert result[0] == 0


def test_future_entry_reserve_preserves_low_speed_until_normal_follow_is_available():
    owner = active_owner_with_old_wall(old_shift=-.20)
    for now, shift in ((.7,.16),(.8,.16),(.9,.08),(1.,.08),(1.1,.08),(1.2,.08)):
        result = moving_command(owner, now, shift=shift)
        assert result[0] == pytest.approx(.10) and owner.moving_pass.active
    deferred = result[2]['current_follow_handoff_deferred']
    assert deferred['normal_speed_mps'] > .10 and deferred['reserve']['cap_mps'] == 0
    # 별도의 새 센서 입력으로 관측 간격이 넓어졌다. 자차 폐루프 이동 증명은 아니다.
    result = moving_command(owner, 1.3, actor=1.65)
    assert result[0] > .10 and result[2]['handoff_state'] == 'FOLLOW'
    assert not owner.moving_pass.active
    assert result[2]['moving_pass']['trial_cap_retained'] is False


def test_deferred_handoff_still_stops_for_new_actual_occupied_returns():
    owner = active_owner_with_old_wall(old_shift=-.20)
    for now, shift in ((.7,.16),(.8,.16),(.9,.08),(1.,.08),(1.1,.08)):
        assert moving_command(owner, now, shift=shift)[0] == pytest.approx(.10)
    assert moving_command(owner, 1.2, obstacle=True)[0] == 0
