"""현재 관측 벽으로 짧은 경로를 재획득하는 저속 연결.

기존 경로 계산/명령/점유 함수를 재사용한다. ROS, STOP, guard 소유권은 없다.
실행 프로필이나 기억 경로를 새로 만들지 않는다.
"""
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import LocalFollow, pursue_path
from arena_autonomy.local_path import local_path, swept_limit, braking_speed
from arena_autonomy.observation_support import source_valid
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.race_runtime_policy.current_route_priority import enabled, observed_command
from arena_autonomy.vehicle_marker_occupancy import additional_clearance


PASS_GEOMETRY_REASONS = frozenset((
    'entry_guide_current_outer_wall_unavailable',
    'entry_guide_current_wall_disagrees',
    'entry_guide_current_wall_too_short',
    'entry_guide_wall_reanchor_rejected',
    'entry_guide_extension_disagrees',
    'entry_guide_current_prefix_short',
))


def current_wall_candidate(scene, speed, side, previous, dt, cfg, *, cap_mps=.10):
    """현재 선호 벽으로 재획득한다. 반환값은 독립 guard 허가가 아니다."""
    if (not enabled(cfg) or side not in ('left', 'right') or not source_valid(scene)
            or not all(math.isfinite(v) for v in (speed, previous, dt, cap_mps))
            or speed < 0 or not 0 < dt <= .2 or not 0 < cap_mps <= cfg.max_speed_mps
            or abs(previous) > cfg.max_steering_rad):
        return None
    age = scene.t_ref_s-scene.source_first_s
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    if len(points) < 30:
        return None
    bearings = [math.atan2(ray.direction_xy[1], ray.direction_xy[0])
                for ray in scene.rays if ray.status == 'return']
    try:
        # 저속 재획득은 차체 길이만큼 관측한 선호 벽을 사용할 수 있다.
        # 경로 자체의 전방0.25m·형상·제동/원시점유 검사는 뒤에서 그대로 적용한다.
        # 짧다는 이유로 반대 벽으로 전환해 갈림길 방향을 바꾸지 않는다.
        minimum_ahead = min(.28, cfg.length_m) if cap_mps <= .10 else .28
        path, boundary = local_path(points, side, cfg.wall_offset_m,
            horizon=cfg.boundary_min_ahead_m,
            piecewise_corner=cfg.piecewise_wall_trial,
            minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),
            narrow_occluder_bridge=cfg.narrow_occluder_bridge,
            minimum_ahead_m=minimum_ahead, bearings_rad=bearings)
        if path is None:
            return None
        cap, desired, remaining = pursue_path(path, speed, cfg, age, speed_cap=cap_mps)
        steering = float(np.clip(desired, previous-3*dt, previous+3*dt))
        fixed_padding = .025+cfg.measurement_padding_m
        # 고정 축별 여유는 유지한다. 취득 뒤 물체 이동의 속력 상계만 원판으로
        # 적용해 x/y 모두 최대 이동한 것으로 간주하는 사각 대각 과팽창을 피한다.
        motion_radius = per_ray_padding(scene, 0., cfg.object_motion_bound_mps)
        padding = fixed_padding+motion_radius
        radial_motion = cap_mps <= .10
        # 저속 재획득은 현재 반환점+고정 오차 여유로 진행한다. 숨은 상대 이동의
        # 최악 상계 보장은 포기하지만 원시 점유/정지 간격/제동/독립 guard는 유지한다.
        observed_crawl = radial_motion and speed <= .20
        sweep_options = ({'margin': fixed_padding} if observed_crawl else
                         {'margin': fixed_padding, 'radial_margin': motion_radius}
                         if radial_motion else {'margin': padding})
        # 기존 LocalFollow와 동일하게 실제 이번 각도 및 직전 각도를 모두 검사한다.
        sweeps = [swept_limit(points, angle, cfg.wheelbase_m, horizon=2.5,
            **sweep_options,
            length=cfg.length_m, width=cfg.width_m)
            for angle in (steering, previous)]
        clearance = additional_clearance(min(sweeps), steering, previous, cfg, horizon=2.5)
        available = max(0., clearance-cfg.standstill_gap_m)
        target = min(cap, braking_speed(available, age), available/cfg.headway_s)
        if target <= 0:
            return None
    except (ValueError, MotionUnavailable):
        return None
    detail = dict(reason='local_follow', behavior='FOLLOW',
        boundary_selection='current_wall_reacquire_trial', fresh_boundary_accepted=True,
        path_source_s=scene.source_first_s, boundary_age_s=age, boundary_from_memory=False,
        path_wall=side, wall_fallback=False, path_points=path.tolist(),
        observed_path_ahead_m=remaining, swept_clearance_m=clearance,
        unfiltered_occupancy=True, occupied_points=len(points),
        occupancy_timing_mode='per_ray_candidate',
        occupancy_padding_m=float(fixed_padding if observed_crawl else np.max(padding)),
        occupancy_motion_shape=('fixed_box_low_speed_observation' if observed_crawl else
                                'fixed_box_plus_euclidean_motion_ball' if radial_motion else 'axis_padding'),
        occupancy_motion_bound_applied=not observed_crawl,
        residual_unobserved_object_motion_risk=observed_crawl,
        target_speed_before_hold_mps=target,
        current_wall_reacquire=dict(cap_mps=cap_mps,
            minimum_boundary_ahead_m=minimum_ahead,
            current_boundary_ahead_m=boundary.get('boundary_observed_ahead_m'),
            anchor_error_m=boundary.get('path_anchor_error_m'),
            fixed_padding_m=fixed_padding, motion_radius_max_m=float(np.max(motion_radius)),
            prior_anchor_agreement_required=False, path_memory_committed=False,
            actual_steering_rate_checked=True, current_angle_clearance_m=sweeps[0],
            previous_angle_clearance_m=sweeps[1], unknown_filled=False,
            output_requires_independent_guard=True))
    return float(target), steering, detail


def connect_moving_pass(adapter, baseline, scene, history, speed, side, previous, dt,
                        raw_scan, *, cap_mps=.10):
    """active MOVING_PASS의 기존 0 분기에서 호출하는 현재 경로 연결.

    기존 guide 정상 출력은 불변이다. 기억 guide가 계속 불일치해도 기존
    handoff_follow가 새 현재 정상 경로를 획득하면 기존 reset/복원 계약으로
    FOLLOW에 인계한다. 저속 후보 자체는 기억이나 지속 허가로 이월하지 않는다.
    """
    detail = baseline[2]
    entry = (detail.get('moving_pass') or {}).get('entry') or {}
    if (baseline[0] != 0 or not adapter.active or raw_scan is None
            or detail.get('handoff_state') != 'MOVING_PASS'
            or detail.get('reason') != 'moving_pass_current_path_unavailable'
            or entry.get('reason') not in PASS_GEOMETRY_REASONS):
        return baseline
    adapter._ensure_follow_handoff()
    prepared, monitor = adapter.handoff_follow, adapter.handoff_monitor
    normal = prepared.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
    monitor.update(scene, history)
    evidence = normal[2]
    if evidence.get('reason') in ('follow_partial_scan_scene_mismatch',
            'follow_partial_scan_missing', 'follow_scene_stale', 'follow_invalid_state'):
        return baseline
    ready = (normal[0] > 0 and evidence.get('reason') == 'local_follow'
            and evidence.get('current_wall_reacquire') is None
            and evidence.get('boundary_from_memory') is False
            and evidence.get('path_source_s') == scene.source_first_s)
    deferred = None
    if ready and adapter.owner.execution.follow_entry_space_enabled:
        # 기존 읽기 전용 reserve 판정을 준비된 monitor로 한 번 확인한다.
        # 미래 우회용 여유 0은 실제 현재 경로의 충돌 0과 구분한다. reset을
        # 미루고 아래 helper를 재검사할 뿐 전진을 금지하거나 여유를 조작하지 않는다.
        old_monitor = adapter.owner.monitor
        adapter.owner.monitor = monitor
        try:
            follow_target, reserve = adapter.owner._follow_entry_space_current(
                scene, normal, previous, normal[0])
        finally:
            adapter.owner.monitor = old_monitor
        if follow_target == 0 and reserve.get('reason') == 'observed_entry_distance_reserve':
            ready = False
            deferred = dict(reason='future_entry_reserve_only', normal_speed_mps=normal[0],
                reserve=reserve, old_pass_not_reset=True, current_candidate_rechecked=True)
    if ready:
        # 기존 centered 인계와 동일하다. 반환값은 새 owner의 최종 판정 그대로다.
        adapter.owner.reset()
        adapter.owner.normal, adapter.owner.monitor = prepared, monitor
        output = adapter.owner._base_command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        return output[0], output[1], {**output[2], 'moving_pass': dict(
            reason='returned_to_current_follow_after_wall_reacquire', completed=False,
            original_geometry_reason=entry['reason'], trial_cap_retained=False)}
    candidate = current_wall_candidate(scene, speed, side, previous, dt,
                                       adapter.owner.config, cap_mps=cap_mps)
    if candidate is None:
        return baseline
    # LocalFollow.command가 사용하는 원시 입력 무결성/현재 전방 정책을 공유한다.
    candidate = observed_command(adapter.owner.config, scene, candidate, raw_scan,
                                 history.config.lidar_yaw_rad)
    if candidate is None or candidate[0] <= 0:
        return baseline
    target, steering, evidence = candidate
    return target, steering, {**detail, 'reason': 'moving_pass_current_wall_reacquire_trial',
        'behavior': 'MOVING_PASS', 'current_wall_reacquire': evidence,
        'reacquire_original_reason': entry['reason'],
        'current_follow_handoff_deferred': deferred,
        'output_requires_independent_guard': True,
        'execution_delivery': 'existing_nonretained_plan_boundary'}
