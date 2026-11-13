"""통과 뒤 코너 정지에서 작은 직진/제자리 조향을 비교하는 관측 후보.

현재 G1 경로가 있고 측방 통과가 확인된 경우에만 비교한다. 중앙 복귀
곡선의 미관측을 없애지 않고, 별도 명령의 현재 관측·점유·노면을 검사한다.
모터축/자이로 정지는 차체 정지나 서보 실측의 증명이 아니다.
"""
import math
import numpy as np

from arena_autonomy import m3_execution_trial as mx
from arena_autonomy.local_path import braking_speed
from arena_autonomy.observation_support import source_valid
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.pass_release_trial import motion_region, region_blocked, NORMAL_BASELINE


def compare_released_straight(owner, scene, history, baseline, selected, speed, previous, dt):
    """순수 비교 함수. selected를 바꾸지 않으며 연결자가 사용할 후보와 근거만 돌려준다."""
    cfg = owner.follow_config
    b_speed, _, detail = baseline
    partial = detail.get('partial_observation') or {}
    if not (cfg.traffic_recovery_trial and cfg.piecewise_wall_trial
            and cfg.guard_curve_trial and owner.mode == 'RELEASED'
            and not any(s['present'] for s in owner.sides.values())
            and selected[0] <= 0
            and selected[2].get('selector_reason') == 'large_motion_region_unobserved'
            and detail.get('reason') in NORMAL_BASELINE and b_speed > 0
            and detail.get('boundary_from_memory') is False
            and detail.get('path_source_s') == scene.source_first_s
            and partial.get('allowed') is True and partial.get('reason') == 'fully_observed'
            and source_valid(scene)):
        return None
    evidence = dict(accepted=False, reason='outside_low_speed_scope',
        unknown_filled=False, body_standstill_verified=False,
        baseline_steering_rad=baseline[1], measured_speed_mps=speed)
    if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in (speed, previous, dt, b_speed))
            or not 0 <= speed <= owner.config.guard_speed_cap_mps or not 0 < dt <= .2
            or abs(previous) > cfg.max_steering_rad):
        return evidence
    path = detail.get('path_points')
    if path is None or len(path) < 2:
        return dict(evidence, reason='current_path_unavailable')
    age = scene.t_ref_s-scene.source_first_s
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    padding = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
    execution = mx.ExecutionConfig()

    def command(steer, old):
        clearance = min(mx.swept_limit(points, angle, cfg.wheelbase_m, horizon=2.5,
            margin=padding, length=cfg.length_m, width=cfg.width_m) for angle in (steer, old))
        available = max(0., clearance-cfg.standstill_gap_m)
        target = min(.2, b_speed, braking_speed(available, age), available/cfg.headway_s)
        region = motion_region(scene, speed, target, steer, old, dt, cfg, owner.lat)
        blocked = region_blocked(region, owner.config)
        geometry = mx.arc_checks(scene, [], path, (steer, old), region['distance_m'], owner.lat, execution)
        # 지역 관측은 기존 selector의 현재 광선·제한된 장착 가림 검사다.
        # arc_checks의 노면/측방 기하와 종방향 제동/간격도 함께 유지한다.
        accepted = target > 0 and blocked is None and geometry['geometry_ok']
        return dict(accepted=accepted, target_mps=target, steering_rad=steer,
            previous_steering_rad=old, clearance_m=clearance, motion_region=region,
            blocked=blocked, geometry=geometry['geometry'], geometry_ok=geometry['geometry_ok'])

    steer = float(np.clip(0., previous-3*dt, previous+3*dt))
    actual = command(steer, previous)
    evidence.update(reason='current_straight_command_rejected', command=actual)
    if actual['accepted']:
        return dict(evidence, accepted=True, reason='released_current_straight',
                    speed_mps=actual['target_mps'], steering_rad=steer)
    if abs(speed) > mx.STOPPED_SPEED_MPS or steer == previous or history is None:
        return evidence
    quiet = mx.presteer_motion_quiet(history, scene.t_ref_s)
    evidence['motion_quiet'] = quiet
    if not quiet:
        return evidence
    # 0각 가정은 조향만 바꿀 가치가 있는지 보는 용도다. 양수 명령에는
    # 위 actual의 실제 직전 발행각을 사용하며 가정한 각도를 넘기지 않는다.
    settled = command(0., 0.)
    evidence['settled_hypothesis'] = settled
    if not settled['accepted']:
        return evidence
    support = mx.presteer_support(scene, [], (previous, steer), owner.lat, execution)
    evidence['wheel_support'] = support
    if not support['supported']:
        return evidence
    return dict(evidence, accepted=True, reason='released_straight_presteer',
                speed_mps=0., steering_rad=steer)
