"""오프라인 후보 진단 사본(운영 미연결): observed_fresh_v2 평행 경로 평가 + 팽창 기준 시각 반사실 인자.
10/29 오프라인 v4의 parallel_speed.py에서 목표 상한 경계만 고쳤다. 측정속도는 자르지 않고
기존 물리/점유/관측/횡변위 기준을 유지하며, 호출자는 새 경로의 이동 영역 관측을 별도로 검사한다.
"""
import math
from types import SimpleNamespace
import numbers
import numpy as np
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_path import braking_speed, swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.parallel_lane_trial import lateral_excursion

def proposal(scene, baseline, speed, previous, dt, config, target_cap=.35, padding_reference_s=None):
    from arena_autonomy.local_follow import pursue_path
    result = {'eligible': False, 'reason': 'baseline_unavailable', 'unknown_filled': False,
              'hidden_rear_risk_unresolved': True, 'straight_trial_only': True}
    detail = baseline[2]
    if detail.get('reason') not in ('local_follow', 'follow_gap_hold', 'follow_release_wait',
                                    'local_follow_corridor_candidate'):
        return result
    partial = detail.get('partial_observation') or {}
    if partial.get('reason') != 'fully_observed' or partial.get('allowed') is not True:
        return {**result, 'reason': 'partial_observation_outside_trial'}
    if (not all(math.isfinite(v) for v in (speed, previous, dt)) or speed < 0.
            or not 0 < dt <= .2 or abs(previous) > config.max_steering_rad+1e-9):
        return {**result, 'reason': 'invalid_motion'}
    age = scene.t_ref_s-scene.source_first_s
    if (isinstance(target_cap, bool) or not isinstance(target_cap, numbers.Real)
            or not math.isfinite(target_cap) or not target_cap > 0):
        return {**result, 'reason': 'invalid_target_cap'}
    # 명령 상한은 설정 최대속도와 기존 경로 cap의 min이다. 측정속도 유효성 검사와 분리한다.
    target_cap = min(float(target_cap), config.max_speed_mps)
    from arena_autonomy.m3_execution_trial import envelope_distance, ExecutionConfig
    if not math.isfinite(age) or not 0 <= age < .3:
        return {**result, 'reason': 'invalid_motion_age'}
    measured_distance = envelope_distance(speed, age, dt, ExecutionConfig())
    # 기존 수치 스윕의 horizon=2.5를넘는큰값은배열생성전에거부한다.
    if measured_distance > 2.5:
        return {**result, 'reason': 'measured_envelope_outside_swept_horizon',
                'measured_distance_m': measured_distance}
    path = np.asarray(detail.get('path_points', []), dtype=float).reshape(-1, 2)
    if len(path) < 6 or not np.all(np.isfinite(path)) or not 0 <= age < .3:
        return result
    selected = path[(path[:, 0] >= -.05) & (path[:, 0] <= .8)]
    if len(selected) < 6 or np.ptp(selected[:, 0]) < .4:
        return {**result, 'reason': 'short_reference'}
    slope, intercept = np.linalg.lstsq(np.column_stack([selected[:, 0], np.ones(len(selected))]),
                                      selected[:, 1], rcond=None)[0]
    residual = float(np.max(abs(selected[:, 1]-(slope*selected[:, 0]+intercept))))
    # selector에서 복귀 중 차체가 기울어도 직선 도로의 곡률이 생긴 것은 아니다.
    # 기존 직선 전용 비교는 5도 제한을 유지한다. selector에서는 실제 원호 횡이동으로 제한한다.
    if (abs(math.atan(slope)) > math.radians(5) and not config.pass_release_trial) or residual > .01:
        return {**result, 'reason': 'reference_not_straight', 'fit_residual_m': residual}
    shift = float(intercept-slope*config.wheelbase_m/2)
    parallel = path.copy()
    parallel[:, 1] -= shift
    try:
        cap, steering, remaining = pursue_path(parallel, speed, config, age)
    except MotionUnavailable as error:
        return {**result, 'reason': str(error)}
    steering = float(np.clip(steering, previous-3*dt, previous+3*dt))
    target = min(target_cap, cap)
    lateral = lateral_excursion(max(speed, target), age, steering, previous, dt, config)
    lateral_limited = False
    if lateral > .01 and config.pass_release_trial:
        # 선택기 fallback 전용: 같은 1 cm 조건을 지키는 속도까지 낮춘다.
        # 현재 속도 자체가 크면 감속 완료를 가정하지 않고 기존 HOLD를 유지한다.
        if lateral_excursion(speed, age, steering, previous, dt, config) <= .01:
            lo, hi = 0., target
            for _ in range(20):
                mid = (lo+hi)/2
                if lateral_excursion(max(speed,mid),age,steering,previous,dt,config) <= .01:
                    lo = mid
                else:
                    hi = mid
            target = lo
            lateral = lateral_excursion(max(speed,target),age,steering,previous,dt,config)
            lateral_limited = True
    result.update(shift_from_reference_m=shift, steering_rad=steering,
                  maximum_body_corner_lateral_m=lateral, fit_residual_m=residual,
                  observed_ahead_m=remaining, path_points=parallel.tolist())
    if config.pass_release_trial:
        result.update(lateral_speed_limited=lateral_limited, lateral_speed_cap_mps=target)
    if lateral > .01:
        return {**result, 'reason': 'lateral_motion_not_small'}
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    if not len(points) or not np.all(np.isfinite(points)):
        return {**result, 'reason': 'occupancy_invalid'}
    # 진단 반사실 전용: 같은 raw 광선·원래 취득 시각에서 팽창 나이만 지정 시각 기준으로 계산한다.
    padding_scene = scene if padding_reference_s is None else SimpleNamespace(t_ref_s=padding_reference_s, rays=scene.rays)
    padding = per_ray_padding(padding_scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)
    clearance = min(swept_limit(points, angle, config.wheelbase_m, horizon=2.5, margin=padding,
                               length=config.length_m, width=config.width_m)
                    for angle in (steering, previous))
    available = max(0., clearance-config.standstill_gap_m)
    target = min(target, braking_speed(available, age), available/config.headway_s)
    result.update(clearance_m=clearance, target_speed_mps=target,
                  unfiltered_occupancy=True, occupied_points=len(points),
                  maximum_padding_m=float(np.max(padding)))
    if clearance <= config.hold_clearance_m:
        return {**result, 'reason': 'unchanged_gap_hold'}
    if speed > config.max_speed_mps:
        # 목표상한초과측정은클리핑하지않는다. 현재속도의정지공간/관측을별도로증명한다.
        from arena_autonomy.pass_release_trial import motion_region, region_blocked, lateral_config, SelectorConfig
        result['measured_above_target_cap'] = True
        result['measured_speed_mps'] = speed
        result['measured_distance_m'] = measured_distance
        if measured_distance > available:
            return {**result, 'reason': 'measured_stop_space_insufficient'}
        region = motion_region(scene, speed, target, steering, previous, dt, config, lateral_config(config))
        blocked = region_blocked(region, SelectorConfig())
        result['measured_region'] = region
        if blocked is not None:
            return {**result, 'reason': 'measured_region_unobserved', 'measured_region_blocked': blocked}
    return {**result, 'eligible': True, 'reason': 'parallel_current_position'}
