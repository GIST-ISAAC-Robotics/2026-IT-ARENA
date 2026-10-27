"""저장 입력용 현재 횡위치 유지 후보. 운영 연결/후방 빈 공간 증명은 아니다.

기존 관측 경로가 직선에 가까울 때만 후륜축을 지나는 평행 경로로 옮긴다.
현재 점유/점별 팽창/제동/앞차 간격은 바꾸지 않는다. 이 함수의 양수 후보는
확정 주행 명령이 아니며 fresh 재개·독립 보호·실제 추종 검증이 별도로 필요하다.
"""
import math
import numpy as np

from arena_autonomy.local_follow import pursue_path
from arena_autonomy.local_path import swept_limit, braking_speed
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.occupancy_timing import per_ray_padding
from probe_saved import lateral_motion


def candidate(scene, baseline, speed, previous, dt, cfg):
    result = {'eligible': False, 'reason': 'baseline_unavailable', 'unknown_filled': False,
              'runtime_authorized': False, 'hidden_rear_risk_unresolved': True}
    detail = baseline[2]
    if detail.get('reason') not in ('local_follow', 'follow_gap_hold', 'follow_release_wait',
                                    'local_follow_corridor_candidate'):
        return result
    partial = detail.get('partial_observation')
    if partial is not None and partial.get('reason') != 'fully_observed':
        return {**result, 'reason': 'partial_observation_outside_comparison'}
    if not math.isfinite(speed) or speed < 0 or not 0 < dt <= .2:
        return {**result, 'reason': 'invalid_motion'}
    age = scene.t_ref_s-scene.source_first_s
    path = np.asarray(detail.get('path_points', []), dtype=float).reshape(-1, 2)
    if len(path) < 6 or not np.all(np.isfinite(path)) or not 0 <= age < .3:
        return result
    selected = path[(path[:,0] >= -.05) & (path[:,0] <= .8)]
    if len(selected) < 6 or np.ptp(selected[:,0]) < .4:
        return {**result, 'reason': 'short_reference'}
    slope, intercept = np.linalg.lstsq(np.column_stack([selected[:,0], np.ones(len(selected))]),
                                      selected[:,1], rcond=None)[0]
    residual = float(np.max(abs(selected[:,1]-(slope*selected[:,0]+intercept))))
    if abs(math.atan(slope)) > math.radians(5) or residual > .01:
        return {**result, 'reason': 'reference_not_straight', 'fit_residual_m': residual}
    # 목표를 중앙으로 당기지 않고 현재 후륜축을 지나는 같은 기울기의 평행 경로로 바꾼다.
    shift = float(intercept-slope*cfg.wheelbase_m/2)
    proposal = path.copy()
    proposal[:,1] -= shift
    try:
        cap, steering, remaining = pursue_path(proposal, speed, cfg, age)
    except MotionUnavailable as error:
        return {**result, 'reason': str(error)}
    steering = float(np.clip(steering, previous-3*dt, previous+3*dt))
    target = min(.35, cap)
    # 각도만 작다고 작은 이동으로 보지 않는다. 현재 운동과 직전 조향을 포함한다.
    lateral = lateral_motion(max(abs(speed), target), age, steering, previous, dt, cfg.wheelbase_m)
    result.update(shift_from_reference_m=shift, steering_rad=steering,
                  maximum_body_corner_lateral_m=lateral, fit_residual_m=residual,
                  observed_ahead_m=remaining)
    if lateral > .01:
        return {**result, 'reason': 'lateral_motion_not_small'}
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    padding = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
    clearance = min(swept_limit(points, angle, cfg.wheelbase_m, horizon=2.5, margin=padding,
                               length=cfg.length_m, width=cfg.width_m)
                    for angle in (steering, previous))
    available = max(0., clearance-cfg.standstill_gap_m)
    target = min(target, braking_speed(available, age), available/cfg.headway_s)
    result.update(clearance_m=clearance, target_speed_mps=target,
                  unfiltered_occupancy=True, occupied_points=len(points),
                  maximum_padding_m=float(np.max(padding)))
    if clearance <= cfg.hold_clearance_m:
        return {**result, 'reason': 'unchanged_gap_hold'}
    return {**result, 'eligible': True, 'reason': 'parallel_current_position_candidate'}
