"""직선의 현재 횡위치 유지 비교. 기본 OFF이며 코스 주행/후방 안전 인증이 아니다.

관측 경로의 기울기는 유지하고 현재 후륜축을 지나도록 평행 이동한다.
모든 현재 점유와 기존 시차 팽창·간격을 유지한다. 커브/결측/입력 실패 시
기존 중앙 복귀로 우회하지 않고 대기한다. 상대 차량 정답은 사용하지 않는다.
"""
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_path import braking_speed, swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding


def lateral_excursion(speed, age, steering, previous, dt, config):
    """현재/직전 일정 조향의 반응+제동 동안 차체 모서리 횡변위의 극값.

    2 m/s² 제동 근사이며 실측 제동 보증은 아니다. 샘플 간 극값도 계산한다.
    """
    distance = speed*(age+.1+dt)+speed*speed/4
    maximum = 0.
    for angle in (steering, previous):
        k = math.tan(angle)/config.wheelbase_m
        if abs(k) < 1e-9:
            continue
        end = distance*k
        lo, hi = sorted((0., end))
        for x in (-config.length_m/2, config.length_m/2):
            for y in (-config.width_m/2, config.width_m/2):
                a, b = 1/k-y, config.wheelbase_m/2+x
                critical = math.atan2(-b, a)
                angles = [0., end] + [critical+n*math.pi for n in range(
                    math.ceil((lo-critical)/math.pi), math.floor((hi-critical)/math.pi)+1)]
                maximum = max(maximum, *(abs(a*2*math.sin(t/2)**2+b*math.sin(t)) for t in angles))
    return maximum


def proposal(scene, baseline, speed, previous, dt, config):
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
    if (not all(math.isfinite(v) for v in (speed, previous, dt)) or not 0 <= speed <= 1.
            or not 0 < dt <= .2 or abs(previous) > config.max_steering_rad+1e-9):
        return {**result, 'reason': 'invalid_motion'}
    age = scene.t_ref_s-scene.source_first_s
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
    target = min(.35, cap)
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
    padding = per_ray_padding(scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)
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
    return {**result, 'eligible': True, 'reason': 'parallel_current_position'}


class ParallelLaneTrial:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.held = True
        self.confirmations = 0
        self.last_source = self.last_ref = self.epoch = None

    def apply(self, scene, baseline, speed, previous, dt):
        source, now = scene.source_first_s, scene.t_ref_s
        hold_steer = float(np.clip(previous, -self.config.max_steering_rad,
                                   self.config.max_steering_rad)) if math.isfinite(previous) else 0.
        if source is None or not math.isfinite(source) or not math.isfinite(now):
            self.reset()
            return 0., hold_steer, {**baseline[2], 'reason': 'parallel_hold_invalid_time',
                                    'behavior': 'HOLD', 'parallel_hold_selected': False}
        if (self.epoch != scene.source_epoch or
                (self.last_ref is not None and now < self.last_ref) or
                (self.last_source is not None and source < self.last_source)):
            self.reset()
        fresh = self.last_source is None or source > self.last_source
        self.epoch, self.last_source, self.last_ref = scene.source_epoch, source, now
        result = proposal(scene, baseline, speed, previous, dt, self.config)
        detail = {**baseline[2], 'baseline_reason': baseline[2].get('reason'),
                  'parallel_hold_trial': result, 'parallel_hold_selected': False,
                  'reason': 'parallel_hold_'+result['reason'], 'behavior': 'HOLD'}
        if not result['eligible']:
            self.held, self.confirmations = True, 0
            return 0., hold_steer, detail
        steering = result['steering_rad']
        if self.held:
            if result['clearance_m'] < self.config.resume_clearance_m:
                self.confirmations = 0
            elif fresh:
                self.confirmations += 1
            if self.confirmations < self.config.confirmations:
                return 0., steering, {**detail, 'reason': 'parallel_hold_release_wait',
                                      'parallel_release_count': self.confirmations}
            self.held = False
        detail.update(parallel_hold_selected=True, reason='parallel_hold_follow', behavior='FOLLOW',
                      path_points=result['path_points'], swept_clearance_m=result['clearance_m'],
                      target_speed_before_hold_mps=result['target_speed_mps'])
        return result['target_speed_mps'], steering, detail
