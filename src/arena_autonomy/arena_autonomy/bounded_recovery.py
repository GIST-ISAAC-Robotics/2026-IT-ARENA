"""미연결 직진 관측 회복 후보. 시간/모터축 이동 한 번만 허용하는 시험 계약.

센서 조용함은 실제 차체 정지 증명이 아니며 모터축 적분도 지면 이동이 아니다.
물체 미관측을 자유로 치환하거나 보호층/정지 허가를 대체하지 않는다.
"""
import math

import numpy as np

from arena_autonomy.linear_observation import RECOVERY_SPEED_MPS, linear_observation


PULSE_TIME_S = .25
PULSE_MOTOR_DISTANCE_M = .05
SETTLE_WINDOW_S = .2


def quiet_feedback(history, now):
    if not math.isfinite(now) or now < SETTLE_WINDOW_S:
        return False
    for series, limit in ((history.wheels, .05), (history.gyro, .05)):
        stamps, values = series.available(now)
        first = max(0, int(np.searchsorted(stamps, now-SETTLE_WINDOW_S, side='right'))-1)
        stamps, values = stamps[first:], values[first:]
        if (len(stamps) < 2 or stamps[0] > now-SETTLE_WINDOW_S+1e-9
                or now-stamps[-1] > history.config.max_extrapolation_s+1e-9
                or np.max(np.diff(stamps)) > history.config.max_input_gap_s+1e-9
                or np.any(np.abs(values) > limit)):
            return False
    return True


def path_prefix_supported(path, distance, tolerance=.08):
    """관측 경로의 보간 구간에서만 직진 prefix를 비교. 자유 공간 외삽 아님."""
    p = np.asarray(path, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 2 or not np.all(np.isfinite(p))
            or not math.isfinite(distance) or not .05 < distance <= .4
            or not math.isfinite(tolerance) or not 0 < tolerance <= .08):
        return False
    p = p[(p[:, 0] >= 0) & (p[:, 0] <= distance+.16)]
    if (len(p) < 5 or p[0, 0] > .1 or p[-1, 0] < distance
            or np.any(np.diff(p[:, 0]) <= 0)
            or np.max(np.linalg.norm(np.diff(p, axis=0), axis=1)) > .08):
        return False
    # 구간 끝과 원래 절점을 모두 검사하여 표본 사이 선형 최대치도 포함한다.
    x = np.r_[p[0, 0], p[(p[:, 0] >= p[0, 0]) & (p[:, 0] < distance), 0], distance]
    return bool(np.max(np.abs(np.interp(x, p[:, 0], p[:, 1]))) <= tolerance)


def recovery_evidence(scene, history, speed, previous_steering, path, config):
    result = dict(eligible=False, reason='recovery_not_settled', actual_body_stop_verified=False,
                  motor_distance_is_ground_distance=False, pulse_time_s=PULSE_TIME_S,
                  pulse_motor_distance_m=PULSE_MOTOR_DISTANCE_M)
    if (not all(math.isfinite(x) for x in (speed, previous_steering))
            or not 0 <= speed <= .05 or abs(previous_steering) > .001
            or not quiet_feedback(history, scene.t_ref_s)):
        return result
    observation = linear_observation(scene, speed, length=config.length_m, width=config.width_m,
        standstill_gap=config.standstill_gap_m, measurement_padding=config.measurement_padding_m,
        object_motion_bound=config.object_motion_bound_mps)
    result['observation'] = observation
    if not observation['supported']:
        return {**result, 'reason': observation['reason']}
    # .16 m 정지 간격은 여전히 전체 관측 요구에 남긴다. 경로 비교는 명령 pulse와
    # 기존 .06 m 여유 + 반응/제동 이동까지다. 이 둘을 같은 길이로 혼동하지 않는다.
    age = scene.t_ref_s-scene.source_first_s
    v = RECOVERY_SPEED_MPS
    motion_prefix = PULSE_MOTOR_DISTANCE_M+.06+v*(age+.10)+v*v/4
    result['path_prefix_distance_m'] = motion_prefix
    result['path_supported'] = path_prefix_supported(path, motion_prefix, config.boundary_agreement_m)
    return {**result, 'eligible': result['path_supported'],
            'reason': 'bounded_recovery_evidence' if result['path_supported'] else 'recovery_path_disagrees'}


class PulseBudget:
    """한 번만 허용하는 pulse. fresh scan/순간 허가/시간 경과로 재무장하지 않는다.

    reset은 외부의 명시적인 새 정상 주행 episode 판정용이다. 운용 미연결.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.started = self.last = None
        self.motor_distance = 0.
        self.spent = False

    def step(self, now, speed, allowed):
        if (not math.isfinite(now) or now < 0 or not math.isfinite(speed) or speed < 0
                or (self.last is not None and (now < self.last or now-self.last > .1))):
            self.spent = True
        if self.spent:
            return False
        if self.started is None:
            if not allowed:
                return False
            self.started = self.last = now
        else:
            # 양 끝 중 큰 속도: 단순 끝점 적분의 과소평가를 피하나 지면 이동 상계는 아니다.
            self.motor_distance += max(speed, self.previous_speed)*(now-self.last)
            self.last = now
        self.previous_speed = speed
        if (not allowed or now-self.started >= PULSE_TIME_S-1e-9
                or self.motor_distance >= PULSE_MOTOR_DISTANCE_M-1e-9
                or speed > RECOVERY_SPEED_MPS+.05):
            self.spent = True
            return False
        return True
