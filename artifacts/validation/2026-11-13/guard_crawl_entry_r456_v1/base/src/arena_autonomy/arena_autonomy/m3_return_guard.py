"""기본 경로 복귀 뒤에도 실제 출력 외곽을 평가하는 미연결 진단.

출력을 수정하거나 발행하지 않는다. 상태 장부의 해제와 이번 명령의 관측/간격 검사를
구분하며, 기존 M3의 반응·제동·측방·두 바퀴 노면 기준을 그대로 사용한다.
sampled_sweep_ok는 연속 안전·실물 제동·동적 사각 안전의 인증이 아니다.
"""
import math

import numpy as np

from arena_autonomy.m3_execution_trial import arc_checks, envelope_distance
from arena_autonomy.observation_support import source_valid


def assess_return_command(scene, frames, reference_path, *, target_speed, steering,
                          measured_speed, previous_steering, control_dt, cfg, execution):
    """실제 목표/이전 조향각으로 한 주기+반응+제동 외곽만 평가한다.

    원래 경로를 다시 추종해 다른 명령을 만들지 않는다. 목표 0이라도 측정 속도가
    남으면 제동 외곽은 사라지지 않는다. 반대로 STOP 자체를 거부/지연하는 API도 아니다.
    """
    result = {'sampled_sweep_ok': False, 'reason': None, 'envelope': None,
              'runtime_authorized': False, 'output_changed': False,
              'continuous_sweep_certified': False, 'hidden_future_risk_unresolved': True,
              'stop_command_must_not_be_delayed': True}
    values = (target_speed, steering, measured_speed, previous_steering, control_dt)
    if (any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)
            or min(target_speed, measured_speed) < 0 or not 0 < control_dt <= .2
            or max(abs(steering), abs(previous_steering)) > cfg.max_steering_rad):
        return {**result, 'reason': 'return_command_input_invalid'}
    if not source_valid(scene):
        return {**result, 'reason': 'return_observation_invalid'}
    try:
        path = np.asarray(reference_path, dtype=float)
        if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2 or not np.all(np.isfinite(path)):
            return {**result, 'reason': 'return_reference_path_invalid'}
        age = scene.t_ref_s - scene.source_first_s
        bound = max(measured_speed, target_speed)
        distance = envelope_distance(bound, age, control_dt, execution)
        checked = arc_checks(scene, frames, path, (steering, previous_steering), distance, cfg, execution)
    except (ValueError, TypeError, IndexError) as error:
        return {**result, 'reason': 'return_reference_geometry_invalid', 'input_error': str(error)}
    reasons = []
    if not checked['observation_ok']:
        reasons.append('return_sweep_unobserved')
    if not checked['geometry_ok']:
        reasons.append('return_geometry_unconfirmed')
    return {**result, 'sampled_sweep_ok': not reasons, 'reason': reasons[0] if reasons else 'return_sweep_supported',
            'reasons': reasons, 'envelope': checked, 'envelope_speed_mps': bound,
            'source_age_s': age, 'target_speed_mps': target_speed, 'steering_rad': steering,
            'previous_steering_rad': previous_steering,
            'body_standstill_verified': False}


class ReturnGuardAudit:
    """해제 이후 진단 범위를 유지한다. 운영 명령·위협 장부를 변경하지 않는다.

    새로운 세대나 입력 단절 자체로 관찰 필요성을 지우지 않는다. 실제 경기/명시
    재설정 경계를 알고 있는 호출자만 reset을 호출한다. 스캔 freshness는 원래 검사에서 다룬다.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.ever_released = False
        self.release_events = 0

    def observe_release(self, detail):
        lateral = detail.get('m3_lateral') or {}
        if lateral.get('released_to_baseline') is True:
            self.ever_released = True
            self.release_events += 1
        return self.ever_released
