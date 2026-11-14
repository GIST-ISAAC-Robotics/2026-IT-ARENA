"""MCU/host 선택 프로필 일치 확인. 교정값 생성이나 장치 I/O 없음."""
import hashlib
import json
import math
from pathlib import Path
from run_runtime_actuation_host import strict_json


def matching_mcu_profile(path, board):
    profile = strict_json(Path(path).read_bytes())
    required = {'schema', 'mode', 'assembly_id', 'encoder_direction', 'motor_direction_confirmed',
        'forward_dir_high', 'maximum_duty', 'kp', 'ki', 'maximum_speed_mps', 'maximum_steering_rad', 'feedback_period_us'}
    if type(profile) is not dict or set(profile) != required or profile['schema'] != 'uno_q_profile_v1':
        raise ValueError('exact uno_q_profile_v1 required')
    if profile['mode'] != 'installed' or board is None:
        raise ValueError('matching installed MCU and host profiles required for runtime ARM')
    for key in ('motor_direction_confirmed', 'forward_dir_high'):
        if type(profile[key]) is not bool:raise ValueError('explicit MCU boolean required: '+key)
    for key in ('encoder_direction', 'feedback_period_us'):
        if type(profile[key]) is not int:raise ValueError('explicit MCU integer required: '+key)
    if type(profile['assembly_id']) is not str:raise ValueError('MCU assembly_id required')
    for field in ('assembly_id', 'encoder_direction', 'motor_direction_confirmed', 'forward_dir_high',
                  'maximum_duty', 'kp', 'ki', 'maximum_speed_mps', 'feedback_period_us'):
        if type(profile[field]) is bool and type(board[field]) is not bool:
            raise ValueError('MCU/host profile type mismatch: '+field)
        if profile[field] != board[field]:
            raise ValueError('MCU/host profile mismatch: '+field)
    if profile['maximum_steering_rad'] != board['servo']['command_limit']:
        raise ValueError('MCU/host profile mismatch: maximum_steering_rad')
    if not 0 < profile['maximum_steering_rad'] <= .28475436568260193:
        raise ValueError('MCU selected steering limit')
    canonical = json.dumps(profile, sort_keys=True, separators=(',', ':'), ensure_ascii=True, allow_nan=False)
    return hashlib.sha256(canonical.encode('ascii')).hexdigest()


def matching_motor_feedback(record):
    if record is None:return
    declared = record['declared']
    # installed MCU가 이미 encoder_direction을 적용한 count를 내보낸다.
    expected = dict(counts_per_motor_revolution=28, gear_ratio=2.5*81/17,
                    wheel_radius_m=.0325, encoder_sign=1)
    for key, value in expected.items():
        if not math.isclose(declared[key], value, rel_tol=1e-12, abs_tol=1e-12):
            raise ValueError('MCU/host encoder geometry or sign mismatch: '+key)


class ProfileGatedMotorFeedback:
    """raw count가 설치된 부호라고 확인되기 전에는 정규 속도를 발행하지 않는다."""
    def __init__(self, converter, allowed):self.converter, self.allowed = converter, allowed
    def __getattr__(self, name):return getattr(self.converter, name)
    def convert(self, status):
        if not self.allowed(status):return self.converter.reject('MCU_profile_unconfirmed', reset=True)
        return self.converter.convert(status)
