"""명시 선택형 모터축 운동 관측. 원 MCU 카운트/취득 간격을 보존한다."""
from collections import Counter
from dataclasses import dataclass
import math

from arena_vehicle_interface.drive_feedback import MotorEncoderSpec


@dataclass(frozen=True)
class MotorMotion:
    stamp_ns: int
    position_rad: float
    velocity_rad_s: float
    boot: str
    clock_epoch: int
    generation: int
    sequence: int
    mcu_capture_us: int
    motor_counts: int
    host_receipt_ns: int
    time_basis: str
    time_contract_verified: bool = False


class MotorFeedbackRuntime:
    """첫 표본/무효 입력에는 속도를 만들지 않는다. 두 유효 카운트의 평균 속도다.

    속도는 MCU 취득 간격으로 계산하며 ROS 취득 시각은 기존 상태 시계의 추정값이다.
    MCU 시계율/엔코더 분해능/장착 방향의 정확도를 검증하는 기능은 아니다.
    """
    def __init__(self, spec, *, encoder_sign, maximum_interval_ns, maximum_age_ns, maximum_speed_mps):
        if not isinstance(spec, MotorEncoderSpec):
            raise ValueError('explicit motor encoder specification required')
        if type(encoder_sign) is not int or encoder_sign not in (-1, 1):
            raise ValueError('explicit encoder sign required')
        if any(type(v) is not int or v <= 0 for v in (maximum_interval_ns, maximum_age_ns)):
            raise ValueError('positive integer interval and age bounds required')
        if (isinstance(maximum_speed_mps, bool) or not math.isfinite(maximum_speed_mps)
                or maximum_speed_mps <= 0):
            raise ValueError('positive finite physical speed bound required')
        self.spec, self.sign = spec, encoder_sign
        self.max_interval, self.max_age, self.max_speed = maximum_interval_ns, maximum_age_ns, maximum_speed_mps
        self.previous = None
        self.identity = None
        self.retired_boots = set()
        self.counts = Counter()
        self.reason = 'no_sample'

    def reject(self, reason, *, reset=False):
        self.reason = reason
        self.counts[reason] += 1
        if reset: self.previous = None
        return None

    def convert(self, status):
        raw = status.get('motor_feedback') if type(status) is dict else None
        if type(raw) is not dict or raw.get('schema') != 'motor_feedback_observation_v1':
            return self.reject('absent_feedback', reset=True)
        if raw.get('source_valid') is not True:
            return self.reject('invalid_feedback', reset=True)
        boot = raw.get('boot')
        if (type(boot) is not str or not boot or raw.get('time_contract_verified') is not False
                or raw.get('time_basis') != 'minimum_receipt_offset_mcu_state_estimate'):
            return self.reject('invalid_provenance', reset=True)
        if any(type(raw.get(k)) is not int or raw[k] < 0 for k in
               ('clock_epoch', 'generation', 'sequence', 'mcu_capture_us', 'estimated_capture_ns', 'host_receipt_ns')):
            return self.reject('invalid_value', reset=True)
        count = raw.get('motor_counts')
        if type(count) is not int or not -(1 << 63) <= count < (1 << 63):
            return self.reject('invalid_count', reset=True)
        if any(status.get(k) != raw[k] for k in ('boot', 'clock_epoch', 'generation', 'host_receipt_ns')):
            return self.reject('status_identity_mismatch', reset=True)
        if boot in self.retired_boots:
            return self.reject('retired_boot')
        identity = (boot, raw['clock_epoch'], raw['generation'])
        if identity != self.identity:
            if self.identity is not None:
                if boot == self.identity[0] and (identity[1] < self.identity[1] or identity[2] < self.identity[2]):
                    return self.reject('retired_epoch')
                if boot != self.identity[0]:
                    if len(self.retired_boots) >= 64: raise RuntimeError('start a new explicit encoder session')
                    self.retired_boots.add(self.identity[0])
            self.identity = identity
            self.previous = None
        age = raw['host_receipt_ns'] - raw['estimated_capture_ns']
        if not 0 <= age <= self.max_age:
            return self.reject('capture_age', reset=True)
        sample = (raw['sequence'], raw['mcu_capture_us'], raw['estimated_capture_ns'], count)
        previous = self.previous
        if previous is not None:
            if sample == previous: return self.reject('duplicate')
            if sample[0] <= previous[0] or sample[1] <= previous[1] or sample[2] <= previous[2]:
                return self.reject('nonmonotonic')
        self.previous = sample
        if previous is None: return self.reject('first_sample')
        dt_ns = (sample[1] - previous[1]) * 1000
        if dt_ns > self.max_interval: return self.reject('long_interval')
        velocity = self.spec.motor_rad_s_from_counts(self.sign*(count - previous[3]), dt_ns*1e-9)
        if abs(self.spec.mean_wheel_mps(velocity)) > self.max_speed:
            return self.reject('implausible_speed', reset=True)
        position = self.spec.motor_angle_from_counts(self.sign*count)
        self.reason = 'observed_count_difference'
        self.counts['published'] += 1
        return MotorMotion(raw['estimated_capture_ns'], position, velocity, boot, raw['clock_epoch'],
                           raw['generation'], raw['sequence'], raw['mcu_capture_us'], count,
                           raw['host_receipt_ns'], raw['time_basis'])
