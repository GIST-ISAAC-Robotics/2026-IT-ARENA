"""구동 피드백 표현의 순수 계약. ROS 의존성이 없다.

기본 표현은 모터 뒤축 PGE-213 하나다. 모터 각 = 감속비 × (좌 + 우 바퀴 각) / 2로
기계식 차동의 평균 입력만 관측한다. 좌우 차이·개별 슬립·기어 유격·좌우 반대 회전은
이 측정 하나로 알 수 없으며, 이 모듈은 그 값을 좌우 독립 측정으로 꾸며 내지 않는다.

`rear_wheel_pair_legacy`는 이전 좌우 후륜 엔코더 2개 계약과 옛 기록의 명시 호환용이다.
"""
from dataclasses import dataclass
import math

MODE_MOTOR = 'drive_motor_shaft'
MODE_LEGACY = 'rear_wheel_pair_legacy'
MODES = (MODE_MOTOR, MODE_LEGACY)

MOTOR_TOPIC = '/drive_motor/encoder'
MOTOR_JOINT = 'drive_motor_shaft'
MOTOR_FRAME = 'drive_motor_encoder'
LEGACY_TOPIC = '/wheel_states'
LEGACY_TICKS_TOPIC = '/wheel_encoder_ticks'
LEGACY_JOINTS = ('rear_left_wheel_joint', 'rear_right_wheel_joint')
TOPIC_BY_MODE = {MODE_MOTOR: MOTOR_TOPIC, MODE_LEGACY: LEGACY_TOPIC}

# 단일 모터축 관측에서 정지 판정이 보장하는 범위. 평균 0만 볼 수 있다.
MOTOR_STATIONARY_SCOPE = 'mean_wheel_only_opposite_rotation_unobservable'
LEGACY_STATIONARY_SCOPE = 'both_rear_wheels_low_speed'


def _finite(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def validate_mode(mode):
    if mode not in MODES:
        raise ValueError(f'drive feedback mode must be one of {MODES}, got {mode!r}')
    return mode


@dataclass(frozen=True)
class MotorEncoderSpec:
    """모터축 엔코더 + 전체 감속비 + 바퀴 반지름. 기본값을 두지 않아 출처를 명시하게 한다."""
    counts_per_motor_revolution: int
    gear_ratio: float
    wheel_radius_m: float

    def __post_init__(self):
        if type(self.counts_per_motor_revolution) is not int or not 1 <= self.counts_per_motor_revolution <= 1_000_000:
            raise ValueError('counts_per_motor_revolution must be a positive integer')
        for name in ('gear_ratio', 'wheel_radius_m'):
            value = getattr(self, name)
            if not _finite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')

    @property
    def motor_rad_per_count(self):
        return math.tau / self.counts_per_motor_revolution

    @property
    def counts_per_wheel_revolution(self):
        """평균 바퀴 1회전당 카운트. 개별 바퀴의 카운트가 아니다."""
        return self.counts_per_motor_revolution * self.gear_ratio

    @property
    def mean_wheel_travel_m_per_count(self):
        return math.tau * self.wheel_radius_m / self.counts_per_wheel_revolution

    def counts_from_motor_angle(self, motor_angle_rad):
        if not _finite(motor_angle_rad):
            raise ValueError('motor angle must be finite')
        return round(motor_angle_rad / self.motor_rad_per_count)

    def motor_angle_from_counts(self, counts):
        if type(counts) is not int:
            raise ValueError('counts must be an integer')
        return counts * self.motor_rad_per_count

    def counts_from_position(self, position_rad, tolerance_counts=1e-6):
        """메시지 위치를 정수 카운트로 되돌린다. 카운트 격자에서 벗어나면 거절한다."""
        if not _finite(position_rad):
            raise ValueError('motor position must be finite')
        exact = position_rad / self.motor_rad_per_count
        # 잘못된 거대 유한 값도 round(inf) 예외로 ROS 콜백을 중단시키지 않는다.
        # 전송 계약의 부호 있는 64비트 카운터 범위를 넘는 위치는 거절한다.
        if not math.isfinite(exact) or not -(2**63) <= exact < 2**63:
            raise ValueError('motor position exceeds the signed 64-bit count range')
        counts = round(exact)
        if abs(exact - counts) > tolerance_counts:
            raise ValueError('motor position is not on the encoder count grid')
        return counts

    def motor_rad_s_from_counts(self, delta_counts, delta_time_s):
        if type(delta_counts) is not int or not _finite(delta_time_s) or delta_time_s <= 0:
            raise ValueError('count delta must be integer and capture interval positive')
        return delta_counts * self.motor_rad_per_count / delta_time_s

    def mean_wheel_rad_s(self, motor_rad_s):
        if not _finite(motor_rad_s):
            raise ValueError('motor speed must be finite')
        return motor_rad_s / self.gear_ratio

    def mean_wheel_mps(self, motor_rad_s):
        return self.mean_wheel_rad_s(motor_rad_s) * self.wheel_radius_m

    def speed_quantum_mps(self, delta_time_s):
        """한 카운트가 표본 간격에서 만드는 평균 바퀴 속도 단계. 제어 오차 보장값이 아니다."""
        if not _finite(delta_time_s) or delta_time_s <= 0:
            raise ValueError('delta_time_s must be positive')
        return self.mean_wheel_travel_m_per_count / delta_time_s


def motor_angle_from_wheels(left_rad, right_rad, gear_ratio):
    """대칭 차동·동일 반지름·전진 부호 일치 가정의 모터 각. 시뮬레이션 관절 정답에서만 합성한다."""
    if not all(_finite(v) for v in (left_rad, right_rad, gear_ratio)) or gear_ratio <= 0:
        raise ValueError('wheel angles and gear ratio must be finite; ratio positive')
    return gear_ratio * (left_rad + right_rad) / 2


def _index(names, target, allow_prefix):
    matches = [i for i, name in enumerate(names)
               if name == target or (allow_prefix and name.endswith('::' + target))]
    if len(matches) != 1:
        raise ValueError(f'exactly one {target!r} joint required')
    return matches[0]


def parse_motor_state(names, positions, velocities, require_finite=True):
    """단일 모터축 메시지를 (위치, 속도)로 읽는다. 바퀴 이름이 섞이면 거절한다.

    require_finite=False는 호출자가 비유한 값을 거절·기록하는 경우에만 쓴다.
    """
    names = list(names)
    if len(names) != 1 or names[0] != MOTOR_JOINT:
        raise ValueError(f'drive motor feedback must contain only {MOTOR_JOINT!r}')
    if len(positions) != 1 or len(velocities) != 1:
        raise ValueError('drive motor feedback requires one position and one velocity')
    position, velocity = float(positions[0]), float(velocities[0])
    if require_finite and (not math.isfinite(position) or not math.isfinite(velocity)):
        raise ValueError('drive motor feedback must be finite')
    return position, velocity


def parse_legacy_wheels(names, values, left=LEGACY_JOINTS[0], right=LEGACY_JOINTS[1], require_finite=True):
    """옛 좌우 후륜 메시지. 두 이름이 모두 있어야 하며 모터축 이름은 거절한다."""
    names = list(names)
    if any(name == MOTOR_JOINT or name.endswith('::' + MOTOR_JOINT) for name in names):
        raise ValueError('motor-shaft feedback cannot be read as a rear wheel pair')
    li, ri = _index(names, left, True), _index(names, right, True)
    if max(li, ri) >= len(values):
        raise ValueError('both named wheel values are required')
    pair = float(values[li]), float(values[ri])
    if require_finite and not all(math.isfinite(v) for v in pair):
        raise ValueError('wheel values must be finite')
    return pair
