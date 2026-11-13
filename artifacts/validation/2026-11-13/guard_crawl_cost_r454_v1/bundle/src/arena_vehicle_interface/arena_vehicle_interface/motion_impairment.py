"""시험 전용 IMU/휠 입력 오류 설정. 실제 센서 분포/교정값이 아니다."""
from dataclasses import asdict, dataclass, field
import math


@dataclass(frozen=True)
class StreamFault:
    scale: float = 1.
    bias: float = 0.
    delay_s: float = 0.
    jitter_s: float = 0.
    stamp_offset_s: float = 0.
    start_s: float = 0.
    end_s: float | None = None
    drop: bool = False

    def __post_init__(self):
        for key, value in asdict(self).items():
            if key not in ('drop', 'end_s') and (isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)):
                raise ValueError(f'{key}: finite number required')
        if not .1 <= self.scale <= 10 or not 0 <= self.jitter_s <= self.delay_s <= 2:
            raise ValueError('invalid gain or delay/jitter')
        if abs(self.stamp_offset_s) > 1 or self.start_s < 0 or type(self.drop) is not bool:
            raise ValueError('invalid stamp/window/drop')
        if self.end_s is not None and (isinstance(self.end_s, bool) or not isinstance(self.end_s, (int, float)) or
                                      not math.isfinite(self.end_s) or self.end_s <= self.start_s):
            raise ValueError('end must be finite and after start')

    def active(self, stamp):
        return stamp >= self.start_s and (self.end_s is None or stamp < self.end_s)

    def transform(self, value, stamp):
        return value * self.scale + self.bias if self.active(stamp) else value

    def claimed_stamp(self, stamp):
        return stamp + self.stamp_offset_s if self.active(stamp) else stamp

    def delay(self, stamp, rng):
        return max(0., self.delay_s + rng.uniform(-self.jitter_s, self.jitter_s)) if self.active(stamp) else 0.


@dataclass(frozen=True)
class MotionFaultConfig:
    imu: StreamFault = field(default_factory=StreamFault)
    wheels: StreamFault = field(default_factory=StreamFault)
    seed: int = 22

    def __post_init__(self):
        if type(self.seed) is not int or not 0 <= self.seed < 2**32:
            raise ValueError('seed must be uint32')
        if self.wheels.bias != 0:
            raise ValueError('wheel additive error is not implemented; use scale')

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict) or set(data) - {'imu', 'wheels', 'seed'}:
            raise ValueError('unknown motion fault keys')
        return cls(imu=StreamFault(**data.get('imu', {})), wheels=StreamFault(**data.get('wheels', {})),
                   seed=data.get('seed', 22))

    def as_dict(self):
        return asdict(self)
