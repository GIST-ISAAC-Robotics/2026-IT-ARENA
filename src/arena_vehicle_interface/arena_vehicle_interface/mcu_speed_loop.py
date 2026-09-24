"""엔코더 평균 피드백의 단일 모터 PI 참조. normalized effort는 ESC 신호가 아니다."""
from dataclasses import dataclass
from arena_vehicle_interface.actuation_contract import number


@dataclass(frozen=True)
class MotorIntent:
    effort: float
    brake_requested: bool


class SpeedPI:
    def __init__(self, kp=.6, ki=1.2, max_dt_s=.05):
        if not all(number(v) and v > 0 for v in (kp, ki, max_dt_s)):
            raise ValueError("invalid PI settings")
        self.kp, self.ki, self.max_dt_s = kp, ki, max_dt_s
        self.integral = 0.

    def update(self, target_mps, left_mps, right_mps, dt_s, *, enabled):
        if (type(enabled) is not bool or not all(number(v)
                for v in (target_mps, left_mps, right_mps, dt_s)) or
                target_mps < 0 or not 0 < dt_s <= self.max_dt_s):
            self.integral = 0.
            return MotorIntent(0., True)
        if not enabled or target_mps == 0:
            self.integral = 0.
            return MotorIntent(0., True)
        error = target_mps - (left_mps + right_mps) / 2
        candidate = self.integral + self.ki * error * dt_s
        raw = self.kp * error + candidate
        # 포화가 더 커지는 방향의 적분만 막는다. 중지/재허가에는 적분을 남기지 않는다.
        if -1 <= raw <= 1 or (raw > 1 and error < 0) or (raw < -1 and error > 0):
            self.integral = max(-1., min(1., candidate))
        effort = max(-1., min(1., self.kp * error + self.integral))
        return MotorIntent(effort, effort < 0)
