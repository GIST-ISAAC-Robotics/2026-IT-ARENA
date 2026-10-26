"""B3 순수 계약 후보. ROS/MCU 명령을 발행하거나 운영 정책을 자동 교체하지 않는다.

시각 출발 허가, 일시 보호, 명시 STOP/고정 고장, MCU 재허가를 분리한다.
실제 정지 증명/인증/안전 회로를 이 상태기계로 대체하지 않는다.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PermitDecision:
    allowed: bool
    reason: str
    race_epoch: int
    intervention: int
    started: bool


class RacePermission:
    def __init__(self):
        self.race_epoch = 0
        self.intervention = 0
        self.started = False
        self.latch = None
        self.last_visual_stamp = None

    def new_race(self):
        """명시 경기 초기화. STOP/고장을 자동 해제하지 않는다."""
        self.race_epoch += 1
        self.started = False
        self.last_visual_stamp = None
        return self.race_epoch

    def observe_start(self, *, race_epoch, visual_confirmed, capture_s, now_s, max_age_s):
        """신선도/세대 검사를 통과한 카메라 판정만 수락한다.

        visual_confirmed는 타이머/UDP가 아니라 픽셀 검출기의 결과여야 한다.
        순수 모델은 호출자의 실제 출처까지 인증하지 않으므로 ROS 연결 시 별도 감사한다.
        """
        valid = (type(race_epoch) is int and race_epoch == self.race_epoch and
                 visual_confirmed is True and self.latch is None and
                 all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                     for v in (capture_s, now_s, max_age_s)) and
                 max_age_s > 0 and capture_s >= 0 and 0 <= now_s-capture_s < max_age_s and
                 (self.last_visual_stamp is None or capture_s > self.last_visual_stamp))
        if not valid:
            return False
        self.started = True
        self.last_visual_stamp = capture_s
        return True

    def stop(self, reason='explicit_stop'):
        self.intervention += 1
        self.latch = str(reason) or 'explicit_stop'
        return self.intervention

    def fault(self, reason):
        return self.stop('fault:'+str(reason))

    def clock_reset(self):
        self.new_race()
        return self.stop('clock_reset')

    def acknowledge(self, *, race_epoch, intervention, hazards_cleared, operator_reviewed):
        """별도 명시 개입만 해제한다. 구동 ARM/센서 상태/정지 확인을 수행하지 않는다.

        race/개입 번호가 맞아야 하며 늦게 도착한 옛 해제는 새 STOP을 풀 수 없다.
        operator_reviewed는 소프트웨어가 차체 정지를 센서로 증명했다는 뜻이 아니다.
        """
        if (type(race_epoch) is not int or race_epoch != self.race_epoch or
                type(intervention) is not int or intervention != self.intervention or
                hazards_cleared is not True or operator_reviewed is not True or self.latch is None):
            return False
        self.latch = None
        return True

    def decide(self, *, sensors_ready, actuator_ready, temporary_hold=False):
        """동작 허가는 요청 상한일 뿐이다. 독립 보호/MCU가 별도로 거절할 수 있다."""
        allowed = (self.latch is None and self.started and sensors_ready is True and
                   actuator_ready is True and temporary_hold is False)
        if self.latch is not None:
            reason = self.latch
        elif not self.started:
            reason = 'wait_visual_start'
        elif sensors_ready is not True:
            reason = 'sensor_hold'
        elif actuator_ready is not True:
            reason = 'actuator_not_ready'
        elif temporary_hold is not False:
            reason = 'temporary_hold'
        else:
            reason = 'race_permitted'
        return PermitDecision(allowed, reason, self.race_epoch,
                              self.intervention, self.started)
