"""실물 독립 구동 계약의 PC 참조 구현. 펌웨어/ESC 드라이버가 아니다.

모든 now_us는 수신 MCU의 단조 시각이다. 임대 토큰은 시계 동기화 없이
명령의 최대 잔여 수명을 제한한다. CRC/토큰은 통신 인증을 대신하지 않는다.

피드백은 feedback_mode로 명시한다. drive_motor_shaft는 모터축 누적 카운트 하나만
받아 평균 바퀴 속도를 계산하며, 정지 판정도 평균 0만 볼 수 있다(좌우 반대 회전 미검출).
rear_wheel_pair_legacy는 이전 좌우 후륜 2개 계약이며 ContractConfig 기본값은 과거
참조 시험 재현을 위해 legacy로 둔다. 현재 launch는 모터축 모드를 명시한다.
"""
from dataclasses import dataclass
from enum import Enum
import math
import uuid

from arena_vehicle_interface.drive_feedback import (
    LEGACY_STATIONARY_SCOPE, MODE_LEGACY, MODE_MOTOR, MODES, MOTOR_STATIONARY_SCOPE,
)


def integer(value, minimum=0, maximum=2**63 - 1):
    return type(value) is int and minimum <= value <= maximum


def number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


class State(str, Enum):
    DISARMED = "DISARMED"
    ARMED = "ARMED"
    ACTIVE = "ACTIVE"
    ESTOP = "ESTOP_LATCHED"
    FAULT = "FAULT_LATCHED"


@dataclass(frozen=True)
class ContractConfig:
    lease_us: int = 100_000
    feedback_timeout_us: int = 50_000
    settle_us: int = 200_000
    max_control_gap_us: int = 50_000
    wheel_radius_m: float = .025
    # legacy: 바퀴 1회전당 틱. drive_motor_shaft: 모터 1회전당 카운트.
    ticks_per_rev: int = 2048
    max_speed_mps: float = 2.5
    max_steering_rad: float = .37
    stopped_wheel_mps: float = .03
    max_wheel_mps: float = 5.
    feedback_mode: str = MODE_LEGACY
    # 모터→바퀴 전체 감속비. legacy 바퀴 틱에는 1이어야 한다.
    gear_ratio: float = 1.
    # 모터축 모드 한정 임시 무진행 감시. 실물 응답 확인 전의 보수적 개발값이다.
    motor_no_progress_us: int = 500_000
    motor_progress_min_command_mps: float = .1

    def __post_init__(self):
        if self.feedback_mode not in MODES:
            raise ValueError("invalid feedback_mode")
        if not number(self.gear_ratio) or self.gear_ratio <= 0:
            raise ValueError("invalid gear_ratio")
        if self.feedback_mode == MODE_LEGACY and self.gear_ratio != 1:
            raise ValueError("legacy wheel ticks require gear_ratio 1")
        for key in ("lease_us", "feedback_timeout_us", "settle_us", "max_control_gap_us", "ticks_per_rev", "motor_no_progress_us"):
            if not integer(getattr(self, key), 1, 10_000_000):
                raise ValueError(f"invalid {key}")
        for key in ("wheel_radius_m", "max_speed_mps", "max_steering_rad", "stopped_wheel_mps", "max_wheel_mps", "motor_progress_min_command_mps"):
            if not number(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f"invalid {key}")
        if self.stopped_wheel_mps >= self.max_speed_mps or self.max_speed_mps > self.max_wheel_mps:
            raise ValueError("inconsistent speed limits")
        if self.motor_progress_min_command_mps > self.max_speed_mps:
            raise ValueError("motor progress threshold exceeds command limit")


class ActuationReceiver:
    """단일 제어 작업에서 호출한다. 통신 콜백과 동시 접근하려면 잠금/큐가 필요하다."""
    def __init__(self, config=None, *, boot_id=None):
        self.config = config or ContractConfig()
        self.boot_id = boot_id or uuid.uuid4().hex
        self.generation = 0
        self.state = State.DISARMED
        self.reason = "boot"
        self.owner = None
        self.sequence = -1
        self.target_speed = self.steering = 0.
        self.deadline_us = None
        self.now_us = None
        self.last_control_us = None
        self.feedback = None
        self.feedback_valid = False
        self.motor_feedback = self.config.feedback_mode == MODE_MOTOR
        # 모터축 모드에는 좌우 값이 존재하지 않는다. 0으로도 채우지 않는다.
        self.left_mps = self.right_mps = None if self.motor_feedback else 0.
        self.mean_wheel_mps = 0.
        self.zero_since_us = None
        self.estop_held = False
        self._leases = {}
        self._lease_sequence = 0
        self._motor_progress_since_us = None
        # 진단 전용. 첫 control_deadline_missed 직전 값을 한 번만 고정하며 판정에는 쓰지 않는다.
        self.deadline_diagnostic = None
        self.deadline_miss_count = 0

    @property
    def armed(self):
        return self.state in (State.ARMED, State.ACTIVE)

    def _transition(self, state, reason):
        self.state, self.reason = state, reason
        self.target_speed = 0.
        # 정지 중 갑작스러운 중앙 조향은 하지 않는다. 실차 정책은 별도 확인한다.
        self.deadline_us = None
        self.owner, self.sequence = None, -1
        self.generation += 1
        self._leases.clear()
        self._motor_progress_since_us = None

    def _fault(self, reason):
        if self.state not in (State.ESTOP, State.FAULT):
            self._transition(State.FAULT, reason)

    def _clock(self, now_us):
        if not integer(now_us):
            self._fault("invalid_clock")
            return False
        if self.now_us is not None and now_us < self.now_us:
            self._fault("clock_regressed")
            self.feedback_valid = False
            self.zero_since_us = None
            return False
        self.now_us = now_us
        self._leases = {key: times for key, times in self._leases.items() if now_us < times[1]}
        return True

    def _fresh_feedback(self, now_us):
        return (self.feedback_valid and self.feedback is not None and
                0 <= now_us - self.feedback[1] < self.config.feedback_timeout_us)

    def _stationary(self, now_us):
        return (self._fresh_feedback(now_us) and self.zero_since_us is not None and
                now_us - self.zero_since_us >= self.config.settle_us)

    def _check(self, now_us, origin="check"):
        if not self._clock(now_us):
            return False
        if (self.armed and (self.last_control_us is None or
                now_us - self.last_control_us > self.config.max_control_gap_us)):
            self.deadline_miss_count += 1
            if self.deadline_diagnostic is None:
                # last_control_us 갱신·상태 전환 전에 고정한다. 이후 정리/시간 초과로 바꾸지 않는다.
                self.deadline_diagnostic = {
                    "origin": origin, "now_us": now_us, "last_control_us": self.last_control_us,
                    "gap_us": None if self.last_control_us is None else now_us - self.last_control_us,
                    "threshold_us": self.config.max_control_gap_us,
                    "state_before": self.state.value, "reason_before": self.reason,
                    "generation_before": self.generation, "deadline_us_before": self.deadline_us}
            self._fault("control_deadline_missed")
        if self.armed:
            if not self._fresh_feedback(now_us):
                self._fault("encoder_timeout")
            elif self.deadline_us is None or now_us >= self.deadline_us:
                self._fault("command_expired")
            elif (self.motor_feedback and self._motor_progress_since_us is not None and
                  now_us - self._motor_progress_since_us >= self.config.motor_no_progress_us):
                # 신선한 시각만 반복되는 고착/구동 정체. 어느 쪽인지는 단일 축으로 판별 불가.
                self._fault("motor_feedback_no_progress")
        return True

    def tick(self, now_us):
        # 통신 수신/상태 발행이 아니라 실제 제어 작업만 이 시각을 갱신한다.
        if self._check(now_us, "tick"):
            self.last_control_us = now_us
        return self.status()

    def update_encoder(self, now_us, *, sequence, capture_us, left_ticks, right_ticks):
        """legacy: MCU에서 같은 시각에 캡처한 좌우 누적 카운트. 도착 시각으로 차분하지 않는다."""
        if self.motor_feedback:
            return self._mode_mismatch(now_us)
        return self._ingest(now_us, sequence, capture_us, (left_ticks, right_ticks))

    def update_motor_encoder(self, now_us, *, sequence, capture_us, motor_counts):
        """모터축 엔코더 한 채널의 누적 카운트. 평균 바퀴 운동만 계산한다."""
        if not self.motor_feedback:
            return self._mode_mismatch(now_us)
        return self._ingest(now_us, sequence, capture_us, (motor_counts,))

    def _mode_mismatch(self, now_us):
        if not self._clock(now_us):
            return False
        self.feedback_valid, self.zero_since_us = False, None
        if self.armed:
            self._fault("feedback_mode_mismatch")
        return False

    def _ingest(self, now_us, sequence, capture_us, counts):
        if not self._clock(now_us):
            return False
        valid = (integer(sequence) and integer(capture_us) and capture_us <= now_us and
                 now_us - capture_us < self.config.feedback_timeout_us and
                 all(integer(count, -(2**63)) for count in counts))
        if not valid:
            self.feedback_valid, self.zero_since_us = False, None
            if self.armed:
                self._fault("invalid_encoder")
            return False
        sample = (sequence, capture_us, *counts)
        previous = self.feedback
        if previous is None:
            self.feedback = sample
            return True
        if sequence <= previous[0] or capture_us <= previous[1]:
            return False  # 중복/순서 역전은 신선도를 갱신하지 않는다.
        dt_us = capture_us - previous[1]
        if dt_us >= self.config.feedback_timeout_us:
            self.feedback = sample
            self.feedback_valid, self.zero_since_us = False, None
            if self.armed:
                self._fault("encoder_gap")
            return False
        scale = (math.tau * self.config.wheel_radius_m /
                 (self.config.ticks_per_rev * self.config.gear_ratio) * 1e6 / dt_us)
        speeds = [(count - old) * scale for count, old in zip(counts, previous[2:])]
        if max(abs(v) for v in speeds) > self.config.max_wheel_mps:
            self.feedback_valid, self.zero_since_us = False, None
            if self.armed:
                self._fault("encoder_implausible")
            return False
        self.feedback = sample
        self.feedback_valid = True
        if self.motor_feedback:
            self.mean_wheel_mps = speeds[0]
            if counts[0] != previous[2] and self._motor_progress_since_us is not None:
                self._motor_progress_since_us = max(self._motor_progress_since_us, capture_us)
        else:
            self.left_mps, self.right_mps = speeds
            self.mean_wheel_mps = (speeds[0] + speeds[1]) / 2
        # 모터축 모드의 정지 준비는 평균 0뿐이다. 좌우 반대 회전은 여기서 검출되지 않는다.
        if max(abs(v) for v in speeds) <= self.config.stopped_wheel_mps:
            if self.zero_since_us is None:
                self.zero_since_us = capture_us
        else:
            self.zero_since_us = None
        return True

    def set_estop(self, pressed, now_us):
        if type(pressed) is not bool:
            raise ValueError("pressed must be bool")
        self._clock(now_us)
        self.estop_held = pressed
        if pressed:
            self._transition(State.ESTOP, "physical_stop")
        # 해제는 상태 전환이 아니다. CLEAR와 ARM이 따로 필요하다.

    def offer(self, now_us):
        """MCU가 발행하는 짧은 유효기간 토큰. 토큰 수신만으로 watchdog을 먹이지 않는다."""
        if not self._check(now_us, "offer"):
            raise ValueError("clock invalid")
        self._lease_sequence += 1
        issued, expires = now_us, now_us + self.config.lease_us
        if len(self._leases) >= 32:
            self._leases.pop(next(iter(self._leases)))
        self._leases[self._lease_sequence] = (issued, expires)
        return {**self.status(), "lease": self._lease_sequence, "lease_issued_us": issued,
                "lease_expires_us": expires}

    def receive(self, message, now_us):
        if not self._check(now_us, "receive") or type(message) is not dict or type(message.get("v")) is not int or message["v"] != 1:
            return False, "invalid_message"
        kind = message.get("kind")
        if kind == "STOP" and set(message) == {"v", "kind"}:
            self._transition(State.ESTOP, "software_stop")
            return True, self.reason
        common = {"v", "kind", "boot", "generation", "lease", "client", "seq"}
        expected = common | ({"speed_mps", "steering_rad", "source_age_us", "ttl_us"} if kind == "DRIVE" else set())
        if kind not in ("ARM", "CLEAR", "DISARM", "DRIVE") or set(message) != expected:
            return False, "schema"
        if (message["boot"] != self.boot_id or not integer(message["generation"]) or
                message["generation"] != self.generation or not integer(message["lease"], 1) or
                type(message["client"]) is not str or not 1 <= len(message["client"]) <= 64 or
                not integer(message["seq"], 0, 2**32 - 1)):
            return False, "session"
        lease = self._leases.get(message["lease"])
        if lease is None:
            return False, "expired_lease"
        if kind in ("CLEAR", "ARM"):
            if not self._stationary(now_us) or self.estop_held:
                return False, "not_ready"
            if self.last_control_us is None or now_us - self.last_control_us > self.config.max_control_gap_us:
                return False, "control_not_ready"
            if kind == "CLEAR":
                if self.state not in (State.FAULT, State.ESTOP):
                    return False, "not_latched"
                self._transition(State.DISARMED, "explicit_clear")
            else:
                if self.state != State.DISARMED:
                    return False, "not_disarmed"
                self._transition(State.ARMED, "explicit_arm")
                self.owner, self.sequence = message["client"], message["seq"]
                self.deadline_us = lease[1]
            return True, self.reason
        if not self.armed or message["client"] != self.owner:
            return False, "not_owner_or_armed"
        if message["seq"] <= self.sequence:
            return False, "sequence"
        if kind == "DISARM":
            self._transition(State.DISARMED, "explicit_disarm")
            return True, self.reason
        speed, angle = message["speed_mps"], message["steering_rad"]
        age, ttl = message["source_age_us"], message["ttl_us"]
        if (not number(speed) or not 0 <= speed <= self.config.max_speed_mps or
                not number(angle) or abs(angle) > self.config.max_steering_rad or
                not integer(age) or not integer(ttl, 1, self.config.lease_us) or age >= ttl):
            self._fault("invalid_drive")
            return False, self.reason
        # 최악 전송 지연을 임대 발행 이후 전체 시간으로 잡아 수명을 부풀리지 않는다.
        deadline = min(lease[1], lease[0] + ttl - age)
        if now_us >= deadline:
            return False, "source_expired"
        self.sequence = message["seq"]
        if self.motor_feedback:
            if speed < self.config.motor_progress_min_command_mps:
                self._motor_progress_since_us = None
            elif self._motor_progress_since_us is None:
                self._motor_progress_since_us = now_us
        self.target_speed, self.steering = float(speed), float(angle)
        self.deadline_us = deadline
        self.state = State.ACTIVE if speed > 0 else State.ARMED
        self.reason = "drive" if speed > 0 else "zero_target"
        return True, self.reason

    def status(self):
        return {"v": 1, "boot": self.boot_id, "generation": self.generation,
                "state": self.state.value, "reason": self.reason, "owner": self.owner,
                "accepted_seq": self.sequence, "target_speed_mps": self.target_speed,
                "steering_rad": self.steering, "left_wheel_mps": self.left_mps,
                "right_wheel_mps": self.right_mps, "mean_wheel_mps": self.mean_wheel_mps,
                "feedback_mode": self.config.feedback_mode,
                "stationary_scope": MOTOR_STATIONARY_SCOPE if self.motor_feedback else LEGACY_STATIONARY_SCOPE,
                "feedback_valid": self._fresh_feedback(self.now_us or 0),
                "feedback_seq": None if self.feedback is None else self.feedback[0],
                "left_ticks": None if self.feedback is None or self.motor_feedback else self.feedback[2],
                "right_ticks": None if self.feedback is None or self.motor_feedback else self.feedback[3],
                "motor_counts": None if self.feedback is None or not self.motor_feedback else self.feedback[2],
                "feedback_capture_us": None if self.feedback is None else self.feedback[1],
                "deadline_us": self.deadline_us, "estop_held": self.estop_held}


class CommandProducer:
    """Jetson 쪽 참조. 원래 생성 시각을 검사하며 같은 표본 재전송으로 수명을 늘리지 않는다."""
    def __init__(self, *, client_id=None, ttl_us=100_000):
        if not integer(ttl_us, 1, 1_000_000):
            raise ValueError("invalid ttl")
        self.client = client_id or uuid.uuid4().hex
        self.ttl_us = ttl_us
        self.sequence = 0
        self.last_source_us = None

    def control(self, offer, kind):
        if kind not in ("ARM", "CLEAR", "DISARM"):
            raise ValueError("invalid control")
        self.sequence += 1
        return {"v": 1, "kind": kind, "boot": offer["boot"], "generation": offer["generation"],
                "lease": offer["lease"], "client": self.client, "seq": self.sequence}

    def drive(self, offer, *, speed_mps, steering_rad, source_us, host_now_us):
        if (not integer(source_us) or not integer(host_now_us) or
                not 0 <= host_now_us - source_us < self.ttl_us or
                (self.last_source_us is not None and source_us <= self.last_source_us)):
            raise ValueError("stale, repeated or regressed source")
        if offer["owner"] != self.client or offer["state"] not in (State.ARMED.value, State.ACTIVE.value):
            raise ValueError("explicit ARM required for this producer")
        self.last_source_us = source_us
        self.sequence += 1
        return {"v": 1, "kind": "DRIVE", "boot": offer["boot"], "generation": offer["generation"],
                "lease": offer["lease"], "client": self.client, "seq": self.sequence,
                "speed_mps": speed_mps, "steering_rad": steering_rad,
                "source_age_us": host_now_us - source_us, "ttl_us": self.ttl_us}
