"""실물 독립 구동 계약의 PC 참조 구현. 펌웨어/ESC 드라이버가 아니다.

모든 now_us는 수신 MCU의 단조 시각이다. 임대 토큰은 시계 동기화 없이
명령의 최대 잔여 수명을 제한한다. CRC/토큰은 통신 인증을 대신하지 않는다.
"""
from dataclasses import dataclass
from enum import Enum
import math
import uuid


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
    ticks_per_rev: int = 2048
    max_speed_mps: float = 2.5
    max_steering_rad: float = .37
    stopped_wheel_mps: float = .03
    max_wheel_mps: float = 5.

    def __post_init__(self):
        for key in ("lease_us", "feedback_timeout_us", "settle_us", "max_control_gap_us", "ticks_per_rev"):
            if not integer(getattr(self, key), 1, 10_000_000):
                raise ValueError(f"invalid {key}")
        for key in ("wheel_radius_m", "max_speed_mps", "max_steering_rad", "stopped_wheel_mps", "max_wheel_mps"):
            if not number(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(f"invalid {key}")
        if self.stopped_wheel_mps >= self.max_speed_mps or self.max_speed_mps > self.max_wheel_mps:
            raise ValueError("inconsistent speed limits")


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
        self.left_mps = self.right_mps = 0.
        self.zero_since_us = None
        self.estop_held = False
        self._leases = {}
        self._lease_sequence = 0

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

    def _check(self, now_us):
        if not self._clock(now_us):
            return False
        if (self.armed and (self.last_control_us is None or
                now_us - self.last_control_us > self.config.max_control_gap_us)):
            self._fault("control_deadline_missed")
        if self.armed:
            if not self._fresh_feedback(now_us):
                self._fault("encoder_timeout")
            elif self.deadline_us is None or now_us >= self.deadline_us:
                self._fault("command_expired")
        return True

    def tick(self, now_us):
        # 통신 수신/상태 발행이 아니라 실제 제어 작업만 이 시각을 갱신한다.
        if self._check(now_us):
            self.last_control_us = now_us
        return self.status()

    def update_encoder(self, now_us, *, sequence, capture_us, left_ticks, right_ticks):
        """MCU에서 같은 시각에 캡처한 좌우 누적 카운트. 도착 시각으로 차분하지 않는다."""
        if not self._clock(now_us):
            return False
        valid = (integer(sequence) and integer(capture_us) and capture_us <= now_us and
                 now_us - capture_us < self.config.feedback_timeout_us and
                 integer(left_ticks, -(2**63)) and integer(right_ticks, -(2**63)))
        if not valid:
            self.feedback_valid, self.zero_since_us = False, None
            if self.armed:
                self._fault("invalid_encoder")
            return False
        sample = (sequence, capture_us, left_ticks, right_ticks)
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
        scale = math.tau * self.config.wheel_radius_m / self.config.ticks_per_rev * 1e6 / dt_us
        left, right = (left_ticks - previous[2]) * scale, (right_ticks - previous[3]) * scale
        if max(abs(left), abs(right)) > self.config.max_wheel_mps:
            self.feedback_valid, self.zero_since_us = False, None
            if self.armed:
                self._fault("encoder_implausible")
            return False
        self.feedback = sample
        self.feedback_valid = True
        self.left_mps, self.right_mps = left, right
        if max(abs(left), abs(right)) <= self.config.stopped_wheel_mps:
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
        if not self._check(now_us):
            raise ValueError("clock invalid")
        self._lease_sequence += 1
        issued, expires = now_us, now_us + self.config.lease_us
        if len(self._leases) >= 32:
            self._leases.pop(next(iter(self._leases)))
        self._leases[self._lease_sequence] = (issued, expires)
        return {**self.status(), "lease": self._lease_sequence, "lease_issued_us": issued,
                "lease_expires_us": expires}

    def receive(self, message, now_us):
        if not self._check(now_us) or type(message) is not dict or type(message.get("v")) is not int or message["v"] != 1:
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
                "right_wheel_mps": self.right_mps,
                "feedback_valid": self._fresh_feedback(self.now_us or 0),
                "feedback_seq": None if self.feedback is None else self.feedback[0],
                "left_ticks": None if self.feedback is None else self.feedback[2],
                "right_ticks": None if self.feedback is None else self.feedback[3],
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
