"""Turn Gazebo wheel-joint truth into a configurable encoder-like interface.

기본 `drive_motor_shaft` 모드는 모터 뒤축 엔코더 하나를 흉내 낸다. 이상적 좌우 바퀴
관절 각에서 모터 각 = 감속비 × 평균 바퀴 각을 먼저 만들고, 그 한 축을 모터축
카운트로 양자화해 /drive_motor/encoder 한 채널만 발행한다. 좌우 값은 발행하지 않는다.
`rear_wheel_pair_legacy`는 이전 좌우 후륜 엔코더 2개 계약의 명시 비교용이다.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import random

from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray

from arena_vehicle_interface.drive_feedback import (
    LEGACY_TICKS_TOPIC, LEGACY_TOPIC, MODE_LEGACY, MODE_MOTOR, MOTOR_FRAME, MOTOR_JOINT, MOTOR_TOPIC,
    MotorEncoderSpec, motor_angle_from_wheels, validate_mode,
)
from arena_vehicle_interface.node_lifecycle import run_node


def radians_per_tick(ticks_per_revolution: int) -> float:
    if ticks_per_revolution <= 0:
        raise ValueError("ticks_per_revolution must be positive")
    return 2.0 * math.pi / ticks_per_revolution


def position_to_ticks(position_rad: float, tick_angle_rad: float) -> int:
    return round(position_rad / tick_angle_rad)


def tick_delta_to_velocity(
    current_ticks: int,
    previous_ticks: int,
    tick_angle_rad: float,
    delta_time_s: float,
) -> float:
    if delta_time_s <= 0.0:
        return 0.0
    return (current_ticks - previous_ticks) * tick_angle_rad / delta_time_s


@dataclass(frozen=True)
class PendingSample:
    capture_time_ns: int
    due_time_ns: int
    stamp: Time
    left_ticks: int | None = None
    right_ticks: int | None = None
    motor_counts: int | None = None


class SimWheelEncoder(Node):
    """Publish sampled drive feedback: one motor-shaft channel or the legacy wheel pair."""

    def __init__(self) -> None:
        super().__init__("sim_wheel_encoder")

        # 모드는 launch가 명시해야 한다. 기본값으로 옛 2개 계약을 몰래 고르지 않는다.
        self.declare_parameter("feedback_mode", "")
        self.declare_parameter("left_joint_name", "rear_left_wheel_joint")
        self.declare_parameter("right_joint_name", "rear_right_wheel_joint")
        self.declare_parameter("ticks_per_revolution", 2048)
        self.declare_parameter("counts_per_motor_revolution", 0)
        self.declare_parameter("gear_ratio", 0.0)
        self.declare_parameter("wheel_radius_m", 0.0)
        self.declare_parameter("sample_rate_hz", 100.0)
        self.declare_parameter("latency_ms", 2.0)
        self.declare_parameter("dropout_probability", 0.0)
        self.declare_parameter("random_seed", 2026)

        self._mode = validate_mode(str(self.get_parameter("feedback_mode").value))
        self._joint_names = (
            str(self.get_parameter("left_joint_name").value),
            str(self.get_parameter("right_joint_name").value),
        )
        sample_rate_hz = float(self.get_parameter("sample_rate_hz").value)
        latency_ms = float(self.get_parameter("latency_ms").value)
        self._dropout_probability = float(
            self.get_parameter("dropout_probability").value
        )

        if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")
        if not math.isfinite(latency_ms) or latency_ms < 0.0:
            raise ValueError("latency_ms cannot be negative")
        if not 0.0 <= self._dropout_probability <= 1.0:
            raise ValueError("dropout_probability must be within [0, 1]")

        self._spec: MotorEncoderSpec | None = None
        self._radians_per_tick = 0.0
        if self._mode == MODE_MOTOR:
            self._spec = MotorEncoderSpec(
                int(self.get_parameter("counts_per_motor_revolution").value),
                float(self.get_parameter("gear_ratio").value),
                float(self.get_parameter("wheel_radius_m").value),
            )
        else:
            self._ticks_per_revolution = int(self.get_parameter("ticks_per_revolution").value)
            self._radians_per_tick = radians_per_tick(self._ticks_per_revolution)

        self._sample_period_ns = round(1_000_000_000 / sample_rate_hz)
        self._latency_ns = round(latency_ms * 1_000_000)
        self._random = random.Random(int(self.get_parameter("random_seed").value))
        self._pending: deque[PendingSample] = deque()
        self._last_capture_time_ns: int | None = None
        self._last_clock_time_ns: int | None = None
        self._last_published: PendingSample | None = None
        self._warned_missing_joint = False

        if self._mode == MODE_MOTOR:
            self._motor_publisher = self.create_publisher(JointState, MOTOR_TOPIC, 20)
        else:
            self._wheel_state_publisher = self.create_publisher(JointState, LEGACY_TOPIC, 20)
            self._tick_publisher = self.create_publisher(Int64MultiArray, LEGACY_TICKS_TOPIC, 20)
        self.create_subscription(
            JointState, "/sim/joint_states_raw", self._capture, 50
        )
        # The timer only releases delayed samples. Sampling itself is driven by
        # incoming joint-state messages and constrained by _sample_period_ns.
        self.create_timer(0.001, self._publish_due_samples)

        if self._mode == MODE_MOTOR:
            spec = self._spec
            self.get_logger().info(
                "Simulated drive motor encoder ready (single channel; left/right difference unobservable): "
                f"{spec.counts_per_motor_revolution} counts/motor rev x {spec.gear_ratio:g}:1 = "
                f"{spec.counts_per_wheel_revolution:g} counts/mean wheel rev, "
                f"{spec.mean_wheel_travel_m_per_count * 1000:.4f} mm/count, "
                f"{sample_rate_hz:g} Hz, {latency_ms:g} ms latency (provisional)"
            )
        else:
            self.get_logger().warning(
                "LEGACY rear wheel pair encoders selected explicitly: "
                f"{self._ticks_per_revolution} ticks/rev, {sample_rate_hz:g} Hz, "
                f"{latency_ms:g} ms latency; not the selected hardware feedback"
            )

    def _find_joint_index(self, message: JointState, target: str) -> int | None:
        for index, name in enumerate(message.name):
            if name == target or name.endswith(f"::{target}"):
                return index
        return None

    def _capture(self, message: JointState) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        self._check_clock_reset(now_ns)
        # 물리 관절 표본의 시각을 사용합니다. ROS 도착 시각을 캡처 시각으로
        # 쓰면 전송 지연의 흔들림이 바퀴 속도 오차로 바뀝니다.
        capture_ns = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        if capture_ns < 0 or capture_ns > now_ns + 100_000_000:
            return
        if (
            self._last_capture_time_ns is not None
            and capture_ns - self._last_capture_time_ns < self._sample_period_ns
        ):
            return

        indices = tuple(
            self._find_joint_index(message, target) for target in self._joint_names
        )
        if any(index is None for index in indices):
            if not self._warned_missing_joint:
                self.get_logger().warning(
                    "Encoder source joints not found yet; available names: "
                    + ", ".join(message.name)
                )
                self._warned_missing_joint = True
            return

        left_index, right_index = indices
        assert left_index is not None and right_index is not None
        if max(left_index, right_index) >= len(message.position):
            return
        if not all(math.isfinite(message.position[index]) for index in indices):
            return

        if self._random.random() < self._dropout_probability:
            self._last_capture_time_ns = capture_ns
            return

        left, right = message.position[left_index], message.position[right_index]
        if self._mode == MODE_MOTOR:
            # 좌우 정답은 여기서 평균 모터 각으로 합쳐진 뒤 버려진다.
            motor_angle = motor_angle_from_wheels(left, right, self._spec.gear_ratio)
            sample = PendingSample(capture_ns, capture_ns + self._latency_ns, message.header.stamp,
                                   motor_counts=self._spec.counts_from_motor_angle(motor_angle))
        else:
            sample = PendingSample(capture_ns, capture_ns + self._latency_ns, message.header.stamp,
                                   left_ticks=position_to_ticks(left, self._radians_per_tick),
                                   right_ticks=position_to_ticks(right, self._radians_per_tick))
        self._pending.append(sample)
        self._last_capture_time_ns = capture_ns

    def _check_clock_reset(self, now_ns: int) -> None:
        if self._last_clock_time_ns is not None and now_ns < self._last_clock_time_ns:
            self._pending.clear()
            self._last_capture_time_ns = None
            self._last_published = None
        self._last_clock_time_ns = now_ns

    def _publish_due_samples(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._check_clock_reset(now_ns)
        while self._pending and self._pending[0].due_time_ns <= now_ns:
            sample = self._pending.popleft()
            self._publish(sample)

    def _motor_state(self, sample: PendingSample) -> JointState:
        """모터축 한 채널. 첫 표본/재시작 직후 속도는 기준이 없으므로 0이 아니라 NaN이다."""
        velocity = math.nan
        previous = self._last_published
        if previous is not None and previous.motor_counts is not None:
            delta_time_s = (sample.capture_time_ns - previous.capture_time_ns) / 1_000_000_000
            if delta_time_s > 0.0:
                velocity = self._spec.motor_rad_s_from_counts(
                    sample.motor_counts - previous.motor_counts, delta_time_s)
        state = JointState()
        state.header.stamp = sample.stamp
        state.header.frame_id = MOTOR_FRAME
        state.name = [MOTOR_JOINT]
        state.position = [self._spec.motor_angle_from_counts(sample.motor_counts)]
        state.velocity = [velocity]
        return state

    def _publish(self, sample: PendingSample) -> None:
        if self._mode == MODE_MOTOR:
            self._motor_publisher.publish(self._motor_state(sample))
            self._last_published = sample
            return
        positions = [
            sample.left_ticks * self._radians_per_tick,
            sample.right_ticks * self._radians_per_tick,
        ]
        velocities = [0.0, 0.0]

        previous = self._last_published
        if previous is not None:
            delta_time_s = (
                sample.capture_time_ns - previous.capture_time_ns
            ) / 1_000_000_000
            if delta_time_s > 0.0:
                velocities = [
                    tick_delta_to_velocity(
                        sample.left_ticks,
                        previous.left_ticks,
                        self._radians_per_tick,
                        delta_time_s,
                    ),
                    tick_delta_to_velocity(
                        sample.right_ticks,
                        previous.right_ticks,
                        self._radians_per_tick,
                        delta_time_s,
                    ),
                ]

        state = JointState()
        state.header.stamp = sample.stamp
        state.header.frame_id = "base_link"
        state.name = list(self._joint_names)
        state.position = positions
        state.velocity = velocities
        self._wheel_state_publisher.publish(state)

        ticks = Int64MultiArray()
        # Stable order: [rear-left, rear-right]. Joint names are available on
        # /wheel_states and documented at the vehicle interface boundary.
        ticks.data = [sample.left_ticks, sample.right_ticks]
        self._tick_publisher.publish(ticks)
        self._last_published = sample


def main(args: list[str] | None = None) -> None:
    run_node(SimWheelEncoder, args=args)


if __name__ == "__main__":
    main()
