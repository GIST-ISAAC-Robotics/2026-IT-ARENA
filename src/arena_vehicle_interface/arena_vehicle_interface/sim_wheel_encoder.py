"""Turn Gazebo wheel-joint truth into a configurable encoder-like interface."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import random

import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int64MultiArray


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
    left_ticks: int
    right_ticks: int


class SimWheelEncoder(Node):
    """Publish sampled wheel ticks and quantized wheel position / velocity."""

    def __init__(self) -> None:
        super().__init__("sim_wheel_encoder")

        self.declare_parameter("left_joint_name", "rear_left_wheel_joint")
        self.declare_parameter("right_joint_name", "rear_right_wheel_joint")
        self.declare_parameter("ticks_per_revolution", 2048)
        self.declare_parameter("sample_rate_hz", 100.0)
        self.declare_parameter("latency_ms", 2.0)
        self.declare_parameter("dropout_probability", 0.0)
        self.declare_parameter("random_seed", 2026)

        self._joint_names = (
            str(self.get_parameter("left_joint_name").value),
            str(self.get_parameter("right_joint_name").value),
        )
        self._ticks_per_revolution = int(
            self.get_parameter("ticks_per_revolution").value
        )
        sample_rate_hz = float(self.get_parameter("sample_rate_hz").value)
        latency_ms = float(self.get_parameter("latency_ms").value)
        self._dropout_probability = float(
            self.get_parameter("dropout_probability").value
        )

        if sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive")
        if latency_ms < 0.0:
            raise ValueError("latency_ms cannot be negative")
        if not 0.0 <= self._dropout_probability <= 1.0:
            raise ValueError("dropout_probability must be within [0, 1]")

        self._sample_period_ns = round(1_000_000_000 / sample_rate_hz)
        self._latency_ns = round(latency_ms * 1_000_000)
        self._radians_per_tick = radians_per_tick(self._ticks_per_revolution)
        self._random = random.Random(int(self.get_parameter("random_seed").value))
        self._pending: deque[PendingSample] = deque()
        self._last_capture_time_ns: int | None = None
        self._last_published: PendingSample | None = None
        self._warned_missing_joint = False

        self._wheel_state_publisher = self.create_publisher(
            JointState, "/wheel_states", 20
        )
        self._tick_publisher = self.create_publisher(
            Int64MultiArray, "/wheel_encoder_ticks", 20
        )
        self.create_subscription(
            JointState, "/sim/joint_states_raw", self._capture, 50
        )
        # The timer only releases delayed samples. Sampling itself is driven by
        # incoming joint-state messages and constrained by _sample_period_ns.
        self.create_timer(0.001, self._publish_due_samples)

        self.get_logger().info(
            "Simulated rear encoders ready: "
            f"{self._ticks_per_revolution} ticks/rev, {sample_rate_hz:g} Hz, "
            f"{latency_ms:g} ms latency"
        )

    def _find_joint_index(self, message: JointState, target: str) -> int | None:
        for index, name in enumerate(message.name):
            if name == target or name.endswith(f"::{target}"):
                return index
        return None

    def _capture(self, message: JointState) -> None:
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if (
            self._last_capture_time_ns is not None
            and now_ns - self._last_capture_time_ns < self._sample_period_ns
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

        if self._random.random() < self._dropout_probability:
            self._last_capture_time_ns = now_ns
            return

        left_ticks = position_to_ticks(
            message.position[left_index], self._radians_per_tick
        )
        right_ticks = position_to_ticks(
            message.position[right_index], self._radians_per_tick
        )
        self._pending.append(
            PendingSample(
                capture_time_ns=now_ns,
                due_time_ns=now_ns + self._latency_ns,
                stamp=now.to_msg(),
                left_ticks=left_ticks,
                right_ticks=right_ticks,
            )
        )
        self._last_capture_time_ns = now_ns

    def _publish_due_samples(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        while self._pending and self._pending[0].due_time_ns <= now_ns:
            sample = self._pending.popleft()
            self._publish(sample)

    def _publish(self, sample: PendingSample) -> None:
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
    rclpy.init(args=args)
    node = SimWheelEncoder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
