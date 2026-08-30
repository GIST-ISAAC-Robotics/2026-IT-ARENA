from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.node import Node


class AckermannToTwist(Node):
    """Translate the competition-facing /drive contract to Gazebo Twist commands."""

    def __init__(self) -> None:
        super().__init__("ackermann_to_twist")
        self.declare_parameter("wheelbase_m", 0.13)
        self.declare_parameter("max_speed_mps", 2.0)
        self.declare_parameter("max_steering_angle_rad", 0.45)
        self.declare_parameter("command_timeout_s", 0.5)

        self._wheelbase = float(self.get_parameter("wheelbase_m").value)
        self._max_speed = abs(float(self.get_parameter("max_speed_mps").value))
        self._max_steering = abs(
            float(self.get_parameter("max_steering_angle_rad").value)
        )
        self._timeout = max(float(self.get_parameter("command_timeout_s").value), 0.05)
        if self._wheelbase <= 0.0:
            raise ValueError("wheelbase_m must be positive")

        self._publisher = self.create_publisher(Twist, "/sim/cmd_vel", 10)
        self._subscription = self.create_subscription(
            AckermannDriveStamped, "/drive", self._on_drive, 10
        )
        self._last_command_ns: int | None = None
        self._stop_sent = True
        self._timer = self.create_timer(0.05, self._watchdog)

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)

    def _on_drive(self, message: AckermannDriveStamped) -> None:
        speed = self._clamp(message.drive.speed, -self._max_speed, self._max_speed)
        steering = self._clamp(
            message.drive.steering_angle, -self._max_steering, self._max_steering
        )

        command = Twist()
        command.linear.x = speed
        command.angular.z = speed * math.tan(steering) / self._wheelbase
        self._publisher.publish(command)
        self._last_command_ns = self.get_clock().now().nanoseconds
        self._stop_sent = False

    def _watchdog(self) -> None:
        if self._last_command_ns is None or self._stop_sent:
            return
        age_s = (self.get_clock().now().nanoseconds - self._last_command_ns) / 1e9
        if age_s >= self._timeout:
            self._publisher.publish(Twist())
            self._stop_sent = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AckermannToTwist()
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
