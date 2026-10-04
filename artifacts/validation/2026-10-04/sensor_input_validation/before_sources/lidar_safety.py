"""ToF 없는 구성의 독립 라이다 보호층. /drive -> /drive/safe."""
import json
import math
import time

import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.lidar_motion_ros import MotionInput, seconds
from arena_autonomy.local_path import mask_scan, swept_limit, braking_speed
from arena_vehicle_interface.node_lifecycle import run_node
from arena_vehicle_interface.timing_probe import TimingProbe


class LidarSafety(Node):
    def __init__(self):
        super().__init__("lidar_safety")
        self.timing = TimingProbe(self)
        self.motion = MotionInput(self)
        self.wall_timeout = float(self.declare_parameter("sensor_wall_timeout_s", 3.).value)
        self.limit = float(self.declare_parameter("max_steering_angle_rad", .37).value)
        self.wheelbase = float(self.declare_parameter('wheelbase_m', .145).value)
        self.length = float(self.declare_parameter('vehicle_length_m', .20).value)
        self.width = float(self.declare_parameter('vehicle_width_m', .15).value)
        self.rear_blind_half_angle = float(self.declare_parameter('rear_blind_half_angle_deg', 30.).value)
        if not all(math.isfinite(v) and v > 0 for v in (self.wall_timeout, self.limit, self.wheelbase, self.length, self.width)):
            raise ValueError('LiDAR safety geometry and timeouts must be finite and positive')
        if not 0 <= self.rear_blind_half_angle <= 80:
            raise ValueError('rear_blind_half_angle_deg must be within 0..80')
        self.scan = self.command = None
        self.scan_wall = self.command_wall = 0.
        self.last_clock = None
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive/safe", 10)
        self.status = self.create_publisher(String, "/safety/status", 10)
        self.create_subscription(LaserScan, "/scan", self.timing.wrap('scan', self.on_scan), qos_profile_sensor_data)
        self.create_subscription(AckermannDriveStamped, "/drive", self.timing.wrap('command', self.on_command), 10)
        self.create_timer(.02, self.timing.wrap('control', self.control))

    def on_scan(self, msg):
        if self.scan is None or seconds(msg.header.stamp) >= seconds(self.scan.header.stamp):
            self.scan, self.scan_wall = mask_scan(msg, self.rear_blind_half_angle), time.monotonic()

    def on_command(self, msg):
        if self.command is None or seconds(msg.header.stamp) >= seconds(self.command.header.stamp):
            self.command, self.command_wall = msg, time.monotonic()

    def control(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock:
            self.scan = self.command = None
            self.motion.history.reset()
        self.last_clock = now
        output = AckermannDriveStamped()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "base_link"
        if self.command is not None and math.isfinite(self.command.drive.steering_angle):
            output.drive.steering_angle = float(np.clip(self.command.drive.steering_angle, -self.limit, self.limit))
        reason, clearance = "input_stop", 0.
        try:
            if self.scan is None or self.command is None:
                raise MotionUnavailable("missing_input")
            if (not 0 <= now - seconds(self.command.header.stamp) <= .2 or
                not 0 <= now - seconds(self.scan.header.stamp) <= .3 or
                max(time.monotonic() - self.scan_wall, time.monotonic() - self.command_wall) > self.wall_timeout):
                raise MotionUnavailable("stale_input")
            speed, steering = self.command.drive.speed, self.command.drive.steering_angle
            if not all(math.isfinite(v) for v in (speed, steering)) or speed < 0:
                raise MotionUnavailable("invalid_or_reverse_command")
            # 전방 결측을 자유 공간으로 바꾸지 않는다. +Inf는 최대 거리 밖 응답.
            ranges = np.asarray(self.scan.ranges)
            angles = self.scan.angle_min + np.arange(len(ranges)) * self.scan.angle_increment
            forward = ranges[np.abs(angles) < math.radians(100)]
            if np.any(np.isnan(forward) | np.isneginf(forward) | (forward < self.scan.range_min)):
                raise MotionUnavailable("front_unobserved")
            points, count = self.motion.project(self.scan)
            if count < 30:
                raise MotionUnavailable("scan_invalid")
            output.drive.steering_angle = float(np.clip(steering, -self.limit, self.limit))
            clearance = swept_limit(points, output.drive.steering_angle, self.wheelbase,
                                    length=self.length, width=self.width)
            cap = braking_speed(clearance, now - seconds(self.scan.header.stamp))
            output.drive.speed = float(min(speed, cap))
            reason = "clear" if cap >= speed else "lidar_braking"
        except MotionUnavailable as error:
            reason = str(error)
        self.publisher.publish(output)
        self.status.publish(String(data=json.dumps({"reason": reason, "safe_speed_mps": output.drive.speed,
                                                    "swept_clearance_m": clearance, "sensor": "C1"})))


def main(args=None):
    run_node(LidarSafety, args=args)
