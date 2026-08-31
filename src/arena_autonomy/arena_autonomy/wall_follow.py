"""카메라 출발 신호와 라이다만 사용하는 반복 본선 주행 노드."""
import json
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from arena_vehicle_interface.node_lifecycle import run_node
from arena_autonomy.core import StartSignal, follow_command, image_rgb, marker_ids, scan_points


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class WallFollow(Node):
    def __init__(self):
        super().__init__("wall_follow")
        defaults = {"wheelbase_m": .145, "lidar_x_m": -.03, "max_steering_angle_rad": .43,
                    "target_wall_distance_m": .425, "max_speed_mps": .35, "min_speed_mps": .14,
                    "scan_timeout_s": .45, "image_timeout_s": 1.0, "control_rate_hz": 20.0,
                    "acceleration_mps2": .5, "lateral_acceleration_limit_mps2": 3.0}
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        if any(not math.isfinite(float(value)) for value in self.settings.values()):
            raise ValueError("자율주행 매개변수는 유한한 수여야 합니다.")
        if any(self.settings[key] <= 0 for key in defaults if key != "lidar_x_m"):
            raise ValueError("거리·주기·속도·조향 한계는 양수여야 합니다.")
        if self.settings["min_speed_mps"] > self.settings["max_speed_mps"]:
            raise ValueError("min_speed_mps는 max_speed_mps보다 클 수 없습니다.")
        self.signal = StartSignal()
        self.enabled = True
        self.side = "left"
        self.scan = None
        self.scan_wall_time = 0.0
        self.rgb_stamp = None
        self.image_wall_time = 0.0
        self.ids = []
        self.last_image_processed = -math.inf
        self.last_status = ""
        self.last_status_time = -math.inf
        self.last_steering = 0.0
        self.last_speed = 0.0
        self.last_control_time = -math.inf
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.status_publisher = self.create_publisher(String, "/autonomy/status", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/color/image_raw", self.on_image, QoSProfile(depth=2))
        self.create_service(SetBool, "/autonomy/enable", self.on_enable)
        self.create_service(Trigger, "/autonomy/reset", self.on_reset)
        self.create_timer(1 / self.settings["control_rate_hz"], self.control)
        self.get_logger().info("RGB 빨강→초록 확인을 기다립니다. /scan과 RGB만 사용하며 지도·정답 위치는 읽지 않습니다.")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, message):
        if self.scan is None or stamp_seconds(message.header.stamp) >= stamp_seconds(self.scan.header.stamp):
            self.scan = message
            self.scan_wall_time = time.monotonic()

    def on_image(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if stamp < self.last_image_processed:
            return  # 순서가 뒤바뀐 프레임으로 출발 판정을 초기화하지 않습니다.
        if stamp - self.last_image_processed < .095:
            return
        self.last_image_processed = stamp
        self.rgb_stamp = stamp
        self.image_wall_time = time.monotonic()
        rgb = image_rgb(message)
        self.signal.update(rgb)
        self.ids = marker_ids(rgb)
        # 현재 코스의 분기 방향만 설정한 간단한 규칙입니다. 좌표를 조회하지 않습니다.
        if 30 in self.ids:
            self.side = "right"
        elif any(marker in self.ids for marker in (0, 20, 45)):
            self.side = "left"

    def stop(self):
        self.last_speed = 0.0
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        self.publisher.publish(message)

    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self.stop()
        response.success = True
        response.message = "주행 허용" if self.enabled else "사용자 정지"
        return response

    def on_reset(self, request, response):
        self.stop()
        self.signal = StartSignal()
        self.side = "left"
        self.scan = None
        self.scan_wall_time = 0.0
        self.ids = []
        self.last_steering = 0.0
        self.last_image_processed = -math.inf
        self.rgb_stamp = None
        self.image_wall_time = 0.0
        self.last_status_time = -math.inf
        self.enabled = True
        response.success = True
        response.message = "새로운 빨강→초록 신호를 기다립니다. 신호 재시작은 /sim/traffic_light/reset입니다."
        return response

    def control(self):
        now = self.now_s()
        if now < self.last_control_time:
            was_enabled = self.enabled
            self.on_reset(None, Trigger.Response())
            self.enabled = was_enabled
        self.last_control_time = now
        status = {"state": "WAIT_GREEN", "side": self.side, "signal": self.signal.observed,
                  "marker_ids": self.ids, "started": self.signal.started, "enabled": self.enabled,
                  "sim_time_s": now}
        speed, steering = 0.0, self.last_steering
        scan_fresh = self.scan is not None and 0 <= now - stamp_seconds(self.scan.header.stamp) < self.settings["scan_timeout_s"]
        image_fresh = self.rgb_stamp is not None and 0 <= now - self.rgb_stamp < self.settings["image_timeout_s"]
        wall_fresh = time.monotonic() - self.scan_wall_time < 3 and time.monotonic() - self.image_wall_time < 3
        if not self.enabled:
            status["state"] = "DISABLED"
        elif not (scan_fresh and image_fresh and wall_fresh):
            status["state"] = "SENSOR_STOP"
        elif self.signal.started:
            points, valid_count = scan_points(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                                              self.scan.range_min, self.scan.range_max, self.settings["lidar_x_m"])
            if valid_count < 30:
                status["state"] = "SCAN_INVALID"
            else:
                speed, steering, details = follow_command(
                    points, self.side, self.settings["target_wall_distance_m"], self.settings["wheelbase_m"],
                    self.settings["max_steering_angle_rad"], self.settings["max_speed_mps"], self.settings["min_speed_mps"])
                status.update(details)
                curvature = abs(math.tan(steering) / self.settings["wheelbase_m"])
                if curvature > 1e-6:
                    speed = min(speed, math.sqrt(self.settings["lateral_acceleration_limit_mps2"] / curvature))
                status["state"] = "RUNNING" if speed > 0 else details["reason"].upper()
        dt = 1 / self.settings["control_rate_hz"]
        # 증속은 완만하게, 위험·센서 누락 시에는 즉시 0 명령을 내립니다.
        speed = min(speed, self.last_speed + self.settings.get("acceleration_mps2", .5) * dt)
        steering = max(self.last_steering - 3 * dt, min(self.last_steering + 3 * dt, steering))
        self.last_speed, self.last_steering = speed, steering
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed, message.drive.steering_angle = float(speed), float(steering)
        self.publisher.publish(message)
        status.update(speed_command_mps=speed, steering_command_rad=steering)
        if now - self.last_status_time > .2 or status["state"] != self.last_status:
            self.status_publisher.publish(String(data=json.dumps(status, ensure_ascii=False, allow_nan=False)))
            self.last_status_time = now
        if status["state"] != self.last_status:
            self.get_logger().info(f"{status['state']} / {self.side} 벽 / {self.signal.observed}")
            self.last_status = status["state"]

    def destroy_node(self):
        self.stop()
        return super().destroy_node()


def main(args=None):
    run_node(WallFollow, args=args)
