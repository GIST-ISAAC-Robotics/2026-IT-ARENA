"""/drive 요청을 ToF/바퀴 엔코더만으로 제한해 /drive/safe에 전달합니다."""
import json
import math
from pathlib import Path
import time

import numpy as np
import yaml
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from std_msgs.msg import String

from arena_vehicle_interface.node_lifecycle import run_node
from arena_autonomy.tof_safety_core import SafetyGeometry, cloud_xyz, safe_speed, swept_clearance, to_body


def seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class TofSafety(Node):
    def __init__(self):
        super().__init__("tof_safety")
        path = self.declare_parameter("vehicle_config", "").value
        config = yaml.safe_load(Path(path).read_text(encoding="utf-8"))["vehicle"]
        defaults = {"sensor_timeout_s": .30, "command_timeout_s": .30, "encoder_timeout_s": .15,
                    "reaction_time_s": .15, "assumed_braking_deceleration_mps2": 2.0,
                    "assumed_detection_distance_m": .50, "stop_margin_m": .05,
                    "clear_hold_s": .8, "control_rate_hz": 50.0,
                    "minimum_obstacle_height_m": .015, "maximum_obstacle_height_m": .22,
                    "swept_width_m": .17, "geometry_margin_m": .015}
        self.p = {key: float(self.declare_parameter(key, value).value) for key, value in defaults.items()}
        if any(not math.isfinite(v) or v <= 0 for v in self.p.values()):
            raise ValueError("ToF 안전층 매개변수는 유한한 양수여야 합니다.")
        if self.p["assumed_detection_distance_m"] <= self.p["stop_margin_m"]:
            raise ValueError("검출 거리 가정은 정지 여유보다 길어야 합니다.")
        if self.p["minimum_obstacle_height_m"] >= self.p["maximum_obstacle_height_m"]:
            raise ValueError("장애물 높이 마스크가 잘못되었습니다.")
        drive = config["drivetrain"]
        self.radius = float(drive["wheel_radius_m"])
        self.steering_limit = math.atan(float(drive["wheelbase_m"]) /
            (float(drive["wheelbase_m"]) / math.tan(float(drive["max_steering_angle_rad"])) +
             float(drive["track_width_m"]) / 2))
        encoders = config["sensors"]["wheel_encoders"]
        self.wheel_names = (encoders["left_joint_name"], encoders["right_joint_name"])
        self.modules = config["sensors"]["tof_ring"]["modules"]
        if not self.modules or not config["sensors"]["tof_ring"]["enabled"]:
            raise ValueError("활성 ToF 모듈이 필요합니다.")
        self.geometry = SafetyGeometry(wheelbase=float(drive["wheelbase_m"]),
            length=float(config["footprint"]["length_m"]),
            width=max(float(config["footprint"]["width_m"]), self.p["swept_width_m"]),
            margin=self.p["geometry_margin_m"], minimum_height=self.p["minimum_obstacle_height_m"],
            maximum_height=self.p["maximum_obstacle_height_m"], horizon=self.p["assumed_detection_distance_m"])
        self.clouds, self.last_cloud_stamps = {}, {}
        self.request = None
        self.request_at = self.encoder_at = -math.inf
        self.request_wall = self.encoder_wall = 0.0
        self.measured_speed = 0.0
        self.latched = False
        self.clear_since = None
        self.last_time = -math.inf
        self.last_status = ""
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive/safe", 10)
        self.status_publisher = self.create_publisher(String, "/safety/status", 10)
        self.create_subscription(AckermannDriveStamped, "/drive", self.on_drive, 10)
        self.create_subscription(JointState, "/wheel_states", self.on_wheels, qos_profile_sensor_data)
        for module in self.modules:
            self.create_subscription(PointCloud2, f"{module['topic']}/points",
                lambda message, module=module: self.on_cloud(message, module), qos_profile_sensor_data)
        self.create_timer(1 / self.p["control_rate_hz"], self.control)
        self.get_logger().info("ToF 최소 감속·정지층 활성. 평지/정적 표적 근사, 실차 안전 보증 아님.")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_drive(self, message):
        now = self.now_s()
        stamp = seconds(message.header.stamp)
        if (not all(math.isfinite(v) for v in (message.drive.speed, message.drive.steering_angle)) or
                (stamp != 0 and not -.03 <= now - stamp <= self.p["command_timeout_s"])):
            self.request = None
            return
        self.request = message
        self.request_at, self.request_wall = now, time.monotonic()

    def on_wheels(self, message):
        stamp = seconds(message.header.stamp)
        if stamp < self.encoder_at:
            return
        values = dict(zip(message.name, message.velocity))
        if not all(name in values and math.isfinite(values[name]) for name in self.wheel_names):
            self.encoder_at = -math.inf
            return
        self.measured_speed = sum(values[name] for name in self.wheel_names) * self.radius / 2
        self.encoder_at, self.encoder_wall = stamp, time.monotonic()

    def on_cloud(self, message, module):
        name, stamp = module["name"], seconds(message.header.stamp)
        if stamp <= self.last_cloud_stamps.get(name, -math.inf):
            return
        self.last_cloud_stamps[name] = stamp
        try:
            if message.header.frame_id != module["frame_id"]:
                raise ValueError("ToF 좌표계가 설정과 다릅니다.")
            points = cloud_xyz(message)
            if not len(points):
                raise ValueError("ToF가 모두 비유효입니다. 빈 공간으로 취급하지 않습니다.")
            self.clouds[name] = (stamp, time.monotonic(), to_body(points, module["xyz_m"], module["rpy_rad"]))
        except (ValueError, TypeError, BufferError):
            self.clouds.pop(name, None)

    def control(self):
        now, wall = self.now_s(), time.monotonic()
        if now < self.last_time:
            self.clouds.clear()
            self.last_cloud_stamps.clear()
            self.request = None
            self.encoder_at = -math.inf
            self.latched = True
            self.clear_since = None
        self.last_time = now
        reason, output, steering, clearance, limit, age = "CLEAR", 0.0, 0.0, math.inf, 0.0, 0.0
        requested = float(self.request.drive.speed) if self.request is not None else 0.0
        stale = [m["name"] for m in self.modules if m["name"] not in self.clouds or
                 not -.03 <= now - self.clouds[m["name"]][0] < self.p["sensor_timeout_s"] or
                 wall - self.clouds[m["name"]][1] > 3.0]
        if self.request is None or not 0 <= now - self.request_at < self.p["command_timeout_s"] or wall - self.request_wall > 3:
            reason = "COMMAND_STOP"
        elif not -.03 <= now - self.encoder_at < self.p["encoder_timeout_s"] or wall - self.encoder_wall > 3:
            reason = "ENCODER_STOP"
        elif stale:
            reason = "TOF_STALE_STOP"
        else:
            steering = float(np.clip(self.request.drive.steering_angle, -self.steering_limit, self.steering_limit))
            age = max(0, max(now - value[0] for value in self.clouds.values()))
            latency = self.p["reaction_time_s"] + age
            points = np.vstack([value[2] for value in self.clouds.values()])
            reversing = abs(self.measured_speed) > .04 and requested * self.measured_speed < 0
            direction = self.measured_speed if reversing or abs(requested) <= .001 else requested
            clearance = swept_clearance(points, steering, direction, self.geometry)
            available = min(clearance, self.p["assumed_detection_distance_m"])
            limit = safe_speed(available - self.p["stop_margin_m"], latency,
                               self.p["assumed_braking_deceleration_mps2"])
            stopping = (self.p["stop_margin_m"] + abs(self.measured_speed) * latency +
                        self.measured_speed ** 2 / (2 * self.p["assumed_braking_deceleration_mps2"]))
            if math.isfinite(clearance) and clearance <= stopping:
                self.latched = True
            if self.latched:
                # 장애물이 현재 관측 경로에서 사라진 상태를 유지해야 자동 재개합니다.
                if math.isinf(clearance) and abs(self.measured_speed) < .04:
                    if self.clear_since is None:
                        self.clear_since = now
                    if now - self.clear_since >= self.p["clear_hold_s"]:
                        self.latched = False
                else:
                    self.clear_since = None
            if self.latched:
                reason = "OBSTACLE_STOP"
            elif reversing:
                reason = "REVERSAL_BRAKE"
            else:
                output = math.copysign(min(abs(requested), limit), requested)
                if abs(output) + 1e-6 < abs(requested):
                    reason = "OBSTACLE_SLOW" if math.isfinite(clearance) else "VISIBILITY_SPEED_LIMIT"
        if reason in {"COMMAND_STOP", "ENCODER_STOP", "TOF_STALE_STOP"}:
            self.clear_since = None
        message = AckermannDriveStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.drive.speed, message.drive.steering_angle = output, steering
        self.publisher.publish(message)
        status = {"state": reason, "sim_time_s": now, "requested_speed_mps": requested,
                  "safe_speed_mps": output, "wheel_speed_mps": self.measured_speed,
                  "clearance_m": clearance if math.isfinite(clearance) else None,
                  "speed_limit_mps": limit, "cloud_age_s": age, "stale_modules": stale,
                  "obstacle_latched": self.latched, "model": "flat_ground_static_obstacle"}
        self.status_publisher.publish(String(data=json.dumps(status, allow_nan=False)))
        if reason != self.last_status:
            self.get_logger().info(reason)
            self.last_status = reason

    def destroy_node(self):
        self.publisher.publish(AckermannDriveStamped())
        return super().destroy_node()


def main(args=None):
    run_node(TofSafety, args=args)
