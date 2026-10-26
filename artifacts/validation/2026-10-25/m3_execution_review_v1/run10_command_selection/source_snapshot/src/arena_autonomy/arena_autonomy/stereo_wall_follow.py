"""RGB 신호·노면과 D435i급 전방 깊이로 동작하는 최소 본선 추종기."""

import json
import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from arena_vehicle_interface.node_lifecycle import run_node
from arena_autonomy.core import (
    StartSignal,
    depth_gap_command,
    depth_lateral_clearance,
    depth_wall_points,
    image_depth_m,
    image_rgb,
    marker_ids,
)
from arena_autonomy.stereo_road import road_target, road_steering


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class StereoWallFollow(Node):
    """RGB-D 목표점 추종과 깊이 통로 복구를 사용하는 단독 주행 기준선."""

    def __init__(self):
        super().__init__("stereo_wall_follow")
        defaults = {
            "wheelbase_m": .145,
            "camera_x_m": .055,
            "camera_y_m": 0.0,
            "camera_z_m": .075,
            "color_fx_px": 612.3337640816779,
            "color_fy_px": 617.1589761579678,
            "color_cx_px": 424.0,
            "color_cy_px": 240.0,
            "max_steering_angle_rad": .43,
            "target_wall_distance_m": .425,
            "max_speed_mps": .35,
            "min_speed_mps": .14,
            "depth_timeout_s": .25,
            "image_timeout_s": 1.0,
            "control_rate_hz": 20.0,
            "acceleration_mps2": .5,
            "lateral_acceleration_limit_mps2": 3.0,
            "depth_stride_px": 4,
            "wall_minimum_height_m": .09,
            "wall_maximum_height_m": .28,
            "wall_minimum_forward_m": .18,
            "wall_maximum_forward_m": 1.45,
            "gap_free_clearance_m": .55,
            "gap_row_half_height_px": 14,
            "gap_steering_weight": .55,
            "lateral_clearance_gain": 1.80,
            "lateral_clearance_hold_s": .80,
            "road_width_m": .45,
            "road_target_x_m": .34,
        }
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        if any(not math.isfinite(float(value)) for value in self.settings.values()):
            raise ValueError("자율주행 매개변수는 유한한 수여야 합니다.")
        if any(float(self.settings[key]) <= 0 for key in defaults if key != "camera_y_m"):
            raise ValueError("거리·주기·속도·조향 한계는 양수여야 합니다.")
        if self.settings["min_speed_mps"] > self.settings["max_speed_mps"]:
            raise ValueError("min_speed_mps는 max_speed_mps보다 클 수 없습니다.")
        self.signal = StartSignal()
        self.enabled = True
        self.side = "left"
        self.points = None
        self.depth = None
        self.rgb = None
        self.depth_valid_count = 0
        self.depth_stamp = None
        self.depth_wall_time = 0.0
        self.depth_info = None
        self.rgb_stamp = None
        self.image_wall_time = 0.0
        self.ids = []
        self.last_depth_processed = -math.inf
        self.last_image_processed = -math.inf
        self.last_status = ""
        self.last_status_time = -math.inf
        self.last_steering = 0.0
        self.last_speed = 0.0
        self.last_control_time = -math.inf
        self.last_lateral_correction = 0.0
        self.last_lateral_clearance_time = -math.inf
        self.publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.status_publisher = self.create_publisher(String, "/autonomy/status", 10)
        self.create_subscription(Image, "/camera/color/image_raw", self.on_image, QoSProfile(depth=2))
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/depth/camera_info", self.on_depth_info, qos_profile_sensor_data)
        self.create_service(SetBool, "/autonomy/enable", self.on_enable)
        self.create_service(Trigger, "/autonomy/reset", self.on_reset)
        self.create_timer(1 / float(self.settings["control_rate_hz"]), self.control)
        self.get_logger().info(
            "RGB 빨강→초록 확인을 기다립니다. D435i급 깊이와 RGB만 사용하며 "
            "/scan·지도·정답 위치는 읽지 않습니다."
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_depth_info(self, message):
        fx, fy, cx, cy = float(message.k[0]), float(message.k[4]), float(message.k[2]), float(message.k[5])
        if all(math.isfinite(value) for value in (fx, fy, cx, cy)) and fx > 0 and fy > 0:
            self.depth_info = (fx, fy, cx, cy)

    def on_depth(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if stamp <= self.last_depth_processed or self.depth_info is None:
            return
        # 제어 주기보다 훨씬 빠른 프레임은 버려 CPU 부하만 늘리지 않습니다.
        if stamp - self.last_depth_processed < .045:
            return
        self.last_depth_processed = stamp
        try:
            fx, fy, cx, cy = self.depth_info
            self.depth = image_depth_m(message)
            self.points, self.depth_valid_count = depth_wall_points(
                self.depth, fx, fy, cx, cy,
                camera_xyz=(self.settings["camera_x_m"], self.settings["camera_y_m"],
                            self.settings["camera_z_m"]),
                stride=int(self.settings["depth_stride_px"]),
                minimum_height=self.settings["wall_minimum_height_m"],
                maximum_height=self.settings["wall_maximum_height_m"],
                minimum_forward=self.settings["wall_minimum_forward_m"],
                maximum_forward=self.settings["wall_maximum_forward_m"],
            )
            self.depth_stamp = stamp
            self.depth_wall_time = time.monotonic()
        except (ValueError, TypeError, BufferError):
            self.points = None
            self.depth = None
            self.depth_valid_count = 0

    def on_image(self, message):
        stamp = stamp_seconds(message.header.stamp)
        if stamp < self.last_image_processed:
            return
        if stamp - self.last_image_processed < .045:
            return
        self.last_image_processed = stamp
        self.rgb_stamp = stamp
        self.image_wall_time = time.monotonic()
        rgb = image_rgb(message)
        self.rgb = rgb
        self.signal.update(rgb)
        self.ids = marker_ids(rgb)
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
        self.points = None
        self.depth = None
        self.rgb = None
        self.depth_valid_count = 0
        self.depth_stamp = None
        self.depth_wall_time = 0.0
        self.ids = []
        self.last_steering = 0.0
        self.last_depth_processed = -math.inf
        self.last_image_processed = -math.inf
        self.rgb_stamp = None
        self.image_wall_time = 0.0
        self.last_status_time = -math.inf
        self.last_lateral_correction = 0.0
        self.last_lateral_clearance_time = -math.inf
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
        status = {
            "state": "WAIT_GREEN", "side": self.side, "signal": self.signal.observed,
            "marker_ids": self.ids, "started": self.signal.started, "enabled": self.enabled,
            "sim_time_s": now, "perception_source": "d435i_rgbd",
            "depth_wall_points": self.depth_valid_count,
        }
        speed, steering = 0.0, self.last_steering
        depth_fresh = (self.depth is not None and self.depth_stamp is not None and
                       0 <= now - self.depth_stamp < self.settings["depth_timeout_s"])
        image_fresh = self.rgb_stamp is not None and 0 <= now - self.rgb_stamp < self.settings["image_timeout_s"]
        transport_fresh = (time.monotonic() - self.depth_wall_time < 3 and
                           time.monotonic() - self.image_wall_time < 3)
        if not self.enabled:
            status["state"] = "DISABLED"
        elif not (depth_fresh and image_fresh and transport_fresh):
            status["state"] = "SENSOR_STOP"
        elif self.signal.started:
            fx, _, cx, cy = self.depth_info
            speed, steering, details = depth_gap_command(
                self.depth, fx, cx, cy, self.side,
                self.settings["max_steering_angle_rad"],
                self.settings["max_speed_mps"], self.settings["min_speed_mps"],
                maximum_range=self.settings["wall_maximum_forward_m"] + 2.55,
                free_clearance=self.settings["gap_free_clearance_m"],
                row_half_height=int(self.settings["gap_row_half_height_px"]),
                column_stride=int(self.settings["depth_stride_px"]),
            )
            wall_clearance = depth_lateral_clearance(self.points, self.side)
            gap_steering = steering
            wall_correction = 0.0
            if wall_clearance is not None and speed > 0:
                side_sign = 1.0 if self.side == "left" else -1.0
                wall_correction = (side_sign * self.settings["lateral_clearance_gain"] *
                                   (wall_clearance - self.settings["target_wall_distance_m"]))
                wall_correction = max(-self.settings["max_steering_angle_rad"],
                                      min(self.settings["max_steering_angle_rad"], wall_correction))
                self.last_lateral_correction = wall_correction
                self.last_lateral_clearance_time = now
                details.update(wall_clearance_m=wall_clearance)
            elif speed > 0 and now - self.last_lateral_clearance_time <= self.settings["lateral_clearance_hold_s"]:
                wall_correction = self.last_lateral_correction
                details["lateral_clearance_held"] = True
            if speed > 0:
                target, road_details = road_target(
                    self.rgb, self.depth,
                    (self.settings["color_fx_px"], self.settings["color_fy_px"],
                     self.settings["color_cx_px"], self.settings["color_cy_px"]),
                    self.depth_info,
                    (self.settings["camera_x_m"], self.settings["camera_y_m"],
                     self.settings["camera_z_m"]),
                    self.settings["road_width_m"], self.settings["road_target_x_m"],
                )
                details.update(road_details)
                if target is not None:
                    steering = road_steering(target, self.settings["wheelbase_m"],
                                             self.settings["max_steering_angle_rad"])
                    speed = min(speed, self.settings["max_speed_mps"] / (1 + 3 * abs(steering)))
                    details["steering_source"] = "rgbd_road_pure_pursuit"
                else:
                    # 노면 표식/가림으로 가까운 목표점이 없을 때만 기존 깊이
                    # 통로·벽 보정을 최저속도로 사용합니다. 두 조향기를 더하지 않습니다.
                    near_correction = float(details.get("near_avoidance_rad", 0.0))
                    steering = max(-self.settings["max_steering_angle_rad"],
                                   min(self.settings["max_steering_angle_rad"],
                                       self.settings["gap_steering_weight"] * gap_steering +
                                       wall_correction + near_correction))
                    speed = min(speed, self.settings["min_speed_mps"])
                    details["steering_source"] = "depth_gap_fallback"
                details.update(gap_steering_rad=gap_steering,
                               wall_clearance_correction_rad=wall_correction)
            status.update(details)
            curvature = abs(math.tan(steering) / self.settings["wheelbase_m"])
            if curvature > 1e-6:
                speed = min(speed, math.sqrt(self.settings["lateral_acceleration_limit_mps2"] / curvature))
            status["state"] = "RUNNING" if speed > 0 else details["reason"].upper()
        dt = 1 / self.settings["control_rate_hz"]
        speed = min(speed, self.last_speed + self.settings["acceleration_mps2"] * dt)
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
    run_node(StereoWallFollow, args=args)
