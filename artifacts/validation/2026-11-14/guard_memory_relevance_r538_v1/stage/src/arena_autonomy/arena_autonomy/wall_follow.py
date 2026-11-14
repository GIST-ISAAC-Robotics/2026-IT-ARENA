"""카메라 출발 신호와 라이다만 사용하는 반복 본선 주행 노드."""
import json
import math
import time

from arena_autonomy.vehicle_geometry import float32_limit_not_above

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from arena_vehicle_interface.node_lifecycle import run_node
from arena_vehicle_interface.timing_probe import TimingProbe
from arena_vehicle_interface.queued_log import QueuedLog
from arena_autonomy.core import StartSignal, follow_command, image_rgb, marker_ids, scan_points
from arena_autonomy.lidar_motion import MODES, MotionUnavailable
from arena_autonomy.lidar_motion_ros import MotionInput


def stamp_seconds(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class WallFollow(Node):
    def __init__(self, node_name="wall_follow"):
        super().__init__(node_name)
        from arena_autonomy.execution_environment import configure
        configure(self)
        self.timing = TimingProbe(self)
        defaults = {"wheelbase_m": .145, "lidar_x_m": -.03, "max_steering_angle_rad": .43,
                    "target_wall_distance_m": .425, "max_speed_mps": .35, "min_speed_mps": .14,
                    "scan_timeout_s": .45, "image_timeout_s": 1.0, "control_rate_hz": 20.0,
                    "acceleration_mps2": .5, "lateral_acceleration_limit_mps2": 3.0,
                    "sensor_wall_timeout_s": 3.0}
        self.settings = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        motion_mode = str(self.declare_parameter("lidar_compensation", "none").value)
        if motion_mode not in MODES:
            raise ValueError("unknown lidar_compensation")
        self.motion = MotionInput(self) if motion_mode != "none" else None
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
        self.state_log = QueuedLog(self.get_logger().info)
        prefix = '/hardware_preview' if self.hardware_shadow else ''
        self.publisher = self.create_publisher(AckermannDriveStamped, prefix+"/drive", 10)
        if self.hardware_shadow and self.publisher.topic_name != prefix+'/drive':
            raise ValueError('hardware shadow output must remain on its diagnostic topic')
        self.status_publisher = self.create_publisher(String, prefix+"/autonomy/status", 10)
        from arena_autonomy.native_lidar_input import subscribe_scan
        subscribe_scan(self,self.timing.wrap('scan',self.on_scan),qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/color/image_raw", self.timing.wrap('image', self.on_image), QoSProfile(depth=2))
        self.create_service(SetBool, prefix+"/autonomy/enable", self.on_enable)
        self.create_service(Trigger, prefix+"/autonomy/reset", self.on_reset)
        self.create_timer(1 / self.settings["control_rate_hz"], self.timing.wrap('control', self.control))
        self.get_logger().info(f"RGB 빨강→초록 대기. lidar_compensation={motion_mode}; 보정 시 엔코더·자이로 추가, 지도·정답 위치 미사용.")

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
        rgb = self.timing.call('image_decode', image_rgb, message)
        self.timing.call('signal_detection', self.signal.update, rgb)
        self.ids = self.timing.call('marker_detection', self.detect_markers, rgb)
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

    def detect_markers(self, rgb):
        return marker_ids(rgb)

    def on_enable(self, request, response):
        self.enabled = bool(request.data)
        if not self.enabled:
            self.stop()
        response.success = True
        response.message = "주행 허용" if self.enabled else "사용자 정지"
        return response

    def on_reset(self, request, response):
        self.stop()
        if self.motion:
            self.motion.history.reset()
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
        emit_command = True
        scan_fresh = self.scan is not None and 0 <= now - stamp_seconds(self.scan.header.stamp) < self.settings["scan_timeout_s"]
        image_fresh = self.rgb_stamp is not None and 0 <= now - self.rgb_stamp < self.settings["image_timeout_s"]
        wall_timeout = self.settings.get("sensor_wall_timeout_s", 3.0)
        wall_fresh = time.monotonic() - self.scan_wall_time < wall_timeout and time.monotonic() - self.image_wall_time < wall_timeout
        if not self.enabled:
            status["state"] = "DISABLED"
        elif not (scan_fresh and image_fresh and wall_fresh):
            status["state"] = "SENSOR_STOP"
        # 기본은 시각 출발 사실만 본다. 선택형 경기 감독이 있을 때만 그 판정을 쓴다.
        elif self.signal.started if getattr(self, 'race', None) is None else self.motion_gate(status):
            try:
                if self.motion:
                    points, valid_count = self.timing.call('motion_projection', self.motion.project, self.scan)
                    status["motion"] = self.motion.last_meta
                else:
                    points, valid_count = scan_points(self.scan.ranges, self.scan.angle_min, self.scan.angle_increment,
                                                      self.scan.range_min, self.scan.range_max, self.settings["lidar_x_m"])
            except MotionUnavailable:
                points, valid_count = None, 0
                status["motion"] = self.motion.last_meta
            if valid_count < 30:
                status["state"] = "MOTION_STOP" if self.motion and self.motion.last_meta.get("reason") != "ok" else "SCAN_INVALID"
            else:
                command = self.timing.call('path_command', self.command, points)
                if command is None:
                    # 선택형 분리 계산: 대기 중 이전 명령의 timestamp를 갱신하지 않는다.
                    emit_command = False
                    speed, steering = self.last_speed, self.last_steering
                    status["state"] = "RUNNING" if speed > 0 else "DECISION_PENDING"
                    status["decision_pending"] = True
                else:
                    speed, steering, details = command
                    status.update(details)
                    curvature = abs(math.tan(steering) / self.settings["wheelbase_m"])
                    if curvature > 1e-6:
                        speed = min(speed, math.sqrt(self.settings["lateral_acceleration_limit_mps2"] / curvature))
                    status["state"] = "RUNNING" if speed > 0 else details["reason"].upper()
        dt = getattr(self, 'decision_output_dt_s', 1 / self.settings["control_rate_hz"])
        # 증속은 완만하게, 위험·센서 누락 시에는 즉시 0 명령을 내립니다.
        self.command_was_published = emit_command
        if emit_command:
            speed = min(speed, self.last_speed + self.settings.get("acceleration_mps2", .5) * dt)
            if getattr(getattr(self, 'race', None), 'development_speed_limit', None) is not None:
                status['development_requested_speed_mps'] = speed
                speed = self.race.limit_speed(speed)
                status['development_speed_limited'] = speed != status['development_requested_speed_mps']
            steering = max(self.last_steering - 3 * dt, min(self.last_steering + 3 * dt, steering))
            self.last_speed, self.last_steering = speed, steering
            message = AckermannDriveStamped()
            reference = getattr(self, 'decision_command_stamp_s', None)
            message.header.stamp = (self.get_clock().now().to_msg() if reference is None else
                rclpy.time.Time(nanoseconds=round(reference * 1e9)).to_msg())
            message.header.frame_id = "base_link"
            message.drive.speed, message.drive.steering_angle = (float32_limit_not_above(speed) if speed > 0 else float(speed)), float(steering)
            self.timing.call('command_publish', self.publisher.publish, message)
        status.update(speed_command_mps=speed, steering_command_rad=steering)
        if getattr(self, 'hardware_shadow', False):
            status.update(execution_environment='hardware_shadow_v1', drive_output=False,
                          motion_timing_verified=bool(self.motion and self.motion.verified),
                          **({'motion_time_policy': self.motion.timing_policy}
                             if self.motion and getattr(self.motion,'timing_policy','verified_only')!='verified_only' else {}))
        if getattr(self, 'hardware_runtime', False):
            status.update(execution_environment=self.execution_environment,
                          motion_timing_verified=False, motion_time_policy=self.motion.timing_policy,
                          command_requires_independent_guard=True, physical_actuation_verified=False)
        status['text_log'] = self.state_log.snapshot()
        if getattr(self, 'vision_pipeline', None) is not None:
            status['vision'] = self.vision_pipeline.snapshot()
            observer = getattr(self.vision_pipeline, 'vehicle_observer', None)
            if observer is not None:
                status['vehicle_markers'] = observer.snapshot(now)
        if getattr(self, 'race', None) is not None:
            status['race'] = self.race.snapshot(now, time.monotonic())
            status['inputs'] = self.input_snapshot(now)
        if now - self.last_status_time > .2 or status["state"] != self.last_status:
            payload = self.timing.call('status_json', json.dumps, status, ensure_ascii=False, allow_nan=False)
            self.timing.call('status_publish', self.status_publisher.publish, String(data=payload))
            self.last_status_time = now
        if status["state"] != self.last_status:
            self.timing.call('state_log', self.state_log.submit,
                             f"t={now:.6f} {status['state']} / {self.side} 벽 / {self.signal.observed}")
            self.last_status = status["state"]

    def destroy_node(self):
        self.stop()
        self.state_log.close()
        return super().destroy_node()

    def command(self, points):
        return follow_command(points, self.side, self.settings["target_wall_distance_m"],
                              self.settings["wheelbase_m"], self.settings["max_steering_angle_rad"],
                              self.settings["max_speed_mps"], self.settings["min_speed_mps"])


def main(args=None):
    run_node(WallFollow, args=args)
