"""벽 추종기와 독립 시험이 공유하는 엔코더/자이로 입력 어댑터.

`motion_feedback_mode`가 구동 피드백 표현을 고른다. `drive_motor_shaft`는 모터축
한 채널(/drive_motor/encoder)을 `motion_gear_ratio`로 나눠 평균 바퀴 속도만 얻는다.
기본값 `rear_wheel_pair_legacy`는 모드 매개변수가 없던 옛 기록·시험 재현용이며
현재 launch는 모드를 명시한다. 회전량은 두 모드 모두 IMU 자이로를 사용한다.
"""
import math
import time

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

from arena_autonomy.lidar_motion import MODES, MotionConfig, MotionHistory, MotionUnavailable
from arena_vehicle_interface.drive_feedback import (
    MODE_LEGACY, MODE_MOTOR, TOPIC_BY_MODE, parse_legacy_wheels, parse_motor_state, validate_mode,
)


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class MotionInput:
    def __init__(self, node, mode=None, timing_verified=None, record=None):
        self.node, self.record = node, record
        declared = str((node.get_parameter("lidar_compensation") if node.has_parameter("lidar_compensation")
                        else node.declare_parameter("lidar_compensation", "none")).value)
        self.mode = declared if mode is None else mode
        if self.mode not in MODES:
            raise ValueError("unknown lidar_compensation")
        verified = bool(node.declare_parameter("motion_scan_timing_verified", False).value)
        self.verified = verified if timing_verified is None else timing_verified
        defaults = vars(MotionConfig())
        config = {name: float(node.declare_parameter("motion_" + name, value).value)
                  for name, value in defaults.items()}
        self.history = MotionHistory(MotionConfig(**config))
        # 차체 z축에 투영할 IMU 각속도. R_BI = Rz(yaw) Ry(pitch) Rx(roll).
        roll = float(node.declare_parameter("motion_imu_roll_rad", 0.).value)
        pitch = float(node.declare_parameter("motion_imu_pitch_rad", 0.).value)
        if not math.isfinite(roll) or not math.isfinite(pitch):
            raise ValueError("IMU 장착 각도는 유한해야 합니다.")
        self.axis = (-math.sin(pitch), math.cos(pitch) * math.sin(roll), math.cos(pitch) * math.cos(roll))
        # 현재 LSM6DSOX 구성의 ±500 dps에 맞춘 보수적 입력 경계다. 자동 교정이나
        # 정상 범위 안의 모든 이상치 검출기가 아니다. 원시 각 축을 투영 전에 검사한다.
        self.gyro_abs_limit = float(node.declare_parameter(
            "motion_gyro_abs_limit_rad_s", math.radians(500.)).value)
        if not math.isfinite(self.gyro_abs_limit) or self.gyro_abs_limit <= 0:
            raise ValueError("motion_gyro_abs_limit_rad_s must be finite and positive")
        # 실측 전 임시 이상치 게이트. 정상값을 지연/평활화하지 않으며, 거절한
        # 표본 뒤의 무제한 직전값 유지는 기존 운동 이력의 시간 제한이 막는다.
        self.gyro_slew_limit = float(node.declare_parameter("motion_gyro_slew_limit_rad_s2", 100.).value)
        self.gyro_step_slack = float(node.declare_parameter("motion_gyro_step_slack_rad_s", .05).value)
        if not math.isfinite(self.gyro_slew_limit) or self.gyro_slew_limit <= 0:
            raise ValueError("motion_gyro_slew_limit_rad_s2 must be finite and positive")
        if not math.isfinite(self.gyro_step_slack) or self.gyro_step_slack < 0:
            raise ValueError("motion_gyro_step_slack_rad_s must be finite and nonnegative")
        self.feedback_mode = validate_mode(str(node.declare_parameter("motion_feedback_mode", MODE_LEGACY).value))
        self.gear_ratio = float(node.declare_parameter("motion_gear_ratio", 0.).value)
        if self.feedback_mode == MODE_MOTOR and not (math.isfinite(self.gear_ratio) and self.gear_ratio > 0):
            raise ValueError("drive_motor_shaft 피드백에는 양의 motion_gear_ratio가 필요합니다.")
        # MCU의 기본 비현실 속도 경계와 일치시킨 입력 무결성 검사다. 명령 최고속도나
        # 가속도 제한이 아니며, 거절한 표본을 0으로 대체하거나 시각을 갱신하지 않는다.
        self.max_motor_feedback_mps = float(node.declare_parameter("motion_max_motor_feedback_mps", 5.).value)
        if not math.isfinite(self.max_motor_feedback_mps) or self.max_motor_feedback_mps <= 0:
            raise ValueError("motion_max_motor_feedback_mps must be finite and positive")
        self.left = str(node.declare_parameter("motion_left_joint", "rear_left_wheel_joint").value)
        self.right = str(node.declare_parameter("motion_right_joint", "rear_right_wheel_joint").value)
        self.feedback_topic = TOPIC_BY_MODE[self.feedback_mode]
        self.last_clock = None
        self.counts = {"wheels": 0, "drive_motor": 0, "gyro": 0, "rejected": 0, "resets": 0}
        self.last_meta = {}
        self.gyro_rejections = {}
        self.last_gyro_rejection = None
        # 독립 비교의 none도 같은 입력 부하를 받는다. 일반 none에서는 생성하지 않아도 된다.
        # 계측 이름 'wheels'는 기존 분석 호환용 구동 피드백 콜백 이름이다.
        wrap = node.timing.wrap if hasattr(node, 'timing') else lambda _, callback: callback
        callback = self.on_drive_motor if self.feedback_mode == MODE_MOTOR else self.on_wheels
        node.create_subscription(JointState, self.feedback_topic, wrap('wheels', callback), 100)
        imu_topic = str(node.declare_parameter("motion_imu_topic", "/camera/imu").value)
        node.create_subscription(Imu, imu_topic, wrap('imu', self.on_imu), qos_profile_sensor_data)

    def clock(self):
        now = self.node.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock - 1e-9:
            self.history.reset()
            self.counts["resets"] += 1
        self.last_clock = now
        return now

    def capture(self, kind, stamp, values, accepted):
        accepted = bool(accepted)  # ROS covariance의 numpy.bool_도 JSON bool로 기록
        self.counts[kind if accepted else "rejected"] += 1
        if self.record:
            self.record({"kind": kind, "stamp_s": stamp, "received_s": self.last_clock,
                         "values": [float(v) if math.isfinite(v) else None for v in values], "accepted": accepted})

    def on_wheels(self, msg):
        now = self.clock()
        stamp = seconds(msg.header.stamp)
        try:
            values = list(parse_legacy_wheels(msg.name, msg.velocity, self.left, self.right, require_finite=False))
        except ValueError:
            # 이름/개수 불일치는 거절한다. 비유한 값은 Series가 거절하고 기록에 남긴다.
            self.counts["rejected"] += 1
            return
        accepted = stamp <= now + .1 and self.history.add_wheels(stamp, *values)
        self.capture("wheels", stamp, values, accepted)

    def on_drive_motor(self, msg):
        """모터축 한 채널 → 평균 바퀴 속도. 좌우 값을 만들지 않는다."""
        now = self.clock()
        stamp = seconds(msg.header.stamp)
        try:
            position, motor_rad_s = parse_motor_state(msg.name, msg.position, msg.velocity, require_finite=False)
        except ValueError:
            self.counts["rejected"] += 1
            return
        mean_wheel = motor_rad_s / self.gear_ratio
        # 위치(누적 카운트)가 비유한이면 같은 표본의 속도도 믿지 않는다. 첫 표본의 NaN 속도는
        # 아직 속도 기준이 없다는 뜻이며 Series가 거절·기록한다.
        speed_mps = mean_wheel * self.history.config.wheel_radius_m * self.history.config.wheel_scale
        accepted = (math.isfinite(position) and math.isfinite(speed_mps)
                    and abs(speed_mps) <= self.max_motor_feedback_mps and stamp <= now + .1
                    and self.history.add_mean_wheel(stamp, mean_wheel))
        self.capture("drive_motor", stamp, [motor_rad_s, mean_wheel], accepted)

    def on_imu(self, msg):
        now = self.clock()
        stamp = seconds(msg.header.stamp)
        values = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        yaw = sum(a * v for a, v in zip(self.axis, values))
        reason = None
        if msg.angular_velocity_covariance[0] == -1:
            reason = "gyro_unavailable"
        elif not all(math.isfinite(v) for v in values):
            reason = "gyro_nonfinite"
        elif any(abs(v) >= self.gyro_abs_limit for v in values):
            reason = "gyro_range_or_saturation"
        elif stamp > now + .1:
            reason = "gyro_future"
        elif (self.history.gyro.times and stamp > self.history.gyro.times[-1]
              and abs((yaw - self.history.config.gyro_bias_rad_s) - self.history.gyro.values[-1]) >
              self.gyro_step_slack + self.gyro_slew_limit * (stamp - self.history.gyro.times[-1])):
            reason = "gyro_slew"
        elif not self.history.add_gyro(stamp, yaw):
            reason = "gyro_timestamp"
        accepted = reason is None
        if reason:
            self.gyro_rejections[reason] = self.gyro_rejections.get(reason, 0) + 1
            self.last_gyro_rejection = reason
        # 거절한 값을 clamp/0으로 대체하거나 취득 시각만 갱신하지 않는다.
        # 짧은 공백은 기존 ZOH 한도, 연속 거절은 motion_stale/gap으로 정지한다.
        self.capture("gyro", stamp, values, accepted)

    def project(self, scan):
        now = self.clock()
        began = time.perf_counter()
        diagnostic = {"gyro_rejections": dict(getattr(self, 'gyro_rejections', {})),
                      "last_gyro_rejection": getattr(self, 'last_gyro_rejection', None)}
        try:
            if scan.header.frame_id != "laser_frame":
                raise MotionUnavailable("scan_frame_mismatch")
            points, indices, meta = self.history.project(
                scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                seconds(scan.header.stamp), float(scan.time_increment), now, self.mode, self.verified)
            self.last_meta = {**meta, **diagnostic, "reason": "ok", "processing_ms": (time.perf_counter() - began) * 1000}
            return points, len(indices)
        except MotionUnavailable as error:
            self.last_meta = {"mode": self.mode, **diagnostic, "reason": str(error),
                              "processing_ms": (time.perf_counter() - began) * 1000}
            raise
