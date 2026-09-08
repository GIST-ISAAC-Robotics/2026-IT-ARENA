"""벽 추종기와 독립 시험이 공유하는 엔코더/자이로 입력 어댑터."""
import math
import time

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

from arena_autonomy.lidar_motion import MODES, MotionConfig, MotionHistory, MotionUnavailable


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
        self.left = str(node.declare_parameter("motion_left_joint", "rear_left_wheel_joint").value)
        self.right = str(node.declare_parameter("motion_right_joint", "rear_right_wheel_joint").value)
        self.last_clock = None
        self.counts = {"wheels": 0, "gyro": 0, "rejected": 0, "resets": 0}
        self.last_meta = {}
        # 독립 비교의 none도 같은 입력 부하를 받는다. 일반 none에서는 생성하지 않아도 된다.
        node.create_subscription(JointState, "/wheel_states", self.on_wheels, 100)
        node.create_subscription(Imu, "/camera/imu", self.on_imu, qos_profile_sensor_data)

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
            li = next(i for i, n in enumerate(msg.name) if n == self.left or n.endswith("::" + self.left))
            ri = next(i for i, n in enumerate(msg.name) if n == self.right or n.endswith("::" + self.right))
            values = [float(msg.velocity[li]), float(msg.velocity[ri])]
        except (StopIteration, IndexError):
            self.counts["rejected"] += 1
            return
        accepted = stamp <= now + .1 and self.history.add_wheels(stamp, *values)
        self.capture("wheels", stamp, values, accepted)

    def on_imu(self, msg):
        now = self.clock()
        stamp = seconds(msg.header.stamp)
        values = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        yaw = sum(a * v for a, v in zip(self.axis, values))
        accepted = (msg.angular_velocity_covariance[0] != -1 and stamp <= now + .1
                    and self.history.add_gyro(stamp, yaw))
        self.capture("gyro", stamp, values, accepted)

    def project(self, scan):
        now = self.clock()
        began = time.perf_counter()
        try:
            if scan.header.frame_id != "laser_frame":
                raise MotionUnavailable("scan_frame_mismatch")
            points, indices, meta = self.history.project(
                scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max,
                seconds(scan.header.stamp), float(scan.time_increment), now, self.mode, self.verified)
            self.last_meta = {**meta, "reason": "ok", "processing_ms": (time.perf_counter() - began) * 1000}
            return points, len(indices)
        except MotionUnavailable as error:
            self.last_meta = {"mode": self.mode, "reason": str(error),
                              "processing_ms": (time.perf_counter() - began) * 1000}
            raise
