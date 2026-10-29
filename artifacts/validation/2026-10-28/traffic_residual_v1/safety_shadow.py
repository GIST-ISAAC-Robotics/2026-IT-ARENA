"""저장 판단 시각의 보호 함수 대조. ROS 수신/타이머 실행을 재현하지 않는다."""
import json
import math
import time
from types import SimpleNamespace

from ackermann_msgs.msg import AckermannDriveStamped
from builtin_interfaces.msg import Time

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion_ros import MotionInput
from arena_autonomy.lidar_safety import LidarSafety


class ShadowSafety:
    def __init__(self, policy, config, rear_half_angle):
        self.now = 0.
        self.output = None
        self.report = None
        self.node = SimpleNamespace(
            scan=None, command=None, last_clock=None, scan_wall=0., command_wall=0.,
            wall_timeout=3., limit=config.max_steering_rad,
            wheelbase=config.wheelbase_m, length=config.length_m, width=config.width_m,
            rear_blind_half_angle=rear_half_angle, recent_observation_enabled=False,
            observation_cache=None, observation_epoch=0, previous_safe_steering=0.,
            partial_policy=policy,
            publisher=SimpleNamespace(publish=self._output),
            status=SimpleNamespace(publish=self._report),
            get_clock=lambda: SimpleNamespace(now=self._clock))
        self.motion = SimpleNamespace(clock=lambda: self.now, mode='both', verified=True,
                                      gyro_rejections={}, last_gyro_rejection=None)
        self.motion.project = lambda scan: MotionInput.project(self.motion, scan)
        self.node.motion = self.motion

    def _clock(self):
        ns = round(self.now * 1e9)
        return SimpleNamespace(nanoseconds=ns, to_msg=lambda: Time(
            sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000))

    def _output(self, message):
        self.output = (float(message.drive.speed), float(message.drive.steering_angle))

    def _report(self, message):
        self.report = json.loads(message.data)

    def step(self, recorded_input, motion_config, command):
        # 제어기의 관측 기억/해제 판단을 받지 않는다. 별도 history와 정책 상태다.
        data, scan, history = restore_call(recorded_input, motion_config)
        self.now = float(data['reference_s'])
        if not math.isfinite(self.now):
            raise ValueError('nonfinite replay time')
        self.motion.history = history
        self.motion.verified = bool(data['timing_verified'])
        message = AckermannDriveStamped()
        message.header.stamp = self._clock().to_msg()
        message.header.frame_id = 'base_link'
        message.drive.speed, message.drive.steering_angle = map(float, command)
        # 저장 입력에는 독립 보호 노드 수신 시각이 없다. 같은 시각에 도착했다고
        # 가정하는 함수 대조이며 실제 transport/queue/ROS 실행 증명이 아니다.
        self.node.command = message
        self.node.scan = scan
        self.node.command_wall = self.node.scan_wall = time.monotonic()
        LidarSafety.control(self.node)
        return self.output, self.report
