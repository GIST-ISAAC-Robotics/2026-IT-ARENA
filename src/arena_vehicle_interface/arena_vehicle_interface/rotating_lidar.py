"""고빈도 실제 광선 프레임에서 회전 순서대로 취득하는 시뮬레이션 어댑터.

점별 위치 보정은 하지 않는다. 각 점은 직전 광선 프레임에서 취득하므로
시간 근사는 max_source_gap_s 이하이며, 그보다 긴 누락은 회전 전체를 폐기한다.
"""
import copy
import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json

from arena_vehicle_interface.node_lifecycle import run_node


class RevolutionAssembler:
    def __init__(self, rate=10., samples=500, max_gap=.0021):
        if not math.isfinite(rate) or rate <= 0 or samples < 2 or max_gap <= 0:
            raise ValueError("회전 주기·점 수·허용 간격이 잘못되었습니다.")
        self.period = 1. / rate
        self.samples = samples
        self.increment = self.period / samples
        self.max_gap = max_gap
        self.previous = None
        self.start = None
        self.ranges = []
        self.errors = []
        self.discarded = 0

    def feed(self, stamp, ranges):
        if len(ranges) != self.samples or not math.isfinite(stamp):
            raise ValueError("입력 광선 개수 또는 시각 불일치")
        output = []
        if self.previous is None or stamp < self.previous[0]:
            self.previous = (stamp, list(ranges))
            self.start = stamp
            self.ranges, self.errors = [], []
            return output
        previous_stamp, previous_ranges = self.previous
        if stamp == previous_stamp:
            return output
        if stamp - previous_stamp > self.max_gap:
            self.discarded += 1
            self.previous = (stamp, list(ranges))
            self.start = stamp
            self.ranges, self.errors = [], []
            return output
        # 아직 도착하지 않은 미래 프레임을 보간에 쓰지 않는다.
        while self.start + len(self.ranges) * self.increment < stamp - 1e-10:
            index = len(self.ranges)
            if index == self.samples:
                break
            when = self.start + index * self.increment
            self.ranges.append(previous_ranges[index])
            self.errors.append(max(0., when - previous_stamp))
        if len(self.ranges) == self.samples and stamp + 1e-10 >= self.start + self.period:
            output.append((self.start, self.ranges, max(self.errors)))
            self.start += self.period
            self.ranges, self.errors = [], []
        self.previous = (stamp, list(ranges))
        return output


class RotatingLidar(Node):
    def __init__(self):
        super().__init__("rotating_lidar")
        rate = float(self.declare_parameter("rotation_rate_hz", 10.).value)
        count = int(self.declare_parameter("samples_per_scan", 500).value)
        self.mode = str(self.declare_parameter("acquisition", "sequential").value)
        if self.mode not in ("sequential", "snapshot_matched"):
            raise ValueError("지원하지 않는 취득 방식")
        self.assembler = RevolutionAssembler(rate, count)
        qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.audit = self.create_publisher(String, "/sim/lidar_acquisition", 10)
        self.create_subscription(LaserScan, "/sim/lidar_instantaneous", self.on_scan, qos)

    def on_scan(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        for first, ranges, error in self.assembler.feed(stamp, message.ranges):
            scan = copy.deepcopy(message)
            sequential = self.mode == "sequential"
            ns = round((first if sequential else stamp) * 1e9)
            scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(ns, 1_000_000_000)
            scan.ranges = ranges if sequential else message.ranges
            scan.intensities = []
            scan.scan_time = self.assembler.period if sequential else 0.0
            scan.time_increment = self.assembler.increment if sequential else 0.0
            self.publisher.publish(scan)
            self.audit.publish(String(data=json.dumps({
                "first_ray_stamp_s": first, "source_stamp_at_publish_s": stamp,
                "last_ray_stamp_s": first + (self.assembler.samples - 1) * self.assembler.increment,
                "max_capture_time_error_s": error,
                "discarded_source_gaps": self.assembler.discarded,
                "model": self.mode,
            })))


def main(args=None):
    run_node(RotatingLidar, args=args)
