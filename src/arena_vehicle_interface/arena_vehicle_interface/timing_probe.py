"""선택형 콜백 계측. 기본 비활성이며 콜백 벽시계 소요시간과 센서 나이를 구분한다."""
import json
import time

from std_msgs.msg import String
from std_srvs.srv import Trigger


class TimingProbe:
    def __init__(self, node):
        self.node = node
        enabled = bool(node.declare_parameter('timing_probe', False).value)
        self.publisher = node.create_publisher(String, '/diagnostics/timing', 100) if enabled else None
        self.previous = {}
        self.completed = {}
        self.snapshot_service = node.create_service(Trigger, '~/timing_snapshot', self.snapshot) if enabled else None

    def snapshot(self, request, response):
        """진단 토픽 손실과 실제 콜백 손실을 구분하기 위한 누적 완료 횟수."""
        response.success = True
        response.message = json.dumps(dict(node=self.node.get_name(), completed=dict(self.completed)))
        return response

    def wrap(self, name, callback):
        if self.publisher is None:
            return callback
        def measured(*args):
            began = time.perf_counter_ns()
            ros_ns = self.node.get_clock().now().nanoseconds
            source = None
            if args and hasattr(args[0], 'header'):
                stamp = args[0].header.stamp
                source = stamp.sec * 1_000_000_000 + stamp.nanosec
            previous = self.previous.get(name)
            self.previous[name] = began
            try:
                return callback(*args)
            finally:
                end = time.perf_counter_ns()
                self.completed[name] = self.completed.get(name, 0)+1
                self.publisher.publish(String(data=json.dumps({
                    'node': self.node.get_name(), 'callback': name, 'receive_ros_ns': ros_ns,
                    'sequence': self.completed[name],
                    'source_ns': source, 'source_age_ms': None if source is None else (ros_ns-source)/1e6,
                    'begin_monotonic_ns': began, 'end_monotonic_ns': end, 'duration_ms': (end-began)/1e6,
                    'wall_interval_ms': None if previous is None else (began-previous)/1e6,
                }, allow_nan=False)))
        return measured
