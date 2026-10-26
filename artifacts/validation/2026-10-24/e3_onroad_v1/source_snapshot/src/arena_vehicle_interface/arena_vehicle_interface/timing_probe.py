"""선택형 콜백 계측. 기본 비활성이며 콜백 벽시계 소요시간과 센서 나이를 구분한다."""
import json
import hashlib
from pathlib import Path
import time

from std_msgs.msg import String
from std_srvs.srv import Trigger


class TimingProbe:
    def __init__(self, node):
        self.node = node
        enabled = bool(node.declare_parameter('timing_probe', False).value)
        self.mode = node.declare_parameter('timing_probe_mode', 'topic').value
        self.capacity = node.declare_parameter('timing_probe_capacity', 100000).value
        self.buffer_path = node.declare_parameter('timing_probe_buffer_path', '').value
        if self.mode not in ('topic', 'buffered'):
            raise ValueError('Unknown timing_probe_mode')
        if type(self.capacity) is not int or not 1 <= self.capacity <= 100000:
            raise ValueError('timing_probe_capacity must be within 1..100000')
        if enabled and self.mode == 'buffered' and not Path(self.buffer_path).is_absolute():
            raise ValueError('Buffered timing needs an absolute export path')
        self.records = []
        self.dropped = 0
        self.frozen = False
        self.publisher = node.create_publisher(String, '/diagnostics/timing', 100) if enabled else None
        self.previous = {}
        self.completed = {}
        self.active_stages = None
        self.snapshot_service = node.create_service(Trigger, '~/timing_snapshot', self.snapshot) if enabled else None
        self.export_service = (node.create_service(Trigger, '~/timing_export', self.export)
                               if enabled and self.mode == 'buffered' else None)

    def export(self, request, response):
        """단일 executor의 EOF 체크포인트에서만 저장한다. 콜백 중 파일/JSON 작업 없음.

        이후 콜백은 이 체크포인트의 측정 범위 밖이다. 기존 파일은 덮어쓰지 않는다.
        """
        response.success = False
        if self.frozen:
            response.message = 'Timing export already attempted'
            return response
        self.frozen = True
        completed = dict(self.completed)
        try:
            digest = hashlib.sha256()
            with Path(self.buffer_path).open('xb') as stream:
                for row in self.records:
                    payload = (json.dumps(row, allow_nan=False)+'\n').encode('utf-8')
                    stream.write(payload)
                    digest.update(payload)
            result = json.loads(self.snapshot(None, type(response)()).message)
            result.update(completed=completed, timing_export=dict(
                mode=self.mode, path=self.buffer_path, rows=len(self.records),
                sha256=digest.hexdigest(), dropped=self.dropped, capacity=self.capacity))
            response.message = json.dumps(result)
            response.success = self.dropped == 0
            self.records.clear()
        except Exception as error:
            response.message = repr(error)
        return response

    def snapshot(self, request, response):
        """진단 토픽 손실과 실제 콜백 손실을 구분하기 위한 누적 완료 횟수."""
        response.success = True
        result = dict(node=self.node.get_name(), completed=dict(self.completed))
        if hasattr(self.node, 'state_log'):
            result['text_log'] = self.node.state_log.snapshot()
        if getattr(self.node, 'vision_pipeline', None) is not None:
            result['vision'] = self.node.vision_pipeline.snapshot()
        response.message = json.dumps(result)
        return response

    def call(self, name, callback, *args, **kwargs):
        """현재 계측 콜백 내부 단계만 측정한다. 추가 ROS 발행/제어 변경 없음."""
        if self.active_stages is None:
            return callback(*args, **kwargs)
        began = time.perf_counter_ns()
        try:
            return callback(*args, **kwargs)
        finally:
            stage = self.active_stages.setdefault(name, {'count': 0, 'duration_ms': 0.})
            stage['count'] += 1
            stage['duration_ms'] += (time.perf_counter_ns()-began)/1e6

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
            parent_stages, stages = self.active_stages, {}
            self.active_stages = stages
            try:
                return callback(*args)
            finally:
                end = time.perf_counter_ns()
                self.active_stages = parent_stages
                self.completed[name] = self.completed.get(name, 0)+1
                row = {
                    'node': self.node.get_name(), 'callback': name, 'receive_ros_ns': ros_ns,
                    'sequence': self.completed[name],
                    'stages': stages,
                    'source_ns': source, 'source_age_ms': None if source is None else (ros_ns-source)/1e6,
                    'begin_monotonic_ns': began, 'end_monotonic_ns': end, 'duration_ms': (end-began)/1e6,
                    'wall_interval_ms': None if previous is None else (began-previous)/1e6,
                }
                if self.mode == 'topic':
                    self.publisher.publish(String(data=json.dumps(row, allow_nan=False)))
                elif not self.frozen:
                    if len(self.records) < self.capacity:
                        self.records.append(row)
                    else:
                        self.dropped += 1
        return measured
