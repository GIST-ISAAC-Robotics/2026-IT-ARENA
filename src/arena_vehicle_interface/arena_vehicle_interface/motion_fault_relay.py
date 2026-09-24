"""시험 입력만 중계: 공개 토픽은 두 소비자에게 동일, 원래 취득 시각 유지.

stamp_offset_s만 명시적인 잘못된 시계 시험이다. /sim 정답을 읽지 않는다.
gyro z의 bias/gain, 양쪽 휠 gain, FIFO 전송 지연과 명시 구간 누락을 지원한다.
"""
from collections import deque
import copy
import json
import math
from pathlib import Path
import random

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState

from arena_vehicle_interface.motion_impairment import MotionFaultConfig
from arena_vehicle_interface.node_lifecycle import run_node


def stamp_s(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def alter_message(kind, original, fault):
    msg = copy.deepcopy(original)
    stamp = stamp_s(original)
    if kind == 'imu':
        before = [original.angular_velocity.z]
        msg.angular_velocity.z = fault.transform(before[0], stamp)
        after = [msg.angular_velocity.z]
    else:
        targets = {'rear_left_wheel_joint', 'rear_right_wheel_joint'}
        indices = [i for i, name in enumerate(msg.name) if name.split('::')[-1] in targets]
        if len(indices) != 2 or any(i >= len(msg.velocity) for i in indices):
            raise ValueError('both named wheel velocities are required')
        before = [original.velocity[i] for i in indices]
        for i in indices:
            msg.velocity[i] = fault.transform(original.velocity[i], stamp)
            if i < len(msg.position):
                msg.position[i] = fault.transform(original.position[i], stamp)
        after = [msg.velocity[i] for i in indices]
    ns = round(fault.claimed_stamp(stamp) * 1e9)
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(ns, 1_000_000_000)
    return msg, before, after


class MotionFaultRelay(Node):
    def __init__(self):
        super().__init__('motion_fault_relay')
        profile = Path(str(self.declare_parameter('profile', '').value))
        self.config = MotionFaultConfig.from_dict(json.loads(profile.read_text()))
        audit = str(self.declare_parameter('audit_path', '').value)
        self.audit = Path(audit).open('x', encoding='utf-8', buffering=1) if audit else None
        self.emit({'event': 'config', 'config': self.config.as_dict()})
        self.pending = {kind: deque() for kind in ('imu', 'wheels')}
        self.rng = {kind: random.Random(self.config.seed + i) for i, kind in enumerate(self.pending)}
        self.sequence = 0
        self.last_clock = None
        self.outputs = {
            'imu': self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data),
            'wheels': self.create_publisher(JointState, '/wheel_states', 100)}
        self.create_subscription(Imu, '/test/raw_imu', lambda msg: self.capture('imu', msg), qos_profile_sensor_data)
        self.create_subscription(JointState, '/test/raw_wheels', lambda msg: self.capture('wheels', msg), 100)
        self.create_timer(.002, self.release)

    def emit(self, record):
        if self.audit:
            self.audit.write(json.dumps(record, allow_nan=False) + '\n')

    def clock(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_clock is not None and now < self.last_clock:
            for queue in self.pending.values():
                queue.clear()
            self.emit({'event': 'clock_reset', 'time': now})
        self.last_clock = now
        return now

    def capture(self, kind, original):
        now = self.clock()
        fault = getattr(self.config, kind)
        source_stamp = stamp_s(original)
        self.sequence += 1
        base = {'id': self.sequence, 'stream': kind, 'source_stamp_s': source_stamp, 'received_s': now,
                'fault_active': fault.active(source_stamp)}
        if fault.drop and base['fault_active']:
            self.emit({**base, 'event': 'drop'})
            return
        msg, before, after = alter_message(kind, original, fault)
        delay = fault.delay(source_stamp, self.rng[kind])
        row = {**base, 'event': 'publish', 'claimed_stamp_s': stamp_s(msg), 'delay_s': delay,
               'due_s': now + delay,
               'before': [v if math.isfinite(v) else None for v in before],
               'after': [v if math.isfinite(v) else None for v in after]}
        self.pending[kind].append((msg, row))
        if len(self.pending[kind]) > 10000:
            raise RuntimeError('test relay queue exceeded bound')
        self.release()

    def release(self):
        now = self.clock()
        for kind, queue in self.pending.items():
            # FIFO: jitter/window end may delay a later packet, never reorder it implicitly.
            while queue and queue[0][1]['due_s'] <= now + 1e-9:
                msg, row = queue.popleft()
                self.outputs[kind].publish(msg)
                self.emit({**row, 'released_s': now})

    def destroy_node(self):
        if self.audit:
            self.emit({'event': 'close', 'pending': {k: len(q) for k, q in self.pending.items()}})
            self.audit.close()
            self.audit = None
        return super().destroy_node()


def main(args=None):
    run_node(MotionFaultRelay, args=args)
