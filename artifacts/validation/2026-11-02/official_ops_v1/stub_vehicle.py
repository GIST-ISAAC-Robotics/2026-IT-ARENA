#!/usr/bin/env python3
"""브라우저 배선 시험용 대역 차량. 실제 RaceSupervisor를 쓰고 센서·MCU만 흉내 낸다. Gazebo 없음.

주행·정지 성능의 증거가 아니다. 버튼 → HTTP → /race/command·MCU 서비스 → 표시 연결만 본다.
"""
import json
import sys
import time
import types

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from arena_autonomy.race_session import RaceSupervisor


class Vision:
    def __init__(self):
        self.epoch, self.fault = 0, None
        self.signal = types.SimpleNamespace(started=False)

    def reset(self):
        self.epoch += 1

    def begin_input_boundary(self, boundary):
        self.epoch += 1


class Stub(Node):
    def __init__(self):
        super().__init__('stub_vehicle')
        self.vision = Vision()
        self.race = RaceSupervisor(self.vision, now_s=time.monotonic(), session='stub'+str(int(time.time())))
        self.boundary = None
        self.mcu = dict(v=1, boot='stubboot', generation=0, state='DISARMED', reason='boot', estop_held=False,
                        target_speed_mps=0., mean_wheel_mps=0., feedback_valid=True, stationary_ready=True,
                        output_ready=True)
        self.status = self.create_publisher(String, '/autonomy/status', 20)
        self.actuation = self.create_publisher(String, '/actuation/status', 20)
        self.result = self.create_publisher(String, '/race/command_result', 20)
        self.create_subscription(String, '/race/command', self.on_command, 20)
        self.create_subscription(String, '/race/operator_link', lambda m: self.race.operator_link(m.data, time.monotonic()), 20)
        for name, kind in (('stop', 'STOP'), ('clear_stop', 'CLEAR'), ('arm', 'ARM'), ('disarm', 'DISARM')):
            self.create_service(Trigger, '/actuation/'+name, lambda _, r, k=kind: self.on_service(k, r))
        self.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.count = 0
        self.green_after = time.monotonic()+float(sys.argv[1]) if len(sys.argv) > 1 else None

    def on_service(self, kind, response):
        state = self.mcu['state']
        change = dict(STOP='ESTOP_LATCHED', CLEAR='DISARMED', ARM='ARMED', DISARM='DISARMED')[kind]
        valid = kind == 'STOP' or (kind == 'CLEAR' and state in ('ESTOP_LATCHED', 'FAULT_LATCHED')) or \
            (kind == 'ARM' and state == 'DISARMED') or (kind == 'DISARM' and state in ('ARMED', 'ACTIVE'))
        if valid:
            self.mcu.update(state=change, reason=kind.lower(), generation=self.mcu['generation']+1)
        response.success, response.message = valid, kind if valid else 'not_ready'
        return response

    def on_command(self, message):
        payload = json.loads(message.data)
        result = self.race.handle_command(payload, time.monotonic(), time.monotonic())
        if result['accepted'] and result['op'] in ('recovery_reset', 'resume_start', 'restore_race'):
            self.boundary = result['boundary_s']
        self.result.publish(String(data=json.dumps(result)))

    def tick(self):
        now, wall = time.monotonic(), time.monotonic()
        self.count += 1
        status = dict(self.mcu, now_us=round(now*1e6))
        self.actuation.publish(String(data=json.dumps(status)))
        self.race.observe_mcu(status, now, wall)
        if self.green_after and now > self.green_after and not self.race.permission.started and self.race.permission.latch is None:
            self.vision.signal.started = True
            self.race.observe_vision(dict(stamp=now-.01), self.vision.epoch, now)
        if self.race.resume is not None:
            ok = now-self.race.resume['boundary_s'] > .4
            self.race.advance_resume(now, wall, inputs=dict(scan=ok, rgb=ok, gyro=ok, encoder=ok),
                                     route=dict(ready=ok, reason='stub', planned_speed_mps=1.) if ok else None)
        if self.count % 10:
            return
        allowed = self.race.decide(sensors_ready=True, now_s=now, wall_s=wall)['allowed']
        self.status.publish(String(data=json.dumps(dict(state='RUNNING' if allowed else 'RACE_LATCHED', side='right',
            reason='stub', behavior='FOLLOW', marker_ids=[], sim_time_s=now, speed_command_mps=1. if allowed else 0.,
            inputs=dict(age_s=dict(scan=.05, rgb=.05, gyro=.005, encoder=.005), side_marker=30),
            race=self.race.snapshot(now, wall)))))


rclpy.init(args=['--ros-args'])
node = Stub()
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
