"""A1-2 실험 전용 ROS 전송: 호스트 / 별도 가상 MCU / Gazebo 출력 감시.

프레임은 ROS byte array로 운반한다. 실제 UART/CAN 펌웨어가 아니며,
가상 MCU의 운동 시계는 /clock, 멎음 감시는 PC 단조 시계다.
"""
from collections import deque
import json
import math
import time
import uuid

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String, UInt8MultiArray
from std_srvs.srv import Trigger

from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig, integer
from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.applied_guard import AppliedGuard
from arena_vehicle_interface.node_lifecycle import run_node


def stamp_us(stamp):
    return stamp.sec * 1_000_000 + stamp.nanosec // 1000


class TransportNode(Node):
    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def wall_timer(self, interval, callback):
        # /clock가 정지해도 감시 콜백이 돌아야 한다.
        return self.create_timer(interval, callback, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def send(self, publisher, payload):
        publisher.publish(UInt8MultiArray(data=list(encode(payload))))

    def wall_limit(self):
        self.declare_parameter('wall_timeout_s', 3.)
        value = float(self.get_parameter('wall_timeout_s').value)
        if not math.isfinite(value) or not .1 <= value <= 60.:
            raise ValueError('wall_timeout_s outside .1..60')
        return value


class ActuationHost(TransportNode):
    """/drive/safe만 입력으로 사용. ARM/CLEAR는 서비스 ACK를 기다린다."""
    def __init__(self):
        super().__init__('actuation_host')
        self.wall_timeout = self.wall_limit()
        self.producer, self.decoder = CommandProducer(), Decoder()
        self.offer = None
        self.offer_wall = 0.
        self.pending = {}
        self.arm_barrier = None
        self.last_now = None
        self.tx = self.create_publisher(UInt8MultiArray, '/actuation/tx', 10)
        self.create_subscription(UInt8MultiArray, '/actuation/rx', self.on_rx, 10)
        self.create_subscription(AckermannDriveStamped, '/drive/safe', self.on_drive, 1)
        group = ReentrantCallbackGroup()
        for service, kind in [('arm', 'ARM'), ('clear_stop', 'CLEAR'), ('disarm', 'DISARM'), ('stop', 'STOP')]:
            async def request(req, response, operation=kind):
                return await self.control(operation, response)
            self.create_service(Trigger, '/actuation/' + service, request, callback_group=group)
        self.wall_timer(.01, self.watch)

    def on_rx(self, message):
        for payload in self.decoder.feed(bytes(message.data)):
            if payload.get('type') == 'offer':
                if (all(integer(payload.get(key)) for key in ('generation', 'lease', 'lease_issued_us')) and
                        type(payload.get('boot')) is str and type(payload.get('state')) is str and 'owner' in payload):
                    self.offer, self.offer_wall = payload, time.monotonic()
            elif payload.get('type') == 'ack':
                request_id = payload.get('request')
                if type(request_id) is not str:
                    continue
                pending = self.pending.pop(request_id, None)
                if pending:
                    future, _, kind = pending
                    if payload.get('accepted') and kind == 'ARM':
                        self.arm_barrier = self.now_us()
                    future.set_result((payload.get('accepted') is True, str(payload.get('reason'))))

    def fresh_offer(self):
        return (self.offer is not None and time.monotonic() - self.offer_wall < self.wall_timeout and
                0 <= self.now_us() - self.offer.get('lease_issued_us', -1) < 100_000)

    async def control(self, kind, response):
        if kind != 'STOP' and not self.fresh_offer():
            response.success, response.message = False, 'fresh MCU offer required'
            return response
        if len(self.pending) >= 8:
            if kind == 'STOP':
                self.send(self.tx, {'type': 'command', 'request': uuid.uuid4().hex,
                                    'command': {'v': 1, 'kind': 'STOP'}})
            response.success, response.message = False, 'request queue full; STOP sent without confirmation' if kind == 'STOP' else 'request queue full'
            return response
        command = {'v': 1, 'kind': 'STOP'} if kind == 'STOP' else self.producer.control(self.offer, kind)
        request_id, future = uuid.uuid4().hex, Future()
        self.pending[request_id] = (future, time.monotonic() + self.wall_timeout, kind)
        self.send(self.tx, {'type': 'command', 'request': request_id, 'command': command})
        response.success, response.message = await future
        return response

    def on_drive(self, message):
        now, source = self.now_us(), stamp_us(message.header.stamp)
        if (not self.fresh_offer() or not self.offer.get('output_ready') or
                self.arm_barrier is None or source < self.arm_barrier):
            return
        try:
            frame = self.producer.drive(self.offer, speed_mps=message.drive.speed,
                                       steering_rad=message.drive.steering_angle,
                                       source_us=source, host_now_us=now)
            self.send(self.tx, {'type': 'command', 'request': '', 'command': frame})
        except ValueError:
            # 재전송/오래된 값을 새 명령으로 포장하지 않는다. MCU의 독립 만료로 정지한다.
            return

    def watch(self):
        now = self.now_us()
        if self.last_now is not None and now < self.last_now:
            self.send(self.tx, {'type': 'command', 'request': '', 'command': {'v': 1, 'kind': 'STOP'}})
            self.offer, self.arm_barrier = None, None
            # 새 시간축에 옛 source watermark/호스트 소유권을 이어 붙이지 않는다.
            # 새 producer여도 MCU 재시작과 명시 ARM 전에는 보낼 수 없다.
            self.producer = CommandProducer()
            for future, _, _ in self.pending.values():
                future.set_result((False, 'clock regressed; previous request outcome unconfirmed'))
            self.pending.clear()
        self.last_now = now
        for request_id, (future, deadline, _) in list(self.pending.items()):
            if time.monotonic() >= deadline:
                self.pending.pop(request_id)
                future.set_result((False, 'MCU acknowledgement timeout; outcome unconfirmed'))


class VirtualMCU(TransportNode):
    """명령 수신·계수 검증만 수행. Gazebo 모터 PI는 이번 단계에서 그대로 둔다."""
    def __init__(self):
        super().__init__('virtual_mcu')
        self.wall_timeout = self.wall_limit()
        self.declare_parameter('wheel_radius_m', .025)
        self.declare_parameter('ticks_per_revolution', 2048)
        self.declare_parameter('max_speed_mps', 2.5)
        self.declare_parameter('max_steering_angle_rad', .37)
        config = ContractConfig(wheel_radius_m=float(self.get_parameter('wheel_radius_m').value),
                                ticks_per_rev=int(self.get_parameter('ticks_per_revolution').value),
                                max_speed_mps=float(self.get_parameter('max_speed_mps').value),
                                max_steering_rad=float(self.get_parameter('max_steering_angle_rad').value))
        self.receiver, self.decoder = ActuationReceiver(config), Decoder()
        self.tick_angle = math.tau / config.ticks_per_rev
        self.wheels = deque(maxlen=32)
        self.feedback_seq = 0
        self.last_now = self.last_tick = self.last_offer = None
        self.progress_wall = time.monotonic()
        self.clock_started = False
        self.last_fault_publish_wall = -math.inf
        self.clock_failed = False
        self.output_ack = None
        self.rx = self.create_publisher(UInt8MultiArray, '/actuation/rx', 10)
        self.status_pub = self.create_publisher(String, '/actuation/status', 10)
        self.create_subscription(UInt8MultiArray, '/actuation/tx', self.on_tx, 10)
        self.create_subscription(JointState, '/wheel_states', self.on_wheels, 10)
        self.create_subscription(String, '/actuation/output_status', self.on_output, 10)
        self.wall_timer(.001, self.control_tick)

    def on_output(self, message):
        try:
            data = json.loads(message.data)
            if type(data) is dict:
                self.output_ack = data
        except ValueError:
            pass

    def output_ready(self):
        ack = self.output_ack
        return bool(ack and ack.get('epoch') == [self.receiver.boot_id, self.receiver.generation] and
                    integer(ack.get('now_us')) and 0 <= self.now_us() - ack['now_us'] < 100_000 and
                    (not self.receiver.armed or ack.get('blocked') is False))

    def send_offer(self):
        self.send(self.rx, {'type': 'offer', **self.receiver.offer(self.now_us()), 'output_ready': self.output_ready()})

    def publish_status(self):
        status = {**self.receiver.status(), 'now_us': self.now_us(), 'output_ready': self.output_ready(),
                  'stationary_ready': self.receiver._stationary(self.now_us())}
        self.status_pub.publish(String(data=json.dumps(status, allow_nan=False)))
        return status

    def on_tx(self, message):
        if self.receiver.now_us is not None and self.now_us() < self.receiver.now_us:
            self.clock_failed = True
            self.receiver._fault('clock_regressed_restart_required')
        for payload in self.decoder.feed(bytes(message.data)):
            if payload.get('type') != 'command' or type(payload.get('command')) is not dict:
                continue
            if self.clock_failed:
                accepted, reason = False, 'clock_restart_required'
            elif payload['command'].get('kind') == 'ARM' and not self.output_ready():
                accepted, reason = False, 'output_not_ready'
            else:
                accepted, reason = self.receiver.receive(payload['command'], self.now_us())
            self.publish_status()
            request_id = payload.get('request')
            if type(request_id) is str and 0 < len(request_id) <= 64:
                self.send(self.rx, {'type': 'ack', 'request': request_id, 'accepted': accepted, 'reason': reason})
            if not self.clock_failed:
                self.send_offer()

    def on_wheels(self, message):
        try:
            indices = [message.name.index(name) for name in ('rear_left_wheel_joint', 'rear_right_wheel_joint')]
            positions = [message.position[i] for i in indices]
            if not all(math.isfinite(p) for p in positions):
                return
            self.wheels.append((stamp_us(message.header.stamp), *(round(p / self.tick_angle) for p in positions)))
        except (ValueError, IndexError):
            return

    def control_tick(self):
        now, wall = self.now_us(), time.monotonic()
        if self.last_now is not None and now < self.last_now:
            self.clock_failed = True
            self.receiver._fault('clock_regressed_restart_required')
        if now != self.last_now:
            self.last_now, self.progress_wall = now, wall
        self.clock_started |= now > 0
        if self.clock_started and wall - self.progress_wall >= self.wall_timeout:
            self.clock_failed = True
            self.receiver._fault('clock_stalled_restart_required')
        if self.clock_failed:
            if wall - self.last_fault_publish_wall >= .01:
                self.publish_status()
                self.last_fault_publish_wall = wall
            return
        if self.last_tick is not None and now - self.last_tick < 5_000:
            return
        self.last_tick = now
        # /clock와 별도 토픽의 배송 순서를 흡수한다. 미래 취득값의 시각을 바꾸지 않는다.
        while self.wheels and self.wheels[0][0] <= now:
            stamp, left, right = self.wheels.popleft()
            self.feedback_seq += 1
            self.receiver.update_encoder(now, sequence=self.feedback_seq, capture_us=stamp,
                                         left_ticks=left, right_ticks=right)
        self.receiver.tick(now)
        self.publish_status()
        if self.last_offer is None or now - self.last_offer >= 20_000:
            self.last_offer = now
            self.send_offer()


class GuardedSimActuator(TransportNode):
    def __init__(self):
        super().__init__('guarded_sim_actuator')
        self.declare_parameter('wheelbase_m', .145)
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        if not math.isfinite(self.wheelbase) or self.wheelbase <= 0:
            raise ValueError('invalid wheelbase')
        self.guard = AppliedGuard(wall_timeout_s=self.wall_limit())
        self.pending = deque(maxlen=16)
        self.twist = self.create_publisher(Twist, '/sim/cmd_vel', 1)
        self.steer = self.create_publisher(Float64, '/sim/steering_angle', 1)
        self.audit = self.create_publisher(String, '/actuation/output_status', 10)
        self.create_subscription(String, '/actuation/status', self.on_status, 10)
        self.wall_timer(.01, self.output)

    def on_status(self, message):
        try:
            status = json.loads(message.data)
            # 수신 측 /clock가 잠시 늦으면 원래 시각을 유지한 채 기다린다.
            if type(status) is dict and type(status.get('now_us')) is int:
                self.pending.append(status)
        except ValueError:
            self.guard.stop('invalid_status_json')

    def output(self):
        now, wall = self.now_us(), time.monotonic()
        while self.pending and self.pending[0]['now_us'] <= now:
            self.guard.update(self.pending.popleft(), now, wall)
        self.guard.watch(now, wall)
        command = Twist()
        command.linear.x = self.guard.speed
        command.angular.z = self.guard.speed * math.tan(self.guard.steering) / self.wheelbase
        self.twist.publish(command)
        self.steer.publish(Float64(data=self.guard.steering))
        self.audit.publish(String(data=json.dumps({'now_us': now, 'speed_mps': self.guard.speed,
                                                  'blocked': self.guard.blocked, 'reason': self.guard.reason,
                                                  'epoch': self.guard.epoch, 'last_status_us': self.guard.last_stamp})))


def host_main(args=None):
    run_node(ActuationHost, args=args)


def mcu_main(args=None):
    run_node(VirtualMCU, args=args)


def actuator_main(args=None):
    run_node(GuardedSimActuator, args=args)
