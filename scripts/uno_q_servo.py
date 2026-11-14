"""USB ST3215 단일 요청 순서기. EEPROM 쓰기/자동 재시도/자동 ARM 없음."""
from dataclasses import dataclass
import math
import queue
import struct
import threading

from arena_vehicle_interface.steering_calibration import cad_servo_goal, check_profile
from probe_bus_servo import packet, feedback
from mounted_servo_setup import bounds

TRANSACTION_NS = 30_000_000


class ServoMailbox:
    """USB 작업이 아직 가져가지 않은 단일 요청만 STOP 때 원자적으로 취소한다."""
    def __init__(self):
        self.lock = threading.Lock()
        self.pending = None
    def put_nowait(self, wire):
        if type(wire) is not bytes or not 1 <= len(wire) <= 1544:
            raise ValueError('bounded servo wire required')
        with self.lock:
            if self.pending is not None:raise queue.Full
            self.pending = wire
    def get_nowait(self):
        with self.lock:
            if self.pending is None:raise queue.Empty
            wire, self.pending = self.pending, None
            return wire
    def cancel(self, wire):
        with self.lock:
            if self.pending != wire:return False
            self.pending = None
            return True


class NoServo:
    """명시 생략한 USB 서보. 진단/모터 단독 BEN1에서 열거나 쓰는 장치가 없다."""
    enabled = engaged = torque_off_confirmed = False
    authorization = fault = observation = observed_ns = observed_realtime_ns = None
    applied_seq = -1
    release_pending = False
    def __init__(self):self.io = self
    def ready(self, now):return False
    def armed_ready(self, now):return False
    def tick(self, now):pass
    def release(self):pass
    def close(self):pass


@dataclass(frozen=True)
class ServoProfile:
    zero: dict
    goal_speed: int
    goal_acceleration: int
    command_limit: float
    wheelbase: float
    track: float
    wheel_limit: float
    position_period_us: int
    observation_max_age_us: int
    id: int = 1

    def __post_init__(self):
        check_profile(self.zero, ready=True)
        if (self.id != 1 or type(self.goal_speed) is not int or not 1 <= self.goal_speed <= 400
                or type(self.goal_acceleration) is not int or not 5 <= self.goal_acceleration <= 20
                or type(self.position_period_us) is not int or self.position_period_us <= 0
                or type(self.observation_max_age_us) is not int
                or not self.position_period_us < self.observation_max_age_us <= 100_000):
            raise ValueError('explicit bounded servo configuration required')
        for angle in (-self.command_limit, 0., self.command_limit):
            self.goal(angle)

    def goal(self, angle):
        return cad_servo_goal(self.zero, angle, command_limit=self.command_limit,
                              wheelbase=self.wheelbase, track=self.track, wheel_limit=self.wheel_limit)['position']


class UsbServo:
    """메인 owner에서 tick; blocking USB read/write는 SerialIngress thread가 소유한다.

    요청 deadline은 enqueue부터 30 ms이며 ACK는 matching TX receipt 뒤에만 받는다.
    timeout은 영구 fault: 요청 ID가 없는 ST ACK를 다음 요청에 재사용하지 않는다.
    """
    def __init__(self, ingress, *, profile=None, enabled=False, event=lambda _: None):
        if type(enabled) is not bool or (enabled and profile is None):
            raise ValueError('enabled output requires explicit installed servo profile')
        self.io, self.profile, self.enabled, self.event = ingress, profile, enabled, event
        self.transaction = None
        self.buffer = bytearray()
        self.fault = None
        self.config = self.observation = None
        self.observed_ns = None
        self.observed_realtime_ns = None
        self.torque = None
        self.torque_off_confirmed = False
        self.stage = 'probe_config'
        self.authorization = None
        self.engaged = False
        self.applied_seq = -1
        self.goal_pending = None
        self.last_goal = None
        self.release_pending = False
        self.last_now = None

    def fail(self, reason):
        if self.fault is None:
            self.fault = reason
            self.event(dict(event='servo_fault', reason=reason, torque_off_confirmed=False))
        self.authorization = None
        self.engaged = False
        self.goal_pending = None
        self.applied_seq = -1
        self.torque_off_confirmed = False

    def ready(self, now):
        limit = 100_000_000 if self.profile is None else self.profile.observation_max_age_us*1000
        return (self.fault is None and self.config is not None and self.observation is not None
                and self.observed_ns <= now < self.observed_ns+limit)

    def armed_ready(self, now):
        return self.engaged and self.authorization is not None and self.ready(now)

    def arm(self, authorization, now):
        if (not self.enabled or not self.ready(now) or self.authorization is not None
                or self.release_pending):
            raise RuntimeError('servo not ready for explicit ARM')
        self.authorization = authorization
        self.applied_seq = -1
        self.stage = 'arm_read'
        self.engaged = False

    def rebind(self, old, new):
        if self.authorization != old or not self.engaged:
            raise RuntimeError('ARM generation transition without held servo')
        self.authorization = new

    def release(self):
        # in-flight 요청의 ACK를 소비하되 완료 callback은 authorization 검사로 무효화한다.
        self.authorization = None
        self.engaged = False
        self.goal_pending = None
        self.applied_seq = -1
        t = self.transaction
        mailbox = getattr(self.io, 'commands', None)
        if t is not None and t['written_ns'] is None and isinstance(mailbox, ServoMailbox) and mailbox.cancel(t['wire']):
            self.event(dict(event='servo_unsent_cancelled', operation=t['operation']))
            self.transaction = None
            self.buffer.clear()
        self.stage = 'standby'
        self.release_pending = True

    def request_goal(self, *, seq, steering_rad, authorization, now, deadline_ns):
        if (not self.armed_ready(now) or authorization != self.authorization
                or type(seq) is not int or seq < 0 or deadline_ns <= now):
            raise RuntimeError('current ARM/goal deadline required')
        target = self.profile.goal(steering_rad)
        self.goal_pending = dict(seq=seq, target=target, authorization=authorization,
                                 deadline_ns=deadline_ns)

    def _begin(self, operation, now, instruction, parameters, expected, context=None):
        wire = packet(1, instruction, parameters)
        deadline = now+TRANSACTION_NS
        if operation == 'goal':
            deadline = min(deadline, context['deadline_ns'])
        self.transaction = dict(operation=operation, wire=wire, expected=expected, started_ns=now,
                                deadline_ns=deadline, written_ns=None, context=context)
        self.buffer.clear()
        self.io.send(wire)
        self.event(dict(event='servo_request', operation=operation, wire_hex=wire.hex(), deadline_ns=deadline))

    def _read(self, operation, now, address, count, context=None):
        self._begin(operation, now, 2, bytes((address, count)), count, context)

    def _goal(self, operation, now, target, context):
        p = self.profile
        params = bytes((41, p.goal_acceleration))+struct.pack('<HHH', target, 0, p.goal_speed)
        self._begin(operation, now, 3, params, 0, context)

    def _complete(self, data, receipt_ns, realtime_ns):
        t, self.transaction = self.transaction, None
        op, context = t['operation'], t['context']
        self.event(dict(event='servo_ack', operation=op, receipt_monotonic_ns=receipt_ns,
                        data_hex=data.hex(), applied_drive_seq=self.applied_seq))
        if op == 'config':
            bounds(data)  # observed ID/baud/return level/endian/scale/mode, no EEPROM changes
            if self.profile is not None:
                lo, hi = bounds(data)
                if not lo <= self.profile.zero['safe_min_count'] < self.profile.zero['safe_max_count'] <= hi:
                    raise RuntimeError('servo EEPROM limits do not contain selected mounted profile')
            self.config = data
            self.stage = 'probe_feedback'
        elif op in ('feedback', 'arm_read'):
            observation = feedback(data)
            if (observation['servo_status'] or not 0 <= observation['position_counts'] <= 4095
                    or not 9 <= observation['voltage_v'] <= 12.6 or observation['temperature_c'] >= 50
                    or abs(observation['current_signed']) >= 150):
                raise RuntimeError('servo feedback outside existing probe observation bounds')
            self.observation, self.observed_ns = observation, receipt_ns
            self.observed_realtime_ns = realtime_ns
            if self.profile is not None and not (self.profile.zero['safe_min_count'] <= observation['position_counts']
                                               <= self.profile.zero['safe_max_count']):
                raise RuntimeError('current position outside selected mounted range')
            if op == 'arm_read' and context == self.authorization and self.authorization is not None:
                self.last_goal = observation['position_counts']
                self.stage = 'arm_hold'
            elif self.stage == 'probe_feedback':
                self.stage = 'probe_torque'
            self.event(dict(event='servo_feedback', observation=observation, receipt_monotonic_ns=receipt_ns))
        elif op == 'torque_read':
            if data[0] not in (0, 1):
                raise RuntimeError('unknown torque state')
            self.torque = data[0]
            self.torque_off_confirmed = data[0] == 0
            self.stage = 'standby'
        elif op == 'hold' and context == self.authorization and self.authorization is not None:
            self.stage = 'arm_torque'
        elif op == 'torque_on' and context == self.authorization and self.authorization is not None:
            self.torque = 1
            self.torque_off_confirmed = False
            self.engaged = True
            self.stage = 'engaged'
        elif op == 'torque_off':
            self.torque = 0
            self.torque_off_confirmed = True
            self.release_pending = False
        elif op == 'goal' and context['authorization'] == self.authorization and self.authorization is not None:
            self.last_goal = context['target']
            self.applied_seq = context['seq']

    def _receipt(self, receipt, now):
        self.event(dict(event='servo_wire_receipt', kind=receipt.kind, wire_hex=receipt.data.hex(),
                        receipt_monotonic_ns=receipt.monotonic_ns, receipt_realtime_ns=receipt.realtime_ns))
        t = self.transaction
        if receipt.kind == 'tx':
            if t is None or receipt.data != t['wire'] or t['written_ns'] is not None:
                raise RuntimeError('unmatched USB transmit receipt')
            if not t['started_ns'] <= receipt.monotonic_ns < t['deadline_ns']:
                raise RuntimeError('USB transmit receipt outside transaction')
            t['written_ns'] = receipt.monotonic_ns
            return
        if receipt.kind != 'rx':
            raise RuntimeError('unknown USB receipt')
        if t is None or t['written_ns'] is None:
            raise RuntimeError('unsolicited USB bytes; drain/restart required')
        if not t['written_ns'] <= receipt.monotonic_ns <= now < t['deadline_ns']:
            raise RuntimeError('USB response late or timestamp order invalid')
        if len(receipt.data) > 1024 or len(self.buffer)+len(receipt.data) > 2048:
            raise RuntimeError('USB receive bound')
        self.buffer.extend(receipt.data)
        while self.buffer:
            if len(self.buffer) < 4:
                return
            if self.buffer[:2] != b'\xff\xff' or not 2 <= self.buffer[3] <= 64:
                raise RuntimeError('USB response framing')
            length = self.buffer[3]+4
            if len(self.buffer) < length:
                return
            frame = bytes(self.buffer[:length]);del self.buffer[:length]
            if sum(frame[2:]) & 255 != 255:
                raise RuntimeError('USB response checksum')
            if frame == t['wire']:
                continue  # local adapter echo is never an ACK
            if frame[2] != 1 or len(frame[5:-1]) != t['expected'] or frame[4] != 0:
                raise RuntimeError('USB servo ID/status/response length')
            if self.buffer:
                raise RuntimeError('ambiguous multiple USB responses')
            self._complete(frame[5:-1], receipt.monotonic_ns, receipt.realtime_ns)
            return

    def tick(self, now):
        if self.fault is not None:
            return
        try:
            if type(now) is not int or now < 0 or (self.last_now is not None and now < self.last_now):
                raise RuntimeError('USB owner clock regression')
            self.last_now = now
            if self.transaction is not None and now >= self.transaction['deadline_ns']:
                raise TimeoutError('USB transaction deadline; no retry')
            for _ in range(32):
                receipt = self.io.get(timeout=0.)
                if receipt is None:
                    break
                self._receipt(receipt, now)
            if self.transaction is not None:
                return
            if self.release_pending:
                self._begin('torque_off', now, 3, b'\x28\x00', 0)
            elif self.config is None:
                self._read('config', now, 0, 40)
            elif self.stage == 'probe_feedback':
                self._read('feedback', now, 56, 15)
            elif self.stage == 'probe_torque':
                self._read('torque_read', now, 40, 1)
            elif self.stage == 'arm_read':
                self._read('arm_read', now, 56, 15, self.authorization)
            elif self.stage == 'arm_hold':
                self._goal('hold', now, self.last_goal, self.authorization)
            elif self.stage == 'arm_torque':
                self._begin('torque_on', now, 3, b'\x28\x01', 0, self.authorization)
            elif self.goal_pending is not None:
                goal, self.goal_pending = self.goal_pending, None
                if now >= goal['deadline_ns']:
                    raise TimeoutError('queued servo goal expired')
                if self.armed_ready(now) and goal['authorization'] == self.authorization:
                    if goal['target'] == self.last_goal:
                        self.applied_seq = goal['seq']
                    else:
                        self._goal('goal', now, goal['target'], goal)
            else:
                period = 20_000_000 if self.profile is None else self.profile.position_period_us*1000
                if self.observed_ns is None or now-self.observed_ns >= period:
                    self._read('feedback', now, 56, 15)
        except Exception as exc:
            self.fail(type(exc).__name__+': '+str(exc))
