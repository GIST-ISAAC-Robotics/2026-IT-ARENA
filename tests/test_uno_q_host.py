"""UNO Q/USB를 열지 않는 host 통합. 시간·전압·기구 실측의 대체가 아니다."""
from collections import deque
from concurrent.futures import Future
from dataclasses import replace
import json
from pathlib import Path
import queue
import struct
import subprocess
import sys
import zlib

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_vehicle_interface')]

from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.serial_ingress import SerialReceipt
from arena_vehicle_interface.steering_calibration import read_profile
from receiver_owned_actuation_host import ReceiverLeaseTiming
from runtime_actuation_link import RuntimeActuationLink
from run_runtime_actuation_host import ClockPolicy
from same_host_command_clock import SameHostCommandClock, SameHostClockBounds
from uno_q_external_servo import encode_status, decode_status, UnoMailbox, UnoIngress
from uno_q_servo import UsbServo, ServoProfile, TRANSACTION_NS, ServoMailbox
from uno_q_host import UnoHostOwner, parser, validate


class Clock:
    now = 1_000_000_000
    def mono(self):return self.now
    def ros(self):return self.now+1_800_000_000_000_000_000
    def step(self, ns=5_000_000):self.now += ns


def profile():
    # goal speed/geometry/command range는 test 명시값이며 운영 교정값을 만들지 않는다.
    return ServoProfile(read_profile(ROOT/'config/hardware/steering_zero.assembled_2026_11_14.json'),
                        128, 10, .2, .128, .1239, .5235987755982988, 20_000, 100_000)


class FakeUsb:
    def __init__(self, clock):
        self.clock = clock;self.rows = deque();self.sent = [];self.closed = False
        self.position = 2824;self.torque = 0;self.drop = False;self.echo = False
        self.latency_ns = 0
        self.events = [];self.last_target = None
        self.cfg = bytearray(40)
        self.cfg[5] = 1;self.cfg[8] = 1;self.cfg[30] = 1
        self.cfg[9:13] = struct.pack('<HH', 0, 4095)
    def receipt(self, kind, data):
        return SerialReceipt(kind, data, self.clock.mono(), self.clock.ros())
    def send(self, wire):
        self.sent.append(wire);self.rows.append(self.receipt('tx', wire))
        self.events.append((self.clock.now, wire))
        instruction, params = wire[4], wire[5:-1]
        if instruction == 2:
            address, size = params
            if address == 0:data = bytes(self.cfg)
            elif address == 40:data = bytes((self.torque,))
            else:
                data = struct.pack('<HHHBBBBBHH', self.position, 0, 0, 120, 25, 0, 0, 0, 0, 0)
            assert len(data) == size
        elif instruction == 3:
            address = params[0]
            assert address in (40, 41), 'No EEPROM writes'
            if address == 40:self.torque = params[1]
            else:self.position = self.last_target = int.from_bytes(params[2:4], 'little')
            data = b''
        else:raise AssertionError('unexpected command')
        if not self.drop:
            body = bytes((1, len(data)+2, 0))+data
            reply = b'\xff\xff'+body+bytes((~sum(body)&255,))
            data = (wire if self.echo else b'')+reply
            self.rows.append(SerialReceipt('rx', data, self.clock.mono()+self.latency_ns,
                                           self.clock.ros()+self.latency_ns))
    def get(self, timeout=0.):
        return self.rows.popleft() if self.rows and self.rows[0].monotonic_ns <= self.clock.now else None
    def close(self):self.closed = True


class FakeMcu:
    """호스트 계약 fixture. 실제 portable MCU 판정은 별도 C++ 시험이 담당한다."""
    def __init__(self, clock):
        self.clock = clock;self.rows = deque();self.sent = [];self.closed = False
        self.boot = '1234abcd';self.generation = 0;self.state = 'DISARMED';self.owner = None
        self.lease = 0;self.ready = False;self.seq = -1;self.last_esv_seq = 0
        self.applied = -1;self.deadline = 0;self.pwm_permitted = False
        self.events = [];self.accept_observation = True
        self.observation_latency_ns = 0
        self.offer()
        self.observation()
    def receipt(self, kind, data):return SerialReceipt(kind, data, self.clock.mono(), self.clock.ros())
    def row(self, row):
        delay = self.observation_latency_ns if row.get('type') == 'uno_q' else 0
        self.rows.append(SerialReceipt('rx', encode(row), self.clock.mono()+delay, self.clock.ros()+delay))
    def offer(self):
        self.lease += 1;us = self.clock.now//1000
        self.row(dict(type='offer', v=1, boot=self.boot, generation=self.generation, state=self.state,
            owner=self.owner, lease=self.lease, lease_issued_us=us, lease_expires_us=us+100_000,
            output_ready=self.ready, status_now_us=us, estop_held=False, reason='fixture',
            motor_feedback_v1=dict(valid=False, sequence=None, capture_us=None, counts=None)))
    def observation(self):
        self.row(dict(type='uno_q', v=1, boot=self.boot, generation=self.generation, accepted_seq=self.seq,
                      state=self.state, reason='fixture', bench_state='disarmed', bench_seq=-1, bench_reason='disarmed',
                      profile_sha256='a'*64, mode='installed'))
    def send(self, wire):
        self.sent.append(wire);self.rows.append(self.receipt('tx', wire))
        if wire[4:8] == b'ESV1':
            r = decode_status(wire)
            if (r['boot'], r['generation'], r['lease']) != (self.boot, self.generation, self.lease):return
            if r['status_seq'] <= self.last_esv_seq:return
            self.last_esv_seq = r['status_seq'];self.ready = r['ready'];self.applied = r['applied_drive_seq']
            self.deadline = self.clock.now+(100_000-r['observation_age_us'])*1000
            self.pwm_permitted = self.ready and self.state == 'ACTIVE' and self.applied == self.seq
            self.events.append((self.clock.now, 'esv', self.seq, self.applied, self.pwm_permitted))
            self.offer();return
        row = Decoder().feed(wire)[0];c = row['command'];kind = c['kind']
        accepted = True
        reasons = dict(ARM='explicit_arm', CLEAR='explicit_clear', DISARM='explicit_disarm', STOP='software_stop')
        if kind == 'ARM':
            accepted = self.ready and self.state == 'DISARMED'
            if accepted:self.generation += 1;self.state = 'ARMED';self.owner = c['client'];self.seq = c['seq']
        elif kind == 'DRIVE':
            assert self.ready and self.state in ('ARMED', 'ACTIVE')
            self.seq = c['seq'];self.state = 'ACTIVE';self.pwm_permitted = self.applied == self.seq
            self.events.append((self.clock.now, 'drive', self.seq, self.applied, self.pwm_permitted))
            if self.accept_observation:self.observation()
            return
        else:
            self.generation += 1;self.state = 'ESTOP_LATCHED' if kind == 'STOP' else 'DISARMED'
            self.owner = None;self.ready = self.pwm_permitted = False
        self.row(dict(type='ack', request=row['request'], accepted=accepted, reason=reasons[kind]))
        self.offer();self.observation()
    def get(self, timeout=0.):
        return self.rows.popleft() if self.rows and self.rows[0].monotonic_ns <= self.clock.now else None
    def close(self):self.closed = True
    def expire(self):
        if self.clock.now >= self.deadline:
            self.ready = self.pwm_permitted = False
            if self.state in ('ARMED', 'ACTIVE'):
                self.state = 'FAULT_LATCHED';self.owner = None;self.generation += 1


def make_owner(enabled=True):
    t = Clock();u = FakeUsb(t);m = FakeMcu(t);events = []
    policy = ClockPolicy(ReceiverLeaseTiming(100_000_000, 80_000_000, 1000, 'fake only'),
        SameHostClockBounds(100_000_000, 100_000_000, 50_000_000, 1_000_000, 0, 0, 1000, True, 'fake only'),
        100_000, 500_000_000)
    engine = RuntimeActuationLink(host_epoch='host', client_id='client', timing=policy.receiver_lease,
        max_speed_mps=1, max_steering_rad=.2, max_lease_us=100_000,
        request_timeout_ns=policy.request_timeout_ns, drive_enabled=enabled)
    servo = UsbServo(u, profile=profile() if enabled else None, enabled=enabled, event=events.append)
    owner = UnoHostOwner(servo=servo, expected_profile_hash='a'*64, engine=engine, ingress=m, policy=policy,
        clock=SameHostCommandClock(host_epoch='host', ros_epoch='ros', bounds=policy.same_host),
        monotonic_ns=t.mono, ros_now_ns=t.ros, read_mode=lambda: ('SYSTEM_TIME', False), event=events.append)
    return owner, t, m, u, events


def ticks(owner, t, count=1):
    for _ in range(count):owner.tick();t.step()


def arm(owner, t):
    ticks(owner, t, 7)
    future = Future();owner.request_control('ARM', future=future)
    for _ in range(20):
        ticks(owner, t)
        if future.done():break
    assert future.done() and future.result()['success'], (future.result() if future.done() else None)
    assert owner.servo.engaged and owner.authorization == ('1234abcd', 1)
    return future


def send_drive(owner, t, angle=.1):
    source = t.ros();t.step(1_000_000)
    owner.drive(source_ros_ns=source, speed_mps=.1, steering_rad=angle)
    t.step(1_000_000)
    return source


def test_dry_run_never_imports_serial_or_ros():
    result = subprocess.run([sys.executable, str(ROOT/'scripts/uno_q_host.py'), '--dry-run'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    data = json.loads(result.stdout)
    assert data['hardware_opened'] is False and data['drive_wire_enabled'] is False
    assert data['baudrate'] == 1_000_000 and data['external_status_frame_bytes'] == 57


def test_configuration_does_not_promote_missing_profile_to_drive():
    with pytest.raises(ValueError, match='installed profile'):
        validate(parser().parse_args(['--dry-run', '--enable-drive-wire', '--ros']))


def test_esv_golden_crc_exact_fields_and_invalid_rejection():
    r = dict(boot='1234abcd', generation=0, lease=1, status_seq=1, observation_age_us=0,
             ready=False, applied_drive_seq=-1)
    wire = encode_status(**r)
    assert wire.hex() == '494131004553563131323334616263640000000000000000010000000000000001000000000000000000000000ffffffffffffffffa6bb51ff'
    assert decode_status(wire) == r
    with pytest.raises(ValueError):decode_status(wire[:-1]+bytes((wire[-1]^1,)))
    for update in ({'ready': True, 'observation_age_us': 100000}, {'lease': 0},
                   {'status_seq': True}, {'boot': 'oldboot'}, {'applied_drive_seq': -2}):
        with pytest.raises(ValueError):encode_status(**(r|update))


def test_mailbox_stop_purges_external_status_and_drive():
    q = UnoMailbox()
    status = encode_status(boot='1234abcd', generation=1, lease=1, status_seq=1,
                           observation_age_us=0, ready=True, applied_drive_seq=3)
    drive = encode(dict(type='command', request='', command=dict(kind='DRIVE')))
    stop = encode(dict(type='command', request='stop', command=dict(kind='STOP')))
    q.put_nowait(status);q.put_nowait(drive);q.put_nowait(stop)
    assert q.get_nowait() == stop
    with pytest.raises(queue.Empty):q.get_nowait()


def test_stop_cancels_usb_request_only_before_io_thread_claims_it():
    class DeferredUsb:
        def __init__(self):self.commands = ServoMailbox()
        def send(self, wire):self.commands.put_nowait(wire)
        def get(self, timeout=0.):return None
    io = DeferredUsb();servo = UsbServo(io, profile=profile(), enabled=True)
    servo._goal('goal', 1_000_000, 2820,
                dict(target=2820, seq=3, authorization=('1234abcd', 1), deadline_ns=30_000_000))
    servo.release()
    assert servo.transaction is None
    with pytest.raises(queue.Empty):io.commands.get_nowait()
    servo.tick(2_000_000)
    assert io.commands.get_nowait()[5:-1] == b'\x28\x00'
    # 가져간 frame은 이미 write 중일 수 있어 취소 성공을 꾸미지 않는다.
    servo.release()
    assert servo.transaction is not None


def test_default_diagnostics_read_only_even_if_servo_torque_already_on():
    owner, t, m, u, events = make_owner(False);u.torque = 1
    ticks(owner, t, 20)
    assert not owner.terminating and owner.servo.torque == 1
    assert all(w[4] == 2 for w in u.sent)
    assert not any(decode_status(w)['ready'] for w in m.sent if w[4:8] == b'ESV1')
    result = owner.request_control('ARM')
    assert result['reason'] == 'drive_disabled_by_host'


def test_arm_reads_current_holds_ack_then_torque_on_without_centering():
    owner, t, m, u, events = make_owner();u.echo = True
    arm(owner, t)
    writes = [w for w in u.sent if w[4] == 3]
    assert len(writes) == 2 and writes[0][5] == 41 and writes[1][5:-1] == b'\x28\x01'
    assert int.from_bytes(writes[0][7:9], 'little') == 2824 != 2811
    order = [r['operation'] for r in events if r['event'] == 'servo_ack']
    assert order.index('arm_read') < order.index('hold') < order.index('torque_on')


def test_mcu_acceptance_then_goal_ack_then_applied_sequence_and_identical_goal_skip():
    owner, t, m, u, events = make_owner();arm(owner, t)
    before = len([w for w in u.sent if w[4] == 3 and w[5] == 41])
    source = send_drive(owner, t)
    assert owner.candidate_drive is not None and not m.pwm_permitted
    assert len([w for w in u.sent if w[4] == 3 and w[5] == 41]) == before
    ticks(owner, t, 5)
    assert not owner.terminating and m.pwm_permitted and m.applied == m.seq
    assert u.last_target == profile().goal(.1)
    first = len([w for w in u.sent if w[4] == 3 and w[5] == 41])
    send_drive(owner, t);ticks(owner, t, 5)
    assert m.pwm_permitted and len([w for w in u.sent if w[4] == 3 and w[5] == 41]) == first
    prepared = [r for r in events if r['event'] == 'drive_prepared']
    assert prepared[0]['source']['source_ros_ns'] == source
    drive_at = [r[0] for r in m.events if r[1] == 'drive']
    permit_at = [r[0] for r in m.events if r[1] == 'esv' and r[4]]
    delays = [next(x for x in permit_at if x >= begin)-begin for begin in drive_at]
    assert max(delays) <= 25_000_000


def test_no_goal_when_mcu_does_not_accept_and_no_automatic_rearm():
    owner, t, m, u, events = make_owner();arm(owner, t);m.accept_observation = False
    before = len([w for w in u.sent if w[4] == 3 and w[5] == 41])
    send_drive(owner, t);ticks(owner, t, 14)
    assert owner.terminating and 'acceptance' in owner.exit_reason
    assert len([w for w in u.sent if w[4] == 3 and w[5] == 41]) == before
    assert owner.authorization is None and not m.pwm_permitted


def test_usb_disconnect_zeroes_readiness_and_stale_ack_cannot_rearm():
    owner, t, m, u, events = make_owner();arm(owner, t);u.drop = True
    send_drive(owner, t);ticks(owner, t, 10)
    assert owner.terminating and owner.servo.fault and not owner.servo.engaged
    count = len(u.sent);u.drop = False
    ticks(owner, t, 10)
    assert len(u.sent) == count and not owner.servo.torque_off_confirmed
    m.expire();assert not m.pwm_permitted


def test_stop_during_hold_never_sends_torque_on_or_next_goal():
    owner, t, m, u, events = make_owner();ticks(owner, t, 7)
    future = Future();owner.request_control('ARM', future=future)
    for _ in range(10):
        ticks(owner, t)
        if owner.servo.transaction and owner.servo.transaction['operation'] == 'hold':break
    owner.request_control('STOP');ticks(owner, t, 6)
    assert future.done() and not future.result()['success']
    assert not any(w[4] == 3 and w[5:7] == b'\x28\x01' for w in u.sent)
    assert owner.servo.torque_off_confirmed and not m.pwm_permitted


def test_mcu_reboot_closes_session_and_servo_authorization():
    owner, t, m, u, events = make_owner();arm(owner, t)
    m.boot = 'deadbeef';m.offer();ticks(owner, t)
    assert owner.terminating and owner.authorization is None


def test_stale_source_does_not_create_servo_goal_or_drive():
    owner, t, m, u, events = make_owner();arm(owner, t)
    before = len(m.sent)
    owner.drive(source_ros_ns=t.ros()-200_000_000, speed_mps=.1, steering_rad=.1)
    assert owner.candidate_drive is None and len(m.sent) == before


def test_serial_constructor_override_keeps_one_megabaud_without_shared_default_change():
    captured = []
    def factory(**kwargs):captured.append(kwargs);return object()
    ingress = UnoIngress('unopened', factory=factory)
    ingress.factory(port=None, baudrate=921600)
    assert captured[0]['baudrate'] == 1_000_000 and captured[0]['write_timeout'] == .02


@pytest.mark.parametrize('usb_ms,report_ms', [(0, 0), (5, 0), (10, 10), (20, 20)])
def test_latency_matrix_records_motor_inhibit_not_claiming_hardware_timing(usb_ms, report_ms):
    owner, t, m, u, events = make_owner();arm(owner, t)
    u.latency_ns = usb_ms*1_000_000;m.observation_latency_ns = report_ms*1_000_000
    results = []
    for angle in (.1, .1, -.1):
        ticks(owner, t, 6)
        sent_at = t.now+1_000_000;send_drive(owner, t, angle)
        for _ in range(15):
            ticks(owner, t)
            if owner.terminating or m.pwm_permitted:break
        results.append(dict(usb_reply_delay_ms=usb_ms, mcu_report_delay_ms=report_ms, angle=angle,
            permitted=m.pwm_permitted, terminated=owner.terminating,
            inhibit_ms=None if not m.pwm_permitted else (t.now-5_000_000-sent_at)/1e6))
        if owner.terminating:break
    print('UNO_FAKE_LATENCY '+json.dumps(results))
    if owner.terminating:
        assert not owner.servo.engaged and owner.authorization is None
    else:
        assert all(row['permitted'] and row['inhibit_ms'] <= 60 for row in results)
