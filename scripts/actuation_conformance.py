#!/usr/bin/env python3
"""MCU 포팅용 결정론적 입출력 벡터. 펌웨어/전기 출력/실제 정지 검증이 아니다."""
import argparse
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/arena_vehicle_interface'))
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig
from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.drive_feedback import MODE_MOTOR

BOOT = 'conformance-mcu-boot-a'
CLIENT = 'conformance-host-a'
FLOAT_TOLERANCE = 1e-9  # double 기반 PC 대조용. MCU float32 오차 예산은 별도 결정한다.
FILES = [
    'scripts/actuation_conformance.py', 'tests/test_actuation_conformance.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py',
    'src/arena_description/config/vehicle.yaml',
]


def motor_config():
    """10/2 팀 선택치 명시. legacy 기본값을 묵시적으로 가져오지 않는다."""
    return ContractConfig(feedback_mode=MODE_MOTOR, ticks_per_rev=28,
                          gear_ratio=15., wheel_radius_m=.0325)


def apply_event(receiver, config, event):
    """파일 입력만으로 참조를 재생한다. 시각은 모두 해당 MCU의 정수 us다."""
    op, now, args = event['op'], event['now_us'], event['args']
    if op == 'reboot':
        receiver = ActuationReceiver(config, boot_id=args['boot_id'])
        result = None
    elif op == 'tick':
        result = receiver.tick(now)
    elif op == 'encoder':
        result = receiver.update_motor_encoder(now, **args)
    elif op == 'legacy_encoder':
        result = receiver.update_encoder(now, **args)
    elif op == 'physical_stop':
        result = receiver.set_estop(args['pressed'], now)
    elif op == 'offer':
        result = receiver.offer(now)
    elif op == 'command':
        # 정확한 바이트를 저장하되 상태기계와 프레이밍 시험을 구별한다.
        frame = bytes.fromhex(args['frame_hex'])
        decoded = Decoder().feed(frame)
        if len(decoded) != 1 or decoded[0] != args['message']:
            raise ValueError('command frame does not match one complete message')
        result = list(receiver.receive(decoded[0], now))
    else:
        raise ValueError(f'unknown operation: {op}')
    return receiver, dict(result=result, status=receiver.status())


class Builder:
    def __init__(self, name):
        self.name, self.config = name, motor_config()
        self.receiver = ActuationReceiver(self.config, boot_id=BOOT)
        self.producer = CommandProducer(client_id=CLIENT)
        self.events, self.checkpoints = [], []
        self.now, self.seq, self.count = 0, 0, 0

    def event(self, op, now=None, **args):
        if now is not None:
            self.now = now
        event = dict(op=op, now_us=self.now, args=args)
        self.receiver, response = apply_event(self.receiver, self.config, event)
        self.events.append(dict(input=deepcopy(event), expected=deepcopy(response)))
        return response['result']

    def feedback(self, count=None):
        self.seq += 1
        if count is not None:
            self.count = count
        return self.event('encoder', sequence=self.seq, capture_us=self.now, motor_counts=self.count)

    def advance(self, duration, *, feedback=True, increment=0, refresh=False):
        if duration <= 0 or duration % 10_000:
            raise ValueError('fixture duration must be positive 10 ms steps')
        for _ in range(duration // 10_000):
            self.now += 10_000
            if feedback:
                self.feedback(self.count + increment)
            self.event('tick')
            if refresh and self.receiver.armed:
                self.drive(1.)

    def prepare(self, arm=True):
        self.feedback()
        self.event('tick')
        self.advance(220_000)
        if arm:
            self.control('ARM')
            self.check(state='ARMED', reason='explicit_arm', target_speed_mps=0.)
        return self

    def send(self, message):
        return self.event('command', message=message, frame_hex=encode(message).hex())

    def control(self, kind):
        return self.send(self.producer.control(self.event('offer'), kind))

    def packet(self, speed, *, age=0):
        offer = self.event('offer')
        self.producer.sequence += 1
        return dict(v=1, kind='DRIVE', boot=offer['boot'], generation=offer['generation'],
                    lease=offer['lease'], client=self.producer.client, seq=self.producer.sequence,
                    speed_mps=speed, steering_rad=.1, source_age_us=age, ttl_us=100_000)

    def drive(self, speed):
        message = self.packet(speed)
        self.send(message)
        return message

    def check(self, **status):
        # 수동으로 적은 기대값과 Python 결과가 먼저 일치해야 벡터를 내보낸다.
        current = self.receiver.status()
        for key, value in status.items():
            if current[key] != value:
                raise AssertionError(f'{self.name}: {key} expected {value!r}, got {current[key]!r}')
        self.checkpoints.append(dict(event_index=len(self.events)-1, expected_status=status))

    def finish(self):
        if not self.checkpoints:
            raise ValueError('independent checkpoint required')
        return dict(name=self.name, config=asdict(self.config), initial_boot=BOOT,
                    events=self.events, checkpoints=self.checkpoints)


def make_cases():
    cases = []
    b = Builder('boot_requires_explicit_arm').prepare(False)
    b.check(state='DISARMED', reason='boot', target_speed_mps=0., left_wheel_mps=None,
            right_wheel_mps=None, motor_counts=0)
    cases.append(b.finish())

    b = Builder('forward_and_zero_is_not_stop').prepare()
    b.drive(1.)
    b.advance(20_000, increment=20)
    b.check(state='ACTIVE', reason='drive', target_speed_mps=1., motor_counts=40)
    b.drive(0.)
    b.check(state='ARMED', reason='zero_target', target_speed_mps=0., steering_rad=.1)
    cases.append(b.finish())

    b = Builder('duplicate_command_does_not_extend_deadline').prepare()
    packet = b.drive(1.)
    deadline = b.receiver.deadline_us
    b.advance(20_000, increment=20)
    if b.send(packet) != [False, 'sequence']:
        raise AssertionError('duplicate command not rejected')
    b.check(state='ACTIVE', deadline_us=deadline)
    cases.append(b.finish())

    b = Builder('old_source_rejected_before_lease_end').prepare()
    packet = b.packet(1., age=90_000)
    b.advance(20_000)
    if b.send(packet) != [False, 'source_expired']:
        raise AssertionError('expired source not rejected')
    b.check(state='ARMED', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('command_timeout_at_100ms').prepare()
    b.drive(1.)
    b.advance(100_000, increment=20)
    b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('feedback_timeout_at_50ms').prepare()
    b.drive(1.)
    b.advance(50_000, feedback=False, refresh=True)
    b.check(state='FAULT_LATCHED', reason='encoder_timeout', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('control_gap_even_if_feedback_fresh').prepare()
    b.drive(1.)
    b.now += 60_000
    # 두 실제 취득 표본을 30 ms 간격으로 제공하되 tick은 60 ms 누락한다.
    b.now -= 30_000
    b.feedback(20)
    b.now += 30_000
    b.feedback(40)
    b.event('tick')
    b.check(state='FAULT_LATCHED', reason='control_deadline_missed', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('physical_stop_release_stays_latched').prepare()
    b.drive(1.)
    b.event('physical_stop', pressed=True)
    b.event('physical_stop', pressed=False)
    b.advance(220_000)
    b.check(state='ESTOP_LATCHED', reason='physical_stop', target_speed_mps=0., estop_held=False)
    cases.append(b.finish())

    b = Builder('software_stop_rejects_queued_drive').prepare()
    packet = b.drive(1.)
    b.send(dict(v=1, kind='STOP'))
    if b.send(packet)[0]:
        raise AssertionError('queued drive accepted after STOP')
    b.check(state='ESTOP_LATCHED', reason='software_stop', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('clear_requires_new_arm').prepare()
    packet = b.drive(1.)
    b.send(dict(v=1, kind='STOP'))
    b.advance(220_000)
    b.control('CLEAR')
    if b.send(packet)[0]:
        raise AssertionError('old drive accepted after CLEAR')
    b.check(state='DISARMED', reason='explicit_clear', target_speed_mps=0.)
    b.control('ARM')
    b.drive(.5)
    b.check(state='ACTIVE', reason='drive', target_speed_mps=.5)
    cases.append(b.finish())

    b = Builder('legacy_feedback_rejected_in_motor_mode').prepare()
    b.drive(1.)
    b.event('legacy_encoder', sequence=30, capture_us=b.now, left_ticks=0, right_ticks=0)
    b.check(state='FAULT_LATCHED', reason='feedback_mode_mismatch', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('encoder_jump_fault').prepare()
    b.drive(1.)
    b.now += 10_000
    b.feedback(100_000)
    b.check(state='FAULT_LATCHED', reason='encoder_implausible', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('signed_motor_counts_not_unsigned_speed').prepare()
    b.drive(1.)
    b.now += 10_000
    b.feedback(-20)
    b.event('tick')
    b.check(state='ACTIVE', motor_counts=-20, left_wheel_mps=None, right_wheel_mps=None)
    if b.receiver.mean_wheel_mps >= 0:
        raise AssertionError('negative count delta lost')
    cases.append(b.finish())

    b = Builder('fresh_stuck_counts_fault_at_500ms').prepare()
    b.drive(1.)
    b.advance(500_000, refresh=True)
    b.check(state='FAULT_LATCHED', reason='motor_feedback_no_progress', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('reboot_invalidates_old_command').prepare()
    packet = b.drive(1.)
    b.event('reboot', now=0, boot_id='conformance-mcu-boot-b')
    if b.send(packet)[0]:
        raise AssertionError('old boot accepted')
    b.check(state='DISARMED', reason='boot', target_speed_mps=0., motor_counts=None)
    cases.append(b.finish())

    b = Builder('foreign_host_cannot_take_over').prepare()
    b.drive(1.)
    packet = b.packet(.5)
    packet['client'] = 'conformance-host-b'
    if b.send(packet) != [False, 'not_owner_or_armed']:
        raise AssertionError('foreign host accepted')
    b.check(state='ACTIVE', target_speed_mps=1., owner=CLIENT)
    cases.append(b.finish())

    b = Builder('clock_regression_latches_fault').prepare()
    b.drive(1.)
    b.event('tick', now=b.now-1)
    b.check(state='FAULT_LATCHED', reason='clock_regressed', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('duplicate_feedback_does_not_refresh_capture').prepare()
    b.drive(1.)
    capture, seq = b.now, b.seq
    for _ in range(5):
        b.now += 10_000
        b.event('encoder', sequence=seq, capture_us=capture, motor_counts=0)
        b.event('tick')
        if b.receiver.armed:
            b.drive(1.)
    b.check(state='FAULT_LATCHED', target_speed_mps=0.)
    cases.append(b.finish())

    b = Builder('int64_counts_above_double_exact_range')
    b.count = 2**53 + 111
    b.prepare()
    b.drive(1.)
    b.advance(10_000, increment=20)
    b.check(state='ACTIVE', motor_counts=2**53+131)
    cases.append(b.finish())
    return cases


def compare_values(expected, actual, path='$'):
    """bool/int를 구별하고 정수 시각/카운트는 완전 일치시킨다. 첫 차이를 반환한다."""
    # JSON decoder를 거쳐도 int64를 double로 바꾸지 않는 수신기를 요구한다.
    if isinstance(expected, float):
        try:
            finite = type(actual) in (int, float) and math.isfinite(actual)
        except OverflowError:
            finite = False
        if not finite:
            return f'{path}: finite numeric value required'
        if not math.isclose(expected, actual, rel_tol=0., abs_tol=FLOAT_TOLERANCE):
            return f'{path}: numeric mismatch'
    elif type(expected) is not type(actual):
        return f'{path}: type mismatch'
    elif isinstance(expected, dict):
        if expected.keys() != actual.keys():
            return f'{path}: object keys mismatch'
        for key in expected:
            issue = compare_values(expected[key], actual[key], f'{path}.{key}')
            if issue:
                return issue
    elif isinstance(expected, list):
        if len(expected) != len(actual):
            return f'{path}: length mismatch'
        for i, (left, right) in enumerate(zip(expected, actual)):
            issue = compare_values(left, right, f'{path}[{i}]')
            if issue:
                return issue
    elif expected != actual:
        return f'{path}: value mismatch'
    return None


def replay(case):
    config = ContractConfig(**case['config'])
    receiver = ActuationReceiver(config, boot_id=case['initial_boot'])
    responses = []
    for row in case['events']:
        receiver, response = apply_event(receiver, config, row['input'])
        responses.append(response)
    return dict(name=case['name'], responses=responses)


def compare_case(case, observed):
    if type(observed) is not dict or set(observed) != {'name', 'responses'}:
        return 'observation schema mismatch'
    if observed['name'] != case['name']:
        return 'case name mismatch'
    return compare_values([r['expected'] for r in case['events']], observed['responses'])


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: digest(ROOT/p) for p in FILES}
    cases = make_cases()
    observations = [replay(case) for case in cases]
    failures = [dict(name=c['name'], error=compare_case(c, o)) for c, o in zip(cases, observations)
                if compare_case(c, o)]
    bundle = dict(schema_version=1, scope='PC reference conformance; not firmware or physical stop proof',
                  clock='per-boot MCU monotonic integer microseconds', float_absolute_tolerance=FLOAT_TOLERANCE,
                  integer_representation='signed integer without float conversion; encode int64 exactly',
                  cases=cases, source_sha256=hashes, hardware_verified=False,
                  driver_pin_mapping_defined=False, physical_braking_verified=False)
    with (args.output/'vectors.json').open('x') as stream:
        json.dump(bundle, stream, indent=2, allow_nan=False)
    with (args.output/'python_observations.json').open('x') as stream:
        json.dump(observations, stream, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.write(ROOT/path, path)
    report = dict(cases=len(cases), events=sum(len(c['events']) for c in cases),
                  checkpoints=sum(len(c['checkpoints']) for c in cases), replay_failures=failures,
                  source_sha256=hashes, source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()),
                  runtime_changed=False, firmware_verified=False, ready_to_drive=False)
    with (args.output/'report.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in report.items() if k != 'source_sha256'}))
    return 0 if not failures and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
