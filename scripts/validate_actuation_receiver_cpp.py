#!/usr/bin/env python3
"""C++ 수신 상태기계와 기존 Python ActuationReceiver의 이벤트별 대조. Python은 독립 비교 기준으로만 쓴다.

운영 ROS/MCU 연결·보드 드라이버·전기 출력·실물 시험이 아니다.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import struct
import subprocess
import zipfile

from actuation_conformance import (
    ROOT, BOOT, CLIENT, Builder, apply_event, compare_values, digest, make_cases as conformance_cases, motor_config,
)
from actuation_transport_conformance import make_cases as transport_cases, apply_transport_event
from actuation_link_reference import make_policy_cases, apply_link_event, LinkOracle, link_op_from_event
import validate_actuation_command_cpp as command_bench
from arena_vehicle_interface.actuation_contract import ActuationReceiver, ContractConfig
from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.drive_feedback import MODE_MOTOR

FILES = [
    'scripts/validate_actuation_receiver_cpp.py', 'scripts/actuation_receiver_cpp_driver.cpp',
    'tests/test_actuation_receiver_cpp.py', 'scripts/check_actuation_receiver_mutants.py',
    'src/arena_vehicle_interface/portable/actuation_receiver.hpp',
    'src/arena_vehicle_interface/portable/actuation_command.hpp',
    'src/arena_vehicle_interface/portable/actuation_link.hpp',
    'src/arena_vehicle_interface/portable/actuation_wire.hpp',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py',
    'scripts/actuation_conformance.py', 'scripts/actuation_transport_conformance.py',
    'scripts/actuation_link_reference.py', 'scripts/validate_actuation_command_cpp.py',
    'scripts/actuation_command_reference.py',
]
INT64 = (-2**63, 2**63 - 1)
STOP = {'v': 1, 'kind': 'STOP'}


def bits(value):
    return struct.pack('>d', float(value)).hex()


def config_line(config, boot):
    c = config if isinstance(config, dict) else config.__dict__
    return ' '.join(['G', '0' if c['feedback_mode'] == MODE_MOTOR else '1', str(c['lease_us']),
                     str(c['feedback_timeout_us']), str(c['settle_us']), str(c['max_control_gap_us']),
                     str(c['ticks_per_rev']), str(c['motor_no_progress_us']), bits(c['wheel_radius_m']),
                     bits(c['max_speed_mps']), bits(c['max_steering_rad']), bits(c['stopped_wheel_mps']),
                     bits(c['max_wheel_mps']), bits(c['gear_ratio']), bits(c['motor_progress_min_command_mps']),
                     boot.encode('ascii').hex()])


def int_arg(value):
    return str(value) if type(value) is int and INT64[0] <= value <= INT64[1] else 'X'


def event_line(event, config):
    op, now, args = event['op'], event['now_us'], event['args']
    if op == 'reboot':
        return config_line(config, args['boot_id'])
    if op == 'tick':
        return f'T {now}'
    if op == 'encoder':
        return f"E {now} {int_arg(args['sequence'])} {int_arg(args['capture_us'])} {int_arg(args['motor_counts'])}"
    if op == 'legacy_encoder':
        return (f"Y {now} {int_arg(args['sequence'])} {int_arg(args['capture_us'])} {int_arg(args['left_ticks'])} "
                f"{int_arg(args['right_ticks'])}")
    if op == 'physical_stop':
        return f"S {now} {int(args['pressed'])}"
    if op == 'offer':
        return f'O {now}'
    if op == 'command':
        return f"M {now} 0 {bytes.fromhex(args['frame_hex'])[4:-4].hex()}"
    if op == 'bytes':
        return f"K {now} {args['hex']}"
    if op == 'decoder_reset':
        return f'R {now}'
    if op == 'link':
        kind = args['kind']
        if kind == 'F':
            return f"F {now} {args['hex']}"
        return {'P': f'P {now}', 'C': f'N {now}', 'D': f'D {now}', 'X': f'X {now}',
                'Z': f"W {now} {args.get('size', 0)}"}[kind]
    raise ValueError(op)


def normalize_python(event, response):
    """Python 응답 → C++ 출력과 같은 비교 형식(r, s)."""
    op, result, status = event['op'], response['result'], response['status']
    if op == 'tick':
        result = 'STATUS'
    elif op == 'offer':
        result = {k: result[k] for k in ('lease', 'lease_issued_us', 'lease_expires_us')}
    elif op == 'bytes':
        result = dict(deliveries=[[d['result'], d['status']] for d in result], injected_stop=None,
                      buffered=response['decoder']['buffered_bytes'])
    elif op == 'decoder_reset':
        result = dict(deliveries=[], injected_stop=None, buffered=0)
    elif op == 'link':
        result = dict(deliveries=[d['result'] for d in result['deliveries'] if d['result'] != [False, 'json_rejected']],
                      injected_stop=result['injected_stop'], buffered=len(response['link']['buffer']) // 2)
    return dict(r=result, s=status)


def normalize_cpp(event, parsed):
    if event['op'] == 'link':
        r = dict(parsed['r'], deliveries=[d[0] for d in parsed['r']['deliveries']])
        return dict(r=r, s=parsed['s'])
    return parsed


# ---------------------------------------------------------------- 새 수동 반례

def invalid_drive(b, **changes):
    packet = b.packet(.7)
    packet.update(speed_mps='fast', **changes)
    return packet


def manual_cases():
    """구현과 별개로 적은 기존 Python 의미의 반례. checkpoint는 Python에서 먼저 확인된다."""
    cases = []

    def active(name):
        b = Builder(name).prepare()
        b.drive(1.)
        b.check(state='ACTIVE', target_speed_mps=1.)
        return b

    b = active('wrong_generation_and_invalid_drive_is_session')
    if b.send(invalid_drive(b, generation=99)) != [False, 'session']:
        raise AssertionError('order')
    b.check(state='ACTIVE', target_speed_mps=1.)
    cases.append(b.finish())

    b = active('expired_lease_and_invalid_drive_is_expired_lease')
    if b.send(invalid_drive(b, lease=9999)) != [False, 'expired_lease']:
        raise AssertionError('order')
    b.check(state='ACTIVE', target_speed_mps=1.)
    cases.append(b.finish())

    b = active('wrong_owner_and_invalid_drive_is_not_owner')
    if b.send(invalid_drive(b, client='intruder')) != [False, 'not_owner_or_armed']:
        raise AssertionError('order')
    b.check(state='ACTIVE', target_speed_mps=1.)
    cases.append(b.finish())

    b = active('old_seq_and_invalid_drive_is_sequence')
    if b.send(invalid_drive(b, seq=1)) != [False, 'sequence']:
        raise AssertionError('order')
    b.check(state='ACTIVE', target_speed_mps=1.)
    cases.append(b.finish())

    b = active('valid_owner_invalid_drive_faults_and_zeroes_target')
    for field, value in [('speed_mps', 'fast')]:
        if b.send(invalid_drive(b)) != [False, 'invalid_drive']:
            raise AssertionError('fault')
    b.check(state='FAULT_LATCHED', reason='invalid_drive', target_speed_mps=0., steering_rad=.1, deadline_us=None)
    cases.append(b.finish())

    for name, change in [('steering_out_of_range', dict(steering_rad=.5)), ('ttl_above_lease', dict(ttl_us=100_001)),
                         ('age_equals_ttl', dict(source_age_us=100_000)), ('speed_bool', dict(speed_mps=True))]:
        b = active('valid_owner_' + name + '_faults')
        packet = b.packet(.7); packet.update(change)
        if b.send(packet) != [False, 'invalid_drive']:
            raise AssertionError(name)
        b.check(state='FAULT_LATCHED', reason='invalid_drive', target_speed_mps=0.)
        cases.append(b.finish())

    b = active('stop_at_same_time_as_command_deadline')
    b.advance(90_000)
    b.now += 10_000  # 320 ms = 명령 마감, 이 시각 tick 없이 STOP 도착
    b.send(dict(STOP))
    b.check(state='ESTOP_LATCHED', reason='software_stop', target_speed_mps=0.)
    cases.append(b.finish())

    b = active('physical_release_then_stale_clear_and_arm')
    old_offer = b.event('offer')
    b.event('physical_stop', pressed=True)
    b.event('physical_stop', pressed=False)
    b.advance(220_000)
    stale_clear = b.producer.control(old_offer, 'CLEAR')
    if b.send(stale_clear) != [False, 'session']:
        raise AssertionError('stale clear')
    b.control('CLEAR')
    b.check(state='DISARMED', reason='explicit_clear')
    if b.send(b.producer.control(old_offer, 'ARM')) != [False, 'session']:
        raise AssertionError('stale arm')
    b.control('ARM')
    b.check(state='ARMED', reason='explicit_arm')
    cases.append(b.finish())

    b = Builder('physical_held_blocks_clear').prepare()
    b.event('physical_stop', pressed=True)
    b.advance(220_000)
    if b.control('CLEAR') != [False, 'not_ready']:
        raise AssertionError('held')
    b.check(state='ESTOP_LATCHED', estop_held=True)
    cases.append(b.finish())

    b = Builder('settle_boundary_for_clear').prepare()
    b.send(dict(STOP))
    b.advance(190_000, increment=0)
    b.check(state='ESTOP_LATCHED')
    cases.append(b.finish())

    b = Builder('lease_capacity_evicts_oldest').prepare()
    offers = [b.event('offer') for _ in range(40)]
    early, late = offers[0], offers[-1]
    if b.send(b.producer.control(early, 'DISARM')) != [False, 'expired_lease']:
        raise AssertionError('evict')
    if b.send(b.producer.control(late, 'DISARM')) != [True, 'explicit_disarm']:
        raise AssertionError('keep')
    b.check(state='DISARMED', reason='explicit_disarm')
    cases.append(b.finish())

    b = active('encoder_gap_while_armed')
    b.now += 60_000
    b.feedback(0)
    b.event('tick')
    b.check(state='FAULT_LATCHED')
    cases.append(b.finish())

    b = active('reverse_order_feedback_ignored')
    b.now += 10_000
    b.event('encoder', sequence=b.seq - 1, capture_us=b.now, motor_counts=b.count)
    b.event('encoder', sequence=b.seq + 1, capture_us=b.now - 20_000, motor_counts=b.count)
    b.event('tick')
    b.check(state='ACTIVE')
    cases.append(b.finish())

    b = active('int64_count_extremes_implausible')
    b.now += 10_000
    b.event('encoder', sequence=b.seq + 1, capture_us=b.now, motor_counts=-2**63)
    b.now += 10_000
    b.event('encoder', sequence=b.seq + 2, capture_us=b.now, motor_counts=2**63 - 1)
    b.check(state='FAULT_LATCHED', reason='encoder_implausible')
    cases.append(b.finish())

    b = Builder('int64_count_near_max_small_delta')
    b.count = 2**63 - 1000
    b.prepare()
    b.drive(1.)
    b.advance(10_000, increment=20)
    b.check(state='ACTIVE', motor_counts=2**63 - 980)
    cases.append(b.finish())

    b = active('physical_press_while_fault_latched_becomes_estop')
    b.advance(100_000)
    b.check(state='FAULT_LATCHED', reason='command_expired')
    b.event('physical_stop', pressed=True)
    b.check(state='ESTOP_LATCHED', reason='physical_stop', estop_held=True)
    cases.append(b.finish())

    b = Builder('expired_lease_rejected').prepare(False)
    old = b.event('offer')
    b.advance(100_000)
    if b.send(b.producer.control(old, 'ARM')) != [False, 'expired_lease']:
        raise AssertionError('expired lease accepted')
    b.check(state='DISARMED')
    cases.append(b.finish())

    b = Builder('int64_count_from_min_to_max')
    b.count = -2**63
    b.prepare()
    b.drive(1.)
    b.now += 10_000
    b.event('encoder', sequence=b.seq + 1, capture_us=b.now, motor_counts=2**63 - 1)
    b.check(state='FAULT_LATCHED', reason='encoder_implausible')
    cases.append(b.finish())

    b = Builder('invalid_encoder_inputs')
    b.prepare()
    b.drive(1.)
    b.now += 10_000
    b.event('encoder', sequence=b.seq + 1, capture_us=b.now, motor_counts=2**63)
    b.check(state='FAULT_LATCHED', reason='invalid_encoder')
    cases.append(b.finish())
    return cases


class LegacyBuilder(Builder):
    def __init__(self, name):
        super().__init__(name)
        self.config = ContractConfig()
        self.receiver = ActuationReceiver(self.config, boot_id=BOOT)
        self.right = 0

    def feedback(self, count=None, right=None):
        self.seq += 1
        if count is not None:
            self.count = count
        self.right = self.count if right is None else right
        return self.event('legacy_encoder', sequence=self.seq, capture_us=self.now, left_ticks=self.count,
                          right_ticks=self.right)


def legacy_cases():
    cases = []
    b = LegacyBuilder('legacy_drive_and_left_right').prepare()
    b.drive(1.)
    b.advance(10_000, increment=40)
    b.check(state='ACTIVE')
    cases.append(b.finish())

    b = LegacyBuilder('legacy_opposite_rotation_not_stationary').prepare()
    b.send(dict(STOP))
    for _ in range(25):
        b.now += 10_000
        b.feedback(b.count + 30, b.right - 30)
        b.event('tick')
    if b.control('CLEAR') != [False, 'not_ready']:
        raise AssertionError('opposite rotation treated as stationary')
    cases.append(b.finish())

    b = LegacyBuilder('motor_feedback_in_legacy_mode_faults').prepare()
    b.drive(1.)
    b.event('encoder', sequence=99, capture_us=b.now, motor_counts=0)
    b.check(state='FAULT_LATCHED', reason='feedback_mode_mismatch')
    cases.append(b.finish())
    return cases


# ---------------------------------------------------------------- 평가

def python_responses(case, kind):
    """사례의 저장된 기대 응답(Python 생성)을 그대로 쓴다."""
    return [row['expected'] for row in case['events']]


def build_plan():
    plan = []
    for case in conformance_cases():
        plan.append(dict(name='conformance_' + case['name'], kind='contract', case=case))
    for case in transport_cases():
        plan.append(dict(name='transport_' + case['name'], kind='transport', case=case))
    for case in make_policy_cases():
        plan.append(dict(name='link_' + case['name'], kind='link', case=case))
    for case in manual_cases():
        plan.append(dict(name='manual_' + case['name'], kind='contract', case=case))
    for case in legacy_cases():
        plan.append(dict(name='legacy_' + case['name'], kind='contract', case=case))
    return plan


def receive_plan():
    """명령 parser 단계의 상태별 receive 사례(3,667)를 C++ 상태기계로 계산한다."""
    states = {}
    for name, b in [('active', Builder('active').prepare()), ('armed', Builder('armed').prepare()),
                    ('disarmed', Builder('disarmed').prepare(False)), ('estop', Builder('estop').prepare())]:
        if name == 'active':
            b.drive(1.)
        if name == 'estop':
            b.send(dict(STOP)); b.advance(220_000)
        b.event('offer')
        states[name] = b
    _, rows = command_bench.receive_cases()
    return states, rows


MAX_NOW = 2**63 - 1 - 2 * 10_000_000


def boundary_checks(binary):
    """Python 임의 정밀과 일부러 다른 C++ 경계. 기대값은 손으로 적은 C++ 설계값이며 Python 동작을 함께 기록한다."""
    g = config_line(motor_config().__dict__, BOOT)
    scripts = [
        ('now_at_limit_ok', [g, f'T {MAX_NOW}'], dict(state='DISARMED', reason='boot'),
         lambda r: (r.tick(MAX_NOW), r.status())[1]),
        ('now_above_limit_invalid_clock', [g, f'T {MAX_NOW + 1}'], dict(state='FAULT_LATCHED', reason='invalid_clock'),
         lambda r: (r.tick(MAX_NOW + 1), r.status())[1]),
        ('negative_now_invalid_clock', [g, 'T -1'], dict(state='FAULT_LATCHED', reason='invalid_clock'),
         lambda r: (r.tick(-1), r.status())[1]),
        ('generation_last_value_used_once', [g, f'Z 0 {2**63 - 2} 0', 'S 0 1'],
         dict(state='ESTOP_LATCHED', reason='physical_stop', generation=2**63 - 1), None),
        ('generation_exhausted_latches', [g, f'Z 0 {2**63 - 2} 0', 'S 0 1', 'S 0 1'],
         dict(state='FAULT_LATCHED', reason='counter_exhausted', generation=2**63 - 1, target_speed_mps=0.0),
         lambda r: (setattr(r, 'generation', 2**63 - 2), r.set_estop(True, 0), r.set_estop(True, 0), r.status())[-1]),
        ('lease_sequence_last_value', [g, f'Z 0 0 {2**63 - 2}', 'O 0'], dict(state='DISARMED', lease=2**63 - 1), None),
        ('lease_sequence_exhausted', [g, f'Z 0 0 {2**63 - 1}', 'O 0'],
         dict(state='FAULT_LATCHED', reason='counter_exhausted', offer='OFFER_ERROR'), None),
    ]
    rows = []
    for name, lines, expect, python in scripts:
        _, proc = run(binary, lines)
        try:
            out = [json.loads(line) for line in proc.stdout.splitlines()]
        except ValueError:
            out = []
        last = out[-1] if out else {}
        observed = dict(last.get('s', {}))
        if isinstance(last.get('r'), dict) and 'lease' in last['r']:
            observed['lease'] = last['r']['lease']
        if last.get('r') == 'OFFER_ERROR':
            observed['offer'] = 'OFFER_ERROR'
        diff = {k: (v, observed.get(k)) for k, v in expect.items() if observed.get(k) != v}
        python_status = None
        if python is not None:
            receiver = ActuationReceiver(motor_config(), boot_id=BOOT)
            status = python(receiver)
            python_status = {k: status[k] for k in ('state', 'reason', 'generation')}
        rows.append(dict(name=name, passed=proc.returncode == 0 and not diff, diff=str(diff) if diff else None,
                         cpp=dict((k, observed.get(k)) for k in expect), python_same_input=python_status))
    return rows


def run(binary, lines):
    stdin = '\n'.join(lines) + '\n'
    proc = subprocess.run([str(binary)], input=stdin, capture_output=True, text=True, timeout=600)
    return stdin, proc


def evaluate(binary, plan=None, receive=None):
    plan = plan or build_plan()
    states, rows = receive or receive_plan()
    lines, expected, owners = [], [], []
    for item in plan:
        case = item['case']
        config = case['config']
        if item['kind'] == 'link':
            lines.append(config_line(config, case['initial_boot']))
            expected.append(None); owners.append((item['name'], -1, None))
            lines.append(f"Q {case['link_timeout']}")
            expected.append(None); owners.append((item['name'], -1, None))
        else:
            lines.append(config_line(config, case['initial_boot']))
            expected.append(None); owners.append((item['name'], -1, None))
        for index, row in enumerate(case['events']):
            lines.append(event_line(row['input'], config))
            expected.append(normalize_python(row['input'], row['expected']))
            owners.append((item['name'], index, row['input']))
    receive_start = len(lines)
    for name, b in states.items():
        lines.append(config_line(b.config.__dict__, BOOT))
        expected.append(None); owners.append(('state_' + name, -1, None))
        for index, row in enumerate(b.events):
            lines.append(event_line(row['input'], b.config.__dict__))
            expected.append(normalize_python(row['input'], row['expected']))
            owners.append(('state_' + name, index, row['input']))
        lines.append('V'); expected.append(None); owners.append(('state_' + name, -1, None))
        for row in [r for r in rows if r['state'] == name]:
            lines.append('U'); expected.append(None); owners.append((row['name'], -1, None))
            receiver = deepcopy(b.receiver)
            message = command_bench.production_decode(row['payload'])
            if row['layout'] == 1:
                message = message if message is None else message
            from actuation_command_reference import deliver
            result = deliver(receiver, row['layout'], message, b.now)
            result = {'not_delivered': 'NOT_DELIVERED', 'envelope_ignored': 'ENVELOPE_IGNORED'}.get(result, result) \
                if isinstance(result, str) else result
            lines.append(f"M {b.now} {row['layout']} {row['payload'].hex()}")
            expected.append(dict(r=result, s=receiver.status()))
            owners.append((row['name'], 0, dict(op='command')))
    stdin, proc = run(binary, lines)
    actual = proc.stdout.splitlines()
    failures = []
    compared = 0
    for i, (want, owner) in enumerate(zip(expected, owners)):
        if i >= len(actual):
            failures.append(dict(case=owner[0], index=owner[1], error='missing output'))
            break
        if want is None:
            continue
        try:
            got = normalize_cpp(owner[2], json.loads(actual[i]))
        except (ValueError, KeyError, TypeError, IndexError) as exc:  # 비JSON 출력(nan 등)도 불일치로 남긴다
            failures.append(dict(case=owner[0], index=owner[1], error=f'unparsable output: {exc}',
                                 actual=actual[i][:400]))
            continue
        compared += 1
        if got != want:
            failures.append(dict(case=owner[0], index=owner[1], op=owner[2]['op'] if owner[2] else None,
                                 error=compare_values(want, got) or 'exact mismatch',
                                 expected=json.dumps(want)[:400], actual=json.dumps(got)[:400]))
    boundaries = boundary_checks(binary)
    report = dict(
        boundary_checks=boundaries,
        cases=len(plan), case_kinds={k: sum(1 for p in plan if p['kind'] == k) for k in ('contract', 'transport', 'link')},
        manual_cases=sum(1 for p in plan if p['name'].startswith(('manual_', 'legacy_'))),
        events_compared=compared, receive_cases=len(rows), driver_lines=len(lines), binary_exit_code=proc.returncode,
        mismatches=failures[:200], mismatch_count=len(failures),
        passed=(proc.returncode == 0 and not proc.stderr and not failures and len(actual) == len(lines) and
                all(row['passed'] for row in boundaries)),
        state_machine_ported=True, ros_or_runtime_connected=False, wcet_measured=False,
        mcu_toolchain_verified=False, firmware_verified=False, ready_to_drive=False)
    return report, stdin, proc.stdout, proc.stderr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: digest(ROOT/p) for p in FILES}
    report, stdin, stdout, stderr = evaluate(args.binary.resolve())
    report.update(source_sha256=hashes, binary_sha256=digest(args.binary),
                  source_unchanged=all(digest(ROOT/p) == v for p, v in hashes.items()))
    for name, value in [('stdin.txt', stdin), ('stdout.txt', stdout), ('stderr.txt', stderr)]:
        (args.output/name).write_text(value, encoding='utf-8', newline='\n')
    with (args.output/'report.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            if (ROOT/path).exists():
                archive.write(ROOT/path, path)
    print(json.dumps({k: v for k, v in report.items() if k not in ('source_sha256', 'mismatches')}))
    return 0 if report['passed'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
