"""장치 없는 수신 연결부: 기존 Python 링크 정책 대조 + 실제 envelope/잘못된 API 경계."""
import json
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from actuation_link_reference import make_policy_cases, RESULTS, STATES, FAULTS, frame
from actuation_conformance import BOOT, CLIENT
from arena_vehicle_interface.actuation_wire import encode


@pytest.fixture(scope='module', params=['sanitizers', 'size_optimized'])
def binary(tmp_path_factory, request):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('g++ required for portable ingress checks')
    path = tmp_path_factory.mktemp('ingress') / 'driver'
    flags = ['-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer'] if request.param == 'sanitizers' else ['-Os']
    run = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic',
                          '-fno-exceptions', '-fno-rtti', *flags,
                          '-I' + str(ROOT / 'src/arena_vehicle_interface/portable'),
                          str(ROOT / 'scripts/actuation_ingress_driver.cpp'), '-o', str(path)],
                         capture_output=True, text=True, timeout=90)
    assert run.returncode == 0, run.stderr
    return path


def execute(binary, lines):
    result = subprocess.run([str(binary)], input='\n'.join(lines) + '\n', capture_output=True, text=True, timeout=15)
    assert result.returncode == 0 and not result.stderr, (result.stdout, result.stderr)
    return [json.loads(line) for line in result.stdout.splitlines()]


def event_line(event):
    op, now, args = event['op'], event['now_us'], event['args']
    if op == 'link':
        suffix = args.get('hex', '') if args['kind'] == 'F' else args.get('size', '')
        return f"{args['kind']} {now} {suffix}"
    if op == 'encoder':
        return f"E {now} {args['sequence']} {args['capture_us']} {args['motor_counts']}"
    if op == 'physical_stop':
        return f"S {now} {int(args['pressed'])}"
    return {'offer': 'O', 'tick': 'T'}[op] + f' {now}'


CASES = make_policy_cases()


@pytest.mark.parametrize('case', CASES, ids=lambda case: case['name'])
def test_existing_python_link_policy(binary, case):
    # 기존 receiver/link 비교는 독립 출력 준비가 참이라는 명시 fixture다.
    rows = execute(binary, [f"I {case['link_timeout']} 0", 'R 0 1'] +
                   [event_line(row['input']) for row in case['events']])[1:]
    assert len(rows) == len(case['events'])
    for actual, event in zip(rows, case['events']):
        expected = event['expected']
        for field in ('state', 'reason', 'generation', 'accepted_seq', 'target_speed_mps', 'steering_rad',
                      'deadline_us', 'estop_held'):
            assert actual[field] == expected['status'][field], (case['name'], event['input'], field, actual)
        assert actual['ingress_fault'] == 0
        if event['input']['op'] != 'link':
            continue
        link, result = expected['link'], expected['result']
        assert RESULTS[actual['link_result']] == link['result']
        assert STATES[actual['link_state']] == link['state']
        assert FAULTS[actual['link_fault']] == link['fault']
        assert actual['breaks'] == link['breaks'] and actual['buffered'] == len(bytes.fromhex(link['buffer']))
        assert actual['stop'] == result['injected_stop']
        assert [(d['origin'], d['result']) for d in actual['deliveries']] == [
            (0 if d['origin'] == 'L' else 1, d['result']) for d in result['deliveries']]


def frame_lines(payload, now=220000):
    wire = encode(payload) if isinstance(payload, dict) else frame(json.dumps(payload).encode('ascii'))
    return [f'F {now} {wire[i:i+256].hex()}' for i in range(0, len(wire), 256)]


def prepared(layout=1, ready=True):
    lines = [f'I 30000 {layout}', 'C 0']
    if ready is not None:
        lines.append(f'R 0 {int(ready)}')
    for now in range(0, 220001, 10000):
        lines.extend([f'E {now} {now // 10000} {now} 0', f'T {now}'])
    return lines


def packet(kind, generation, lease, seq=1, **extra):
    return dict(v=1, kind=kind, boot=BOOT, generation=generation, lease=lease, client=CLIENT, seq=seq, **extra)


def envelope(command, request=''):
    return dict(type='command', request=request, command=command)


def test_envelope_fragmentation_request_and_stop(binary):
    lines = prepared() + ['O 220000'] + frame_lines(envelope(packet('ARM', 0, 1), '승인😀'))
    lines += ['O 220000'] + frame_lines(envelope(packet('DRIVE', 1, 2, 2, speed_mps=.5,
                        steering_rad=.2, source_age_us=0, ttl_us=100000)))
    lines += frame_lines(envelope(dict(v=1, kind='STOP'), '정지'))
    rows = execute(binary, lines)
    deliveries = [d for r in rows for d in r['deliveries']]
    assert [d['result'] for d in deliveries] == [[True, 'explicit_arm'], [True, 'drive'], [True, 'software_stop']]
    assert deliveries[0]['request'] == list(map(ord, '승인😀'))
    assert rows[-1]['state'] == 'ESTOP_LATCHED' and rows[-1]['target_speed_mps'] == 0
    assert rows[-1]['steering_rad'] == .2


@pytest.mark.parametrize('payload', [dict(v=1, kind='STOP'), dict(type='status', command=dict(v=1, kind='STOP')),
                                   dict(type='command', command=[]), [], {'type': 'command'}])
def test_wrong_envelope_or_non_object_cannot_dispatch(binary, payload):
    row = execute(binary, prepared() + frame_lines(payload))[-1]
    assert row['state'] == 'DISARMED' and row['generation'] == 0
    assert len(row['deliveries']) == 1 and not row['deliveries'][0]['called']


@pytest.mark.parametrize('timeout,layout', [(0, 0), (0, 1), (30000, 99)])
def test_invalid_configuration_is_latched(binary, timeout, layout):
    rows = execute(binary, [f'I {timeout} {layout}', 'C 0', 'X 0', 'C 0'])
    assert all(r['ingress_fault'] == 1 and r['state'] == 'ESTOP_LATCHED' for r in rows)
    assert rows[0]['stop'] == [True, 'software_stop']
    assert rows[1]['stop'] is None and rows[2]['stop'] is None


@pytest.mark.parametrize('now', [-1, -2**63, 2**63 - 1, 2**63 - 1 - 20000000 + 1])
def test_invalid_signed_time_never_cast_or_recovers(binary, now):
    rows = execute(binary, [f'I 30000 0', f'C {now}', 'X 0', 'C 0'])
    assert all(r['ingress_fault'] == 2 and r['state'] == 'FAULT_LATCHED' for r in rows)
    assert rows[0]['reason'] == 'invalid_clock' and rows[0]['target_speed_mps'] == 0


def test_crc_valid_bad_json_does_not_reuse_previous_command(binary):
    # make a CRC-valid frame with duplicate fields; encode() cannot create that from a dict.
    good = envelope(dict(v=1, kind='STOP'))
    bad = frame(b'{"type":"command","command":{"v":1,"v":1,"kind":"STOP"}}')
    rows = execute(binary, prepared() + frame_lines(good) + [f'F 220000 {bad.hex()}'])
    assert rows[-1]['generation'] == rows[-2]['generation']
    assert rows[-1]['deliveries'][0]['parse'] == 5 and not rows[-1]['deliveries'][0]['called']


def test_one_feed_can_deliver_multiple_commands_without_queue(binary):
    wire = encode(envelope(dict(v=1, kind='STOP'))) * 2
    assert len(wire) <= 256
    row = execute(binary, prepared() + [f'F 220000 {wire.hex()}'])[-1]
    assert row['generation'] == 2 and len(row['deliveries']) == 2
    assert all(d['result'] == [True, 'software_stop'] for d in row['deliveries'])


@pytest.mark.parametrize('ready', [False, True])
@pytest.mark.parametrize('command', [packet('ARM', 0, 1), {'kind': 'ARM'}, {'v': 2, 'kind': 'ARM'},
                                   packet('ARM', 99, 1), {'v': 1, 'kind': 'STOP'},
                                   packet('CLEAR', 0, 1), {'v': 1, 'kind': 'arm'}])
def test_output_gate_matches_actual_python_callback_precedence(binary, ready, command):
    """실제 on_tx 콜백 대조. ROS 노드/출력 감시는 실행하지 않고 준비값만 주입한다."""
    pytest.importorskip('rclpy')
    from types import SimpleNamespace
    from actuation_conformance import Builder
    from arena_vehicle_interface.actuation_ros import VirtualMCU
    from arena_vehicle_interface.actuation_wire import Decoder
    b = Builder('arm_gate').prepare(arm=False)
    b.receiver.offer(b.now)
    acks = []
    mcu = SimpleNamespace(receiver=b.receiver, decoder=Decoder(), now_us=lambda: b.now,
                          output_ready=lambda: ready, clock_failed=False, timer_entry=None, rx=None,
                          publish_status=lambda: None, send_offer=lambda: None,
                          capture_deadline=lambda *args: None,
                          send=lambda topic, payload: acks.append(payload))
    payload = envelope(command, 'gate-check')
    VirtualMCU.on_tx(mcu, SimpleNamespace(data=encode(payload)))
    row = execute(binary, prepared(ready=ready) + ['O 220000'] + frame_lines(payload))[-1]
    delivery = row['deliveries'][0]
    assert len(acks) == 1
    assert delivery['result'] == [acks[0]['accepted'], acks[0]['reason']]
    expected = b.receiver.status()
    for field in ('state', 'reason', 'generation', 'accepted_seq', 'target_speed_mps',
                  'steering_rad', 'deadline_us', 'estop_held'):
        assert row[field] == expected[field], (field, row, expected)
    blocked = command.get('kind') == 'ARM' and not ready
    assert delivery['output_gate_rejected'] is blocked
    assert delivery['called'] is not blocked


def test_default_closed_readiness_change_does_not_arm_and_stop_clear_still_work(binary):
    lines = prepared(ready=None) + ['O 220000']
    arm0 = envelope(packet('ARM', 0, 1), 'arm')
    lines += frame_lines(arm0) + ['R 220000 1'] + frame_lines(arm0)
    lines += ['R 220000 0'] + frame_lines(envelope({'v': 1, 'kind': 'STOP'}, 'stop'))
    lines += ['O 220000'] + frame_lines(envelope(packet('CLEAR', 2, 2), 'clear'))
    lines += ['O 220000'] + frame_lines(envelope(packet('ARM', 3, 3), 'rearm'))
    rows = execute(binary, lines)
    results = [d['result'] for r in rows for d in r['deliveries']]
    assert results == [[False, 'output_not_ready'], [True, 'explicit_arm'], [True, 'software_stop'],
                       [True, 'explicit_clear'], [False, 'output_not_ready']]
    ready_row = next(r for i, r in enumerate(rows) if i > 0 and rows[i-1]['deliveries'] and
                     rows[i-1]['deliveries'][0]['result'] == [False, 'output_not_ready'])
    assert ready_row['state'] == 'DISARMED' and ready_row['generation'] == 0
    assert rows[-1]['state'] == 'DISARMED' and rows[-1]['target_speed_mps'] == 0


def test_expired_drain_uses_readiness_at_dispatch_not_when_bytes_arrived(binary):
    wire = encode(envelope(packet('ARM', 0, 1), 'drain'))
    # 합법 길이지만 불완전한 앞 프레임 안에 완성 ARM을 숨긴다.
    hidden = b'IA' + (1300).to_bytes(2, 'little') + wire
    assert len(hidden) <= 256
    rows = execute(binary, prepared() + ['O 220000', f'F 220000 {hidden.hex()}', 'R 240000 0', 'P 250001'])
    assert rows[-3]['deliveries'] == []
    d = rows[-1]['deliveries'][0]
    assert d['origin'] == 1 and d['output_gate_rejected'] and not d['called']
    assert d['result'] == [False, 'output_not_ready'] and rows[-1]['generation'] == 0


def test_not_ready_gate_does_not_hide_receiver_clock_regression(binary):
    rows = execute(binary, prepared(ready=False) + ['T 230000'] + frame_lines(envelope({'kind': 'ARM'})))
    assert rows[-1]['state'] == 'FAULT_LATCHED' and rows[-1]['target_speed_mps'] == 0
    d = rows[-1]['deliveries'][0]
    assert d['called'] and not d['output_gate_rejected'] and d['result'] == [False, 'invalid_message']
