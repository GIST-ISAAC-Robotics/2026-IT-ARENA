#!/usr/bin/env python3
"""C++ FrameLink(부분 프레임 절대 기한·연결/오류 정책) PC 대조. 운영 timeout·펌웨어 검증 아님."""
import argparse
import json
from pathlib import Path
import random
import struct
import subprocess
import zipfile

from actuation_conformance import ROOT, digest
from actuation_link_reference import (
    CHUNK, U64_MAX, LinkOracle, compare_policy_case, frame, link_ops, make_policy_cases,
    op_command, parse_line, replay_policy, snapshot_line,
)
import validate_actuation_wire_cpp as wire
from arena_vehicle_interface.actuation_wire import MAGIC, MAX_PAYLOAD, encode

FILES = [
    'scripts/validate_actuation_link_cpp.py', 'scripts/actuation_link_reference.py',
    'scripts/actuation_link_cpp_driver.cpp', 'tests/test_actuation_link_cpp.py',
    'src/arena_vehicle_interface/portable/actuation_link.hpp',
    'src/arena_vehicle_interface/portable/actuation_wire.hpp',
    'scripts/validate_actuation_wire_cpp.py', 'scripts/actuation_wire_cpp_driver.cpp',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
    'scripts/actuation_conformance.py', 'scripts/actuation_transport_conformance.py',
]
STOP = encode({'v': 1, 'kind': 'STOP'})
T = 1_000  # 링크 단위 사례의 시험 tick. 시간 단위/운영값과 무관하다.


def hidden(inner=b''):
    return MAGIC + struct.pack('<H', MAX_PAYLOAD) + inner


def make_link_cases():
    """(name, timeout, ops, checks). checks는 수동으로 적은 기대값(index→필드)이다."""
    cases = []

    def add(name, ops, checks=None, timeout=T):
        cases.append(dict(name=name, timeout=timeout, ops=ops, checks=checks or {}))

    for label, at, delivered in [('before', 1_099, True), ('exact', 1_100, False), ('after', 1_101, False)]:
        add(f'feed_completion_{label}_deadline',
            [('C', 0), ('F', 100, STOP[:10]), ('F', at, STOP[10:])],
            {2: dict(frames=[['L', STOP[4:-4].hex()]] if delivered else [],
                     timeouts=0 if delivered else 1)})
        add(f'poll_{label}_deadline', [('C', 0), ('F', 100, STOP[:10]), ('P', at)],
            {2: dict(buffer=STOP[:10].hex() if delivered else '', anchor=100 if delivered else None,
                     timeouts=0 if delivered else 1)})
    add('silence_polls_do_nothing', [('C', 0)] + [('P', t) for t in (0, 999, 1_000, 10**6)],
        {4: dict(timeouts=0, errors=0, buffer='', anchor=None)})
    trickle = [('C', 0), ('F', 0, hidden())] + [('F', t, b'x') for t in range(100, 1_000, 100)]
    trickle += [('F', 1_000, b'y'), ('F', 1_000, STOP)]
    add('trickle_cannot_extend_deadline', trickle,
        {10: dict(anchor=0, timeouts=0), 11: dict(timeouts=1, buffer='', anchor=None, frames=[]),
         12: dict(frames=[['L', STOP[4:-4].hex()]], buffer='')})
    add('empty_feed_does_not_move_anchor', [('C', 0), ('F', 10, b'I'), ('F', 500, b''), ('P', 1_009),
                                            ('F', 1_010, b'')],
        {2: dict(anchor=10), 3: dict(anchor=10, buffer='49'), 4: dict(anchor=None, timeouts=1)})
    add('lone_magic_byte_expires', [('C', 0), ('F', 5, b'noiseI'), ('P', 1_005), ('F', 1_005, b'A')],
        {1: dict(buffer='49', anchor=5), 2: dict(buffer='', timeouts=1), 3: dict(buffer='')})
    add('frames_and_partial_tail_reanchor', [('C', 0), ('F', 0, STOP[:20]), ('F', 900, STOP[20:] + STOP + STOP[:7]),
                                             ('P', 1_000), ('P', 1_899), ('P', 1_900)],
        {2: dict(anchor=900, buffer=STOP[:7].hex(), frames=[['L', STOP[4:-4].hex()]] * 2),
         3: dict(anchor=900, timeouts=0), 4: dict(timeouts=0), 5: dict(timeouts=1, buffer='')})
    add('old_bytes_remaining_keep_anchor',
        [('C', 0), ('F', 0, hidden(STOP[:12])), ('F', 600, STOP[12:] + STOP[:3]), ('P', 1_000)],
        {2: dict(anchor=0), 3: dict(frames=[['X', STOP[4:-4].hex()]], buffer='', timeouts=1)})
    add('hidden_stop_drained_then_live_frame_same_feed',
        [('C', 0), ('F', 0, hidden(STOP)), ('F', 1_000, STOP[:5] + STOP)],
        {2: dict(frames=[['X', STOP[4:-4].hex()], ['L', STOP[4:-4].hex()]], timeouts=1, buffer='')})
    add('expired_tail_is_garbage_not_joined',
        [('C', 0), ('F', 0, STOP[:12]), ('F', 1_000, STOP[12:] + STOP)],
        {2: dict(frames=[['L', STOP[4:-4].hex()]], timeouts=1, buffer='')})
    add('bad_crc_and_bad_length_resync',
        [('C', 0), ('F', 1, STOP[:-1] + bytes([STOP[-1] ^ 1]) + STOP),
         ('F', 2, MAGIC + b'\xff\xff' + STOP), ('F', 3, MAGIC + b'\x00\x00' + STOP)],
        {1: dict(frames=[['L', STOP[4:-4].hex()]]), 2: dict(frames=[['L', STOP[4:-4].hex()]]),
         3: dict(frames=[['L', STOP[4:-4].hex()]], buffer='')})
    add('disconnect_reconnect_never_joins',
        [('C', 0), ('F', 10, STOP[:12]), ('D', 20), ('F', 30, STOP[12:]), ('P', 40), ('C', 50),
         ('F', 60, STOP[12:]), ('F', 70, STOP), ('D', 80), ('D', 90)],
        {2: dict(state='disconnected', dropped=12, breaks=1), 3: dict(result='not_connected', rejected=1),
         4: dict(result='not_connected', rejected=1), 5: dict(sessions=2, breaks=1),
         6: dict(frames=[], buffer=''), 7: dict(frames=[['L', STOP[4:-4].hex()]]),
         9: dict(result='not_connected', breaks=2)})
    add('implicit_reconnect_counts_break', [('C', 0), ('F', 1, STOP[:9]), ('C', 2), ('F', 3, STOP[9:])],
        {2: dict(breaks=1, sessions=2, dropped=9), 3: dict(frames=[])})
    add('oversized_feed_latches_and_discards_same_call',
        [('C', 0), ('F', 1, STOP[:5]), ('F', 2, STOP + b'\0' * (CHUNK + 1 - len(STOP))), ('F', 3, STOP),
         ('P', 4), ('C', 5), ('D', 6), ('X', 7), ('X', 8), ('F', 9, STOP), ('C', 10), ('F', 11, STOP)],
        {2: dict(result='fault_raised', fault='input_budget', frames=[], dropped=5, breaks=1),
         3: dict(result='fault_latched', rejected=1, frames=[]), 6: dict(rejected=4),
         7: dict(result='ok', state='disconnected', fault='none', last_fault='input_budget'),
         8: dict(result='not_faulted'), 9: dict(result='not_connected'),
         11: dict(frames=[['L', STOP[4:-4].hex()]])})
    add('exact_chunk_limit_is_accepted', [('C', 0), ('F', 1, STOP + b'\0' * (CHUNK - len(STOP)))],
        {1: dict(result='ok', frames=[['L', STOP[4:-4].hex()]])})
    add('null_pointer_feed', [('C', 0), ('Z', 1, 0), ('Z', 2, 1)],
        {1: dict(result='ok'), 2: dict(result='fault_raised', fault='input_budget')})
    add('clock_regression_latches_and_discards_same_call',
        [('C', 100), ('F', 100, STOP[:6]), ('F', 100, STOP[6:9]), ('F', 99, STOP), ('P', 100),
         ('X', 50), ('C', 50), ('F', 60, STOP), ('P', 59)],
        {2: dict(result='ok', anchor=100), 3: dict(result='fault_raised', fault='clock_regressed', frames=[],
                                                    dropped=9),
         4: dict(result='fault_latched'), 5: dict(state='disconnected'), 7: dict(frames=[['L', STOP[4:-4].hex()]]),
         8: dict(result='fault_raised', fault='clock_regressed', breaks=2)})
    add('clock_regression_while_disconnected', [('P', 10), ('P', 9), ('X', 0), ('C', 0)],
        {1: dict(result='fault_raised', breaks=1), 3: dict(state='connected')})
    add('u64_edge_timeout_one', [('C', U64_MAX - 2), ('F', U64_MAX - 2, STOP[:3]), ('P', U64_MAX - 2),
                                 ('F', U64_MAX - 1, STOP[:3]), ('P', U64_MAX)],
        {2: dict(anchor=U64_MAX - 2), 3: dict(timeouts=1, anchor=U64_MAX - 1), 4: dict(timeouts=2, anchor=None)},
        timeout=1)
    add('u64_edge_timeout_max_from_zero', [('C', 0), ('F', 0, STOP[:3]), ('P', U64_MAX - 1), ('P', U64_MAX)],
        {2: dict(timeouts=0, anchor=0), 3: dict(timeouts=1, anchor=None)}, timeout=U64_MAX)
    add('u64_edge_timeout_max_from_one', [('C', 1), ('F', 1, STOP[:3]), ('P', U64_MAX)],
        {2: dict(timeouts=0, anchor=1)}, timeout=U64_MAX)
    add('u64_wrap_is_regression', [('C', U64_MAX), ('F', U64_MAX, STOP[:3]), ('F', 0, STOP)],
        {2: dict(result='fault_raised', fault='clock_regressed', frames=[])})
    add('zero_timeout_is_permanent_config_fault', [('C', 0), ('F', 0, STOP), ('X', 1), ('P', 2)],
        {0: dict(result='fault_latched', fault='invalid_config'), 2: dict(result='fault_latched')}, timeout=0)
    big = bytes(range(256)) * 6
    packet = frame(big)
    add('maximum_payload_completes_before_deadline',
        [('C', 0)] + [('F', i, packet[j:j+CHUNK]) for i, j in enumerate(range(0, len(packet), CHUNK))]
        + [('P', 999)],
        {7: dict(frames=[['L', big.hex()]], buffer=''), 8: dict(timeouts=0)})

    # 기존 C++ 프레임 사례를 같은 시각·사실상 무한 기한으로 넣어 시간 정책이 바이트 결과를 바꾸지 않음을 본다.
    for case in wire.make_cases():
        ops = [('C', 0)]
        for op, data in case['operations']:
            if op == 'R':
                ops.append(('C', 0))
            elif op == 'F':
                ops.append(('F', 0, data))
        if len(ops) > 2:
            add('framing_' + case['name'], ops, timeout=U64_MAX)

    rng = random.Random(0x20261022)
    payloads = [STOP[4:-4], b'{"v":1,"kind":"DRIVE","seq":1}', bytes(range(40))]
    for index in range(240):
        timeout = rng.choice([1, 2, 7, 100, 1_000, U64_MAX])
        now = rng.choice([0, 1_000, U64_MAX - 5_000])
        ops = [('C', now)]
        for _ in range(rng.randrange(5, 60)):
            choice = rng.random()
            step = rng.choice([0, 0, 1, timeout - 1 if timeout < 10**6 else 3, timeout if timeout < 10**6 else 4,
                               timeout + 1 if timeout < 10**6 else 5, rng.randrange(50)])
            now = min(U64_MAX, now + max(step, 0))
            if choice < .55:
                stream = b''
                for _ in range(rng.randrange(1, 4)):
                    packet = bytearray(frame(rng.choice(payloads)))
                    if rng.random() < .3:
                        packet[rng.randrange(len(packet))] ^= 1 << rng.randrange(8)
                    if rng.random() < .15:
                        packet = bytearray(hidden()) + packet
                    stream += bytes(packet)
                stream = rng.randbytes(rng.randrange(4)) + stream
                cut = rng.randrange(1, len(stream) + 1)
                size = CHUNK + 1 if rng.random() < .02 else rng.randrange(1, CHUNK + 1)
                for start in range(0, cut, size):
                    ops.append(('F', now, stream[start:start + size]))
            elif choice < .75:
                ops.append(('P', now))
            elif choice < .82:
                ops.append(('D', now))
            elif choice < .9:
                ops.append(('C', now))
            elif choice < .94:
                ops.append(('X', now))
            elif choice < .97 and now > 0:
                ops.append(('P', now - 1))  # 시계 역행
            else:
                ops.append(('Z', now, rng.choice([0, 1])))
        add(f'random_link_{index}', ops, timeout=timeout)
    return cases


def expected_lines(case):
    oracle = LinkOracle(case['timeout'])
    lines = [f"N;{('disconnected', 'connected', 'faulted').index(oracle.state)};"
             f"{('none', 'invalid_config', 'clock_regressed', 'input_budget').index(oracle.fault)}"]
    snapshots = [oracle.apply(op) for op in case['ops']]
    for index, expected in case['checks'].items():
        for key, value in expected.items():
            if snapshots[index][key] != value:
                raise AssertionError(f"{case['name']}[{index}].{key}: expected {value!r}, "
                                     f"oracle {snapshots[index][key]!r}")
    return lines + [snapshot_line(s) for s in snapshots], snapshots


def run_driver(binary, cases):
    commands, owners = [], []
    for case in cases:
        commands.append(f"N {case['timeout']}")
        owners.append(case['name'])
        for op in case['ops']:
            commands.append(op_command(op))
            owners.append(case['name'])
    stdin = '\n'.join(commands) + '\n'
    run = subprocess.run([str(binary)], input=stdin, capture_output=True, text=True, timeout=300)
    return stdin, run, owners


def evaluate(binary, link_cases, policy_cases):
    # 1) 링크 단위: C++ 출력 줄을 독립 oracle 줄과 완전 비교.
    expected, failures = [], []
    for case in link_cases:
        expected += expected_lines(case)[0]
    policy_link = [dict(name='policy_' + c['name'], timeout=c['link_timeout'], ops=link_ops(c), checks={})
                   for c in policy_cases]
    all_cases = link_cases + policy_link
    for case in policy_link:
        expected += expected_lines(case)[0]
    stdin, run, owners = run_driver(binary, all_cases)
    actual = run.stdout.splitlines()
    for i, line in enumerate(expected):
        if i >= len(actual) or actual[i] != line:
            failures.append(dict(index=i, case=owners[i], expected=line[:400],
                                 actual=actual[i][:400] if i < len(actual) else None))
    if len(actual) > len(expected):
        failures.append(dict(error='unexpected extra output', lines=len(actual) - len(expected)))
    # 2) C++ 링크 출력을 그대로 참조 정책/기존 수신기에 넣어 수동 기대 상태를 재현하는지 본다.
    policy_failures, cursor = [], sum(1 + len(c['ops']) for c in link_cases)
    for case, link_case in zip(policy_cases, policy_link):
        cursor += 1  # N 줄
        lines = actual[cursor:cursor + len(link_case['ops'])]
        cursor += len(link_case['ops'])
        try:
            observed = replay_policy(case, [parse_line(line) for line in lines])
            error = compare_policy_case(case, observed)
        except (ValueError, IndexError, AssertionError) as exc:
            error = f'{type(exc).__name__}: {exc}'
        if error:
            policy_failures.append(dict(name=case['name'], error=error))
    report = dict(
        link_cases=len(link_cases), policy_cases=len(policy_cases),
        driver_lines=len(expected), link_operations=sum(len(c['ops']) for c in all_cases),
        manual_link_checks=sum(len(c['checks']) for c in link_cases),
        policy_events=sum(len(c['events']) for c in policy_cases),
        policy_checkpoints=sum(len(c['checkpoints']) for c in policy_cases),
        policy_link_checkpoints=sum(len(c['link_checkpoints']) for c in policy_cases),
        binary_exit_code=run.returncode, link_mismatches=failures, policy_replay_failures=policy_failures,
        passed=run.returncode == 0 and not failures and not policy_failures,
        test_timeout_values_only=True, production_timeout_selected=False,
        ros_or_runtime_connected=False, json_validation_ported=False, contract_state_machine_ported=False,
        wcet_measured=False, firmware_verified=False, ready_to_drive=False)
    return report, stdin, run.stdout, run.stderr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: digest(ROOT/p) for p in FILES}
    link_cases, policy_cases = make_link_cases(), make_policy_cases()
    report, stdin, stdout, stderr = evaluate(args.binary.resolve(), link_cases, policy_cases)
    report.update(source_sha256=hashes, binary_sha256=digest(args.binary),
                  source_unchanged=all(digest(ROOT/p) == v for p, v in hashes.items()))
    for name, value in [('stdin.txt', stdin), ('stdout.txt', stdout), ('stderr.txt', stderr)]:
        (args.output/name).write_text(value, encoding='utf-8', newline='\n')
    with (args.output/'policy_vectors.json').open('x', encoding='utf-8') as stream:
        json.dump(policy_cases, stream, indent=1, allow_nan=False)
    with (args.output/'report.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.write(ROOT/path, path)
    print(json.dumps({k: v for k, v in report.items()
                      if k not in ('source_sha256', 'link_mismatches', 'policy_replay_failures')}
                     | dict(link_mismatch_count=len(report['link_mismatches']),
                            policy_failure_count=len(report['policy_replay_failures']))))
    return 0 if report['passed'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
