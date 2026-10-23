#!/usr/bin/env python3
"""고정 메모리 C++ 프레임 코어와 독립 CRC/기존 Python 경계의 PC 대조."""
import argparse
import json
from pathlib import Path
import random
import struct
import subprocess
import zipfile
import zlib

from actuation_conformance import ROOT, digest
from actuation_transport_conformance import make_cases as transport_cases
from arena_vehicle_interface.actuation_wire import Decoder, MAX_PAYLOAD, MAGIC

CHUNK = 256
FILES = [
    'scripts/validate_actuation_wire_cpp.py', 'tests/test_actuation_wire_cpp.py',
    'scripts/actuation_wire_cpp_driver.cpp',
    'src/arena_vehicle_interface/portable/actuation_wire.hpp',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py',
    'scripts/actuation_transport_conformance.py', 'scripts/actuation_conformance.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py',
]


def frame(payload):
    body = struct.pack('<H', len(payload)) + payload
    return MAGIC + body + struct.pack('<I', zlib.crc32(body))


class FramingOracle:
    """JSON 해석 이전 바이트 경계만 계산한다. C++의 직접 상태를 읽지 않는다."""
    def __init__(self):
        self.pending = b''
        self.errors = 0

    def feed(self, data):
        if len(data) > CHUNK:
            self.pending = b''
            self.errors += 1
            return False, []
        self.pending += data
        payloads = []
        while self.pending:
            start = self.pending.find(MAGIC)
            if start == -1:
                self.pending = b'I' if self.pending.endswith(b'I') else b''
                break
            if start:
                self.pending = self.pending[start:]
                self.errors += 1
            if len(self.pending) < 4:
                break
            size, = struct.unpack('<H', self.pending[2:4])
            if not 1 <= size <= MAX_PAYLOAD:
                self.pending = self.pending[2:]
                self.errors += 1
                continue
            total = size + 8
            if len(self.pending) < total:
                break
            claimed, = struct.unpack('<I', self.pending[4+size:total])
            if zlib.crc32(self.pending[2:4+size]) != claimed:
                self.pending = self.pending[2:]
                self.errors += 1
                continue
            payloads.append(self.pending[4:4+size])
            self.pending = self.pending[total:]
        return True, payloads


def make_cases():
    rng = random.Random(0x20261021)
    cases = []
    stop = b'{"v":1,"kind":"STOP"}'

    def add(name, ops):
        cases.append(dict(name=name, operations=[('R', b'')] + ops))

    def chunks(data, size):
        return [('F', data[i:i+size]) for i in range(0, len(data), size)]

    add('known_crc', [('C', b''), ('C', b'123456789')])
    add('encode_size_edges', [('E', rng.randbytes(n)) for n in (0, 1, 2, 255, 256, 1536, 1537)])
    add('crc_valid_not_json', chunks(frame(b'\xff\x00not-json') + frame(stop), 7))
    add('partial_magic', [('F', b'noiseI'), ('F', frame(stop)[1:])])
    add('illegal_length_recovery', chunks(MAGIC+b'\x00\x00'+MAGIC+b'\xff\xff'+frame(stop), 3))
    add('legal_partial_requires_reset', [('F', MAGIC+struct.pack('<H', MAX_PAYLOAD)+frame(stop)),
                                        ('F', b''), ('R', b''), ('F', frame(stop))])
    add('oversized_chunk_is_rejected', [('F', b'I'), ('F', b'0'*(CHUNK+1)), ('F', frame(stop))])
    add('maximum_payload', chunks(frame(rng.randbytes(MAX_PAYLOAD))+frame(stop), CHUNK))
    for split in range(1, len(frame(stop))):
        add(f'stop_split_{split}', [('F', frame(stop)[:split]), ('F', frame(stop)[split:])])
    for index in range(len(frame(stop))):
        damaged = bytearray(frame(stop)); damaged[index] ^= 1
        add(f'bit_flip_{index}', chunks(bytes(damaged)+frame(stop), 7))
    for size in (1, 2, 3, 7, 64, 256):
        add(f'coalesced_chunk_{size}', chunks(frame(stop)*9, size))
    for index in range(160):
        stream = rng.randbytes(rng.randrange(201))
        for seq in range(rng.randrange(1, 5)):
            payload = json.dumps(dict(v=1, kind='SAMPLE', seq=seq, text='IArandom'), separators=(',', ':')).encode()
            packet = bytearray(frame(payload))
            if rng.random() < .35:
                packet[rng.randrange(len(packet))] ^= 1 << rng.randrange(8)
            stream += bytes(packet)
        add(f'random_stream_{index}', chunks(stream, rng.randrange(1, CHUNK+1)))
    # 12개 상태 계약 벡터의 바이트/재연결 순서를 그대로 가져온다. 상태기계를 C++로 포팅한 것은 아니다.
    for case in transport_cases():
        ops = []
        for row in case['events']:
            event = row['input']
            if event['op'] == 'bytes':
                ops += chunks(bytes.fromhex(event['args']['hex']), CHUNK)
            elif event['op'] in ('decoder_reset', 'reboot'):
                ops.append(('R', b''))
        add('contract_'+case['name'], ops)
    return cases


def expected_line(oracle, operation):
    op, data = operation
    if op == 'C':
        return f'C;{zlib.crc32(data)}'
    if op == 'E':
        return 'E;' + (frame(data).hex() if 1 <= len(data) <= MAX_PAYLOAD else 'REJECT')
    if op == 'R':
        oracle.pending = b''
        accepted, payloads = True, []
    else:
        accepted, payloads = oracle.feed(data)
    return f'D;{oracle.errors};{int(accepted)};{oracle.pending.hex()};' + ','.join(p.hex() for p in payloads)


def json_boundary_parity(cases):
    """C++는 raw payload만 내므로 JSON 검증은 기존 Python으로 별도 대조한다."""
    comparisons, mismatches = 0, []
    for case in cases:
        oracle, decoder = FramingOracle(), Decoder()
        for op, data in case['operations']:
            if op == 'R':
                oracle.pending = b''; decoder.reset()
            elif op == 'F':
                accepted, raw = oracle.feed(data)
                if not accepted:
                    # C++ 호출량 제한은 원래 Python decoder의 64 KiB 제한과 다른 명시 계약이다.
                    decoder.reset()
                    continue
                expected = decoder.feed(data)
                filtered = [message for payload in raw for message in Decoder().feed(frame(payload))]
                comparisons += 1
                if expected != filtered or bytes(decoder.buffer) != oracle.pending:
                    mismatches.append(case['name'])
    return comparisons, mismatches


def evaluate(binary, cases):
    oracle = FramingOracle()
    commands, expected, owners = [], [], []
    for case in cases:
        for op, data in case['operations']:
            commands.append(op + (' '+data.hex() if data else ''))
            expected.append(expected_line(oracle, (op, data)))
            owners.append(case['name'])
    stdin = '\n'.join(commands)+'\n'
    run = subprocess.run([str(binary)], input=stdin, capture_output=True, text=True, timeout=120)
    actual = run.stdout.splitlines()
    failures = [dict(index=i, case=owners[i], expected=left, actual=actual[i] if i<len(actual) else None)
                for i, left in enumerate(expected) if i>=len(actual) or actual[i] != left]
    if len(actual)>len(expected):
        failures.append(dict(error='unexpected extra output', lines=len(actual)-len(expected)))
    comparisons, json_failures = json_boundary_parity(cases)
    report = dict(cases=len(cases), operations=len(expected), binary_exit_code=run.returncode,
                  framing_mismatches=failures, json_boundary_comparisons=comparisons,
                  json_boundary_mismatches=json_failures,
                  passed=run.returncode==0 and not failures and not json_failures,
                  max_input_per_call=CHUNK, max_payload=MAX_PAYLOAD,
                  json_validation_ported=False, contract_state_machine_ported=False,
                  partial_frame_timeout_implemented=False, bounded_parser_time_measured=False,
                  firmware_verified=False, ready_to_drive=False, runtime_connected=False)
    return report, stdin, run.stdout, run.stderr


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: digest(ROOT/p) for p in FILES}
    report, stdin, stdout, stderr = evaluate(args.binary.resolve(), make_cases())
    report.update(source_sha256=hashes, binary_sha256=digest(args.binary),
                  source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()))
    for name, value in [('stdin.txt', stdin), ('stdout.txt', stdout), ('stderr.txt', stderr)]:
        (args.output/name).write_text(value, encoding='utf-8')
    with (args.output/'report.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.write(ROOT/path, path)
    print(json.dumps({k:v for k,v in report.items() if k not in ('source_sha256', 'framing_mismatches')}))
    return 0 if report['passed'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
