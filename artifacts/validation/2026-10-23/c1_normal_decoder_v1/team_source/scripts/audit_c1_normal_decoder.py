"""제조사 normal 응답 decoder의 PC 한정 감사. C1 실물 형식 확정/드라이버가 아니다.

고정 원문의 handler/타입을 컴파일하고 clock·transport 대신 명시한 합성 입력을 사용한다.
장치/ROS/네트워크를 열지 않고 취득 시각을 복원하거나 운영 승인을 하지 않는다.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import struct
import subprocess

ROOT = Path(__file__).resolve().parents[1]
COMMIT = '34300099fadfc772965962dec837bf436706188f'
SOURCE = ROOT/'artifacts/validation/2026-10-23/c1_normal_decoder_sources_v1'
PINNED = {
    'sdk/src/dataunpacker/unpacker/handler_normalnode.cpp': 'e6f5f0d8200ef10ca50cb8c002c145f1228f64877c6e1f68824ff1e788943e48',
    'sdk/src/dataunpacker/unpacker/handler_normalnode.h': '8edc84bd807bd76a1a98423ed7f66ffb59df5aace77fcc8e58fac13054998dce',
    'sdk/src/dataunpacker/dataunpacker.h': '9b7112ef532c326d2140f42f944f88cd10ae3d08115317706609c87e60052023',
    'sdk/src/dataunpacker/dataunnpacker_internal.h': 'edb58ceb98567b34b3a845a5a570ccc130ac2cbc30f984749846f657bdd7bd29',
    'sdk/src/dataunpacker/dataupacker_namespace.h': '641f5bf540099ae14a918bf1d86c9127dd7e5dc8c88af2d4bc54572a8c94ae79',
    'sdk/include/sl_lidar_cmd.h': '985657d0bc532bd6f0948dd5c586b170616bba3841d73f2a89d0aaa0bc9c2eb2',
    'sdk/include/sl_lidar_protocol.h': 'ccde66d13b7ba271667cd10b7f040d9fea353d8720a5da525083d4dda627c526',
    'sdk/include/sl_types.h': '40a3a48573b13f6f88e2f3cc3b2e3b0b1835406b006be7735be97244be81fb5b',
    'sdk/include/rplidar_cmd.h': '4ef9ef03d6ee182acc74eba043880fa43ff098b81e66bf612b989f079c2b4836',
    'sdk/include/rplidar_protocol.h': '822ae2a80c3007b813213c5db797865308096a0c3d83a5732ad81ee18443f725',
    'sdk/include/rptypes.h': 'b34bbc79208c333601129b0c52387a872293a6338045a64f22a8e895733ef042',
    'sdk/include/sl_lidar_driver.h': '8610bf438629f756b124a1f42a2e153537de41db60a4971288b061df030a9973',
    'LICENSE': '3475642aaf47837729092f181f151866abb6e63b624cf504a9229d7641d2eaf7',
}
COMMON_STUB = '''// TEST dependency shim; not the upstream commondef.h. No I/O or threading.
#pragma once
#include <cassert>
#include <cstring>
#include <vector>
#include "rptypes.h"
#include "rplidar_cmd.h"
#include "sl_lidar_driver.h"
#include "dataupacker_namespace.h"
'''


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build(source, output):
    # Validate every input before creating a build tree. No implicit download/fallback.
    for name, expected in PINNED.items():
        if digest(source/name) != expected:
            raise ValueError('pinned source hash mismatch: '+name)
    output.mkdir(parents=True, exist_ok=False)
    for name in PINNED:
        target = output/name
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source/name, target)
    (output/'sdk/src/dataunpacker/dataunnpacker_commondef.h').write_text(COMMON_STUB)
    shutil.copyfile(ROOT/'tests/fixtures/c1_normal_decoder_probe.cpp', output/'probe.cpp')
    command = ['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
               '-fsanitize=undefined', '-fno-sanitize-recover=all', '-I'+str(output/'sdk/include'),
               str(output/'probe.cpp'), str(output/'sdk/src/dataunpacker/unpacker/handler_normalnode.cpp'),
               '-o', str(output/'probe')]
    result = subprocess.run(command, text=True, capture_output=True, timeout=60)
    (output/'compile.json').write_text(json.dumps(dict(command=command, exit=result.returncode), indent=2)+'\n')
    (output/'compile.stdout').write_text(result.stdout)
    (output/'compile.stderr').write_text(result.stderr)
    if result.returncode:
        raise RuntimeError('native compile failed; evidence preserved')
    return output/'probe'


def bounded_int(value, low, high):
    if type(value) is not int or not low <= value <= high:
        raise ValueError('bounded integer required')
    return value


def node(angle_q6, distance_q2, quality=40, sync=False):
    """독립 합성 입력 encoder. angle_q6는 도 단위가 아닌 wire 정수다."""
    bounded_int(angle_q6, 0, 32767)
    bounded_int(distance_q2, 0, 65535)
    bounded_int(quality, 0, 63)
    if type(sync) is not bool:
        raise ValueError('explicit sync bool required')
    return struct.pack('<BHH', (quality << 2) | (1 if sync else 2),
                       angle_q6*2+1, distance_q2)


def chunk(data, clock_us=1_000_000, step_us=0):
    return dict(data=data.hex(), clock_us=clock_us, step_us=step_us)


def fixture_text(actions, sample_us=200, baud=460800, linkage_us=0, interface=0):
    for v in (sample_us, baud, linkage_us):
        bounded_int(v, 0, 2**32-1)
    bounded_int(interface, 0, 65535)
    bounded_int(len(actions), 0, 10000)
    lines = [f'{sample_us} {baud} {linkage_us} {interface} {len(actions)}']
    total = 0
    for row in actions:
        if row == 'reset':
            lines.append('R')
            continue
        if type(row) is not dict or set(row) != {'data', 'clock_us', 'step_us'}:
            raise ValueError('exact chunk fields required')
        data = row['data']
        if (type(data) is not str or not 2 <= len(data) <= 131072 or len(data) % 2
                or any(c not in '0123456789abcdef' for c in data)):
            raise ValueError('bounded lowercase hex chunk required')
        total += len(data)//2
        bounded_int(total, 0, 131072)
        now = bounded_int(row['clock_us'], 0, 2**64-1)
        step = bounded_int(row['step_us'], 0, 2**32-1)
        if now + step*(len(data)//2) > 2**64-1:
            raise ValueError('possible clock overflow')
        lines.append(f'D {now} {step} {data}')
    return '\n'.join(lines)+'\n'


def decode(executable, actions, **timing):
    request = fixture_text(actions, **timing)
    result = subprocess.run([str(executable)], input=request, text=True, capture_output=True, timeout=15)
    if result.returncode:
        raise RuntimeError('native decode failed: '+result.stderr)
    rows = [json.loads(line) for line in result.stdout.splitlines()]
    if not rows or rows[-1]['kind'] != 'end':
        raise ValueError('missing native footer')
    return rows


def geometry(rows):
    return [tuple(r[k] for k in ('angle_q14', 'distance_q2', 'quality', 'flag'))
            for r in rows if r['kind'] == 'point']


def summary(rows):
    points = rows[:-1]
    stamps = [p['sdk_estimate_us'] for p in points]
    diffs = [b-a for a, b in zip(stamps, stamps[1:])]
    return dict(**rows[-1], equal_stamps=sum(d == 0 for d in diffs),
                backwards_stamps=sum(d < 0 for d in diffs),
                stamp_span_us=stamps[-1]-stamps[0] if stamps else None,
                invalid_future_stamps=sum(p['sdk_estimate_us'] > p['decode_clock_us'] for p in points))


def audit(source, output):
    output.mkdir(parents=True, exist_ok=False)
    (output/'.gitignore').write_text('native/probe\n')
    executable = build(source, output/'native')
    raw = b''.join(node(i*100, 4000+i, sync=i == 0) for i in range(50))
    cases = {
        'spaced_200us': [chunk(raw[i:i+5], 1_000_000+200*(i//5)) for i in range(0, len(raw), 5)],
        'batch_same_us': [chunk(raw, 1_009_800)],
        'batch_step_1us': [chunk(raw, 1_009_800, 1)],
        'byte_chunks': [chunk(bytes([b]), 1_009_800) for b in raw],
        'two_batches_20ms_gap': [chunk(raw[:125], 1_000_000), chunk(raw[125:], 1_020_000)],
        'truncated': [chunk(raw[:-2])],
        'zero_range_quality': [chunk(node(100, 0, quality=0))],
        'distance_bit_flip': [chunk(raw[:3]+bytes([raw[3]^0x80])+raw[4:])],
        'bad_angle_check_bit': [chunk(bytes.fromhex('a200000000')+raw)],
        'bad_sync_pair': [chunk(bytes.fromhex('00000303')+raw)],
        'reset_partial': [chunk(raw[:2]), 'reset', chunk(raw)],
        'timestamp_underflow': [chunk(node(100, 4000), 100)],
        'angle_out_of_physical_range': [chunk(node(32767, 4000))],
    }
    results, records = {}, {}
    for name, actions in cases.items():
        rows = decode(executable, actions)
        (output/(name+'.input.json')).write_text(json.dumps(actions, indent=2)+'\n')
        (output/(name+'.output.json')).write_text(json.dumps(rows, indent=2)+'\n')
        results[name], records[name] = summary(rows), rows
    baseline = geometry(records['spaced_200us'])
    checks = {
        'batching_keeps_geometry_order': all(geometry(records[n]) == baseline for n in
            ('batch_same_us', 'batch_step_1us', 'byte_chunks', 'two_batches_20ms_gap')),
        'spaced_span_9800us': results['spaced_200us']['stamp_span_us'] == 9800,
        'batch_duplicates_49': results['batch_same_us']['equal_stamps'] == 49,
        'batch_step_span_49us': results['batch_step_1us']['stamp_span_us'] == 49,
        'partial_not_published': results['truncated']['points'] == 49 and results['truncated']['pending_bytes'] == 3,
        'zero_return_retained_not_free': geometry(records['zero_range_quality'])[0][1:3] == (0, 0),
        'distance_bit_flip_undetected': results['distance_bit_flip']['points'] == 50 and geometry(records['distance_bit_flip']) != baseline,
        'bad_sync_recovers_fixture': geometry(records['bad_sync_pair']) == baseline,
        'bad_check_recovers_fixture': geometry(records['bad_angle_check_bit']) == baseline,
        'reset_discards_partial': geometry(records['reset_partial']) == baseline,
        'underflow_is_not_acquisition_time': results['timestamp_underflow']['invalid_future_stamps'] == 1,
        'wire_angle_overflow_not_rejected': results['angle_out_of_physical_range']['points'] == 1,
    }
    report = dict(commit=COMMIT, hardware_verified=False, ready_to_drive=False,
                  timing_verified=False, scope='normal response handler 0x81 only; synthetic clock',
                  source_sha256=PINNED, checks=checks, results=results)
    (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    for rel in ('scripts/audit_c1_normal_decoder.py', 'tests/fixtures/c1_normal_decoder_probe.cpp'):
        target = output/'team_source'/rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(ROOT/rel, target)
    if not all(checks.values()):
        raise AssertionError('native audit check failed; evidence preserved')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit(args.source.resolve(), args.output.resolve())
    print(json.dumps(dict(checks=result['checks'], ready_to_drive=False), indent=2))
