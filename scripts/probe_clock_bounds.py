#!/usr/bin/env python3
"""명시한 합성 시계의 구간 포함/거절 시험. 실제 시계 또는 통신 성능 측정 아님."""
import argparse
import copy
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

from audit_clock_exchanges import SCHEMA

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ('src/arena_vehicle_interface/arena_vehicle_interface/clock_bounds.py',
           'scripts/audit_clock_exchanges.py', 'scripts/probe_clock_bounds.py',
           'tests/test_clock_bounds.py', 'tests/test_clock_exchange_audit.py')


def cases():
    header = dict(kind='header', schema=SCHEMA, origin='synthetic',
                  host_clock='host_monotonic_ns', device_clock='mcu_monotonic_us',
                  assumptions=dict(max_rate_error_ppm=200, host_timestamp_error_ns=0,
                                   mcu_timestamp_error_us=0, max_window_us=5_000_000,
                                   max_extrapolation_us=100_000))
    for name in ('asymmetric', 'alternating_delays', 'rate_plus150ppm', 'rate_minus150ppm',
                 'sequence_gap', 'timestamp_uncertainty', 'reboot', 'rewind',
                 'clock_jump', 'extrapolation', 'exact_large_timestamp'):
        current = copy.deepcopy(header)
        ppm = 150 if name == 'rate_plus150ppm' else -150 if name == 'rate_minus150ppm' else 0
        rate = Fraction(1_000_000 + ppm, 1_000_000)
        offset = 2**60 + 123 if name == 'exact_large_timestamp' else 8_000_000_000
        count = 1 if name in ('asymmetric', 'exact_large_timestamp') else 5
        exchanges = []
        for index in range(count):
            stamp = 1_000_000 + 500_000 * index
            forward, reverse = (1_000_000, 9_000_000)
            if name == 'alternating_delays' and index % 2:
                forward, reverse = reverse, forward
            if name == 'clock_jump': forward = reverse = 0
            if name == 'exact_large_timestamp': forward, reverse = 7, 11
            send = rate * stamp * 1000 + offset - forward
            receive = rate * (stamp + 100) * 1000 + offset + reverse
            assert send.denominator == receive.denominator == 1
            exchanges.append(dict(kind='exchange', boot='boot-A', sequence=index,
                                  host_send_ns=int(send), mcu_receive_us=stamp,
                                  mcu_send_us=stamp+100, host_receive_ns=int(receive)))
        if name == 'sequence_gap':
            for row in exchanges[2:]: row['sequence'] += 2
        if name == 'timestamp_uncertainty':
            current['assumptions'].update(host_timestamp_error_ns=1000, mcu_timestamp_error_us=1)
            for row in exchanges:
                row['host_send_ns'] += 1000
                row['host_receive_ns'] -= 1000
                row['mcu_receive_us'] -= 1
                row['mcu_send_us'] += 1
        if name == 'reboot': exchanges[-1]['boot'] = 'boot-B'
        if name == 'rewind': exchanges[-1].update(mcu_receive_us=0, mcu_send_us=100)
        if name == 'clock_jump':
            exchanges[-1]['host_send_ns'] += 10_000_000
            exchanges[-1]['host_receive_ns'] += 10_000_000
        if name == 'exact_large_timestamp': current['assumptions']['max_rate_error_ppm'] = 0
        query = exchanges[0]['mcu_receive_us'] + 50
        if name == 'extrapolation': query = 5_000_000
        expected = name not in ('reboot', 'rewind', 'clock_jump', 'extrapolation')
        truth = rate * query * 1000 + offset
        yield name, [current, *exchanges, dict(kind='query', boot='boot-A', capture_us=query)], expected, truth


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in SOURCES: archive.write(ROOT/path, path)
    results = []
    for name, rows, expected, truth in cases():
        capture, report = args.output/(name+'.jsonl'), args.output/(name+'.report.json')
        with capture.open('x', encoding='utf-8') as stream:
            for row in rows: stream.write(json.dumps(row) + '\n')
        completed = subprocess.run([sys.executable, str(ROOT/'scripts/audit_clock_exchanges.py'),
                                    str(capture), '--output', str(report)], capture_output=True, text=True)
        with (args.output/(name+'.log')).open('x', encoding='utf-8') as stream:
            stream.write(completed.stdout + completed.stderr)
        payload = json.loads(report.read_text()) if report.exists() else {}
        consistent = payload.get('evidence_consistent')
        mapped = payload.get('mapped', [])
        contains_truth = bool(mapped and mapped[0]['host_lower_ns'] <= truth <= mapped[0]['host_upper_ns'])
        passed = (completed.returncode == (0 if expected else 2) and consistent is expected
                  and payload.get('input_unchanged') is True and (not expected or contains_truth)
                  and all(payload.get(flag) is False for flag in
                          ('ready_to_drive', 'ros_stamp_verified', 'clock_mapping_applied')))
        results.append(dict(case=name, expected_consistent=expected, evidence_consistent=consistent,
                            passed=passed, truth_contained=contains_truth if expected else None,
                            interval_width_ns=payload.get('maximum_interval_width_ns'),
                            sequence_gaps=payload.get('sequence_gaps'), error=payload.get('error')))
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == sha for p, sha in hashes.items())
    summary = dict(origin='synthetic', passed=unchanged and all(r['passed'] for r in results),
                   source_unchanged=unchanged, source_sha256=hashes, results=results,
                   ready_to_drive=False, ros_stamp_verified=False, clock_mapping_applied=False)
    with (args.output/'report.json').open('x', encoding='utf-8') as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print(json.dumps(summary, ensure_ascii=False))
    return 0 if summary['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
