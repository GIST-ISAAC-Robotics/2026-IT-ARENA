#!/usr/bin/env python3
"""정규화 왕복 시각 기록의 오프라인 구간 감사. wire protocol/시계 설정과 미연결."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

from arena_vehicle_interface import clock_bounds
from arena_vehicle_interface.clock_bounds import ClockAssumptions, ClockExchange, fit_clock

SCHEMA = 'arena_clock_exchange_audit_v1'
MAX_BYTES = 262_144


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON key: ' + key)
        result[key] = value
    return result


def invalid_constant(value):
    raise ValueError('nonfinite JSON constant: ' + value)


def exact_keys(row, keys):
    if not isinstance(row, dict) or set(row) != set(keys):
        raise ValueError('record fields do not match schema')


def audit_bytes(data):
    if len(data) > MAX_BYTES:
        raise ValueError('capture exceeds byte budget')
    lines = data.decode('utf-8').splitlines()
    if not 3 <= len(lines) <= 129 or any(not line.strip() for line in lines):
        raise ValueError('header, 1..64 exchanges and 1..64 queries required')
    rows = [json.loads(line, object_pairs_hook=unique_object, parse_constant=invalid_constant)
            for line in lines]
    header = rows[0]
    exact_keys(header, ('kind', 'schema', 'origin', 'host_clock', 'device_clock', 'assumptions'))
    if (header['kind'] != 'header' or header['schema'] != SCHEMA
            or header['origin'] not in ('synthetic', 'hardware_capture')
            or header['host_clock'] != 'host_monotonic_ns'
            or header['device_clock'] != 'mcu_monotonic_us'):
        raise ValueError('explicit schema, origin and separate clock units required')
    if not isinstance(header['assumptions'], dict):
        raise ValueError('explicit assumptions required')
    try:
        assumptions = ClockAssumptions(**header['assumptions'])
    except TypeError as error:
        raise ValueError('explicit assumptions do not match schema') from error
    exchanges, queries = [], []
    for row in rows[1:]:
        if not isinstance(row, dict):
            raise ValueError('record must be an object')
        if row.get('kind') == 'exchange' and not queries:
            exact_keys(row, ('kind', 'boot', 'sequence', 'host_send_ns', 'mcu_receive_us',
                             'mcu_send_us', 'host_receive_ns'))
            exchanges.append(ClockExchange(**{k: v for k, v in row.items() if k != 'kind'}))
        elif row.get('kind') == 'query':
            exact_keys(row, ('kind', 'boot', 'capture_us'))
            queries.append(row)
        else:
            raise ValueError('exchanges must precede queries')
    if not 1 <= len(queries) <= 64:
        raise ValueError('one through 64 queries required')
    fit = fit_clock(exchanges, assumptions)
    mapped = [asdict(fit.map_interval(q['capture_us'], boot=q['boot'])) for q in queries]
    return dict(schema=SCHEMA, origin=header['origin'], evidence_consistent=True,
                assumptions=asdict(assumptions), assumptions_hardware_verified=False,
                mapping_model='constant_affine_within_declared_window',
                boot=fit.boot, exchange_count=fit.exchanges, sequence_gaps=fit.sequence_gaps,
                recorded_sequence_complete=fit.sequence_gaps == 0,
                complete_capture_verified=False, query_count=len(mapped), mapped=mapped,
                maximum_interval_width_ns=max(q['width_ns'] for q in mapped),
                rate_ratio_interval=[str(min(v[0] for v in fit.vertices)),
                                     str(max(v[0] for v in fit.vertices))],
                offset_anchor_host_ns=fit.host_anchor_ns, offset_anchor_mcu_us=fit.mcu_anchor_us,
                ready_to_drive=False, ros_stamp_verified=False, clock_mapping_applied=False)


def file_digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists() or args.capture.resolve() == args.output.resolve():
        parser.error('output must be new and must not replace input')
    with args.capture.open('rb') as stream:
        data = stream.read(MAX_BYTES + 1)
    source_digest = hashlib.sha256(data).hexdigest()
    try:
        result = audit_bytes(data)
    except (ValueError, UnicodeError) as error:
        result = dict(schema=SCHEMA, evidence_consistent=False, error=str(error),
                      ready_to_drive=False, ros_stamp_verified=False, clock_mapping_applied=False)
    result['capture_sha256'] = source_digest
    result['input_unchanged'] = file_digest(args.capture) == source_digest
    if not result['input_unchanged']:
        result['evidence_consistent'] = False
    result['source_sha256'] = {str(Path(p).name): file_digest(p)
                              for p in (__file__, clock_bounds.__file__)}
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('mapped', 'source_sha256')},
                     ensure_ascii=False))
    return 0 if result['evidence_consistent'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
