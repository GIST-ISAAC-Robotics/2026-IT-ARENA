#!/usr/bin/env python3
"""왕복 시각 원문과 별도 검토 조건을 결합한다. 실제 센서/제어기에는 미연결."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import zipfile

from audit_clock_exchanges import audit_bytes, exact_keys, invalid_constant, unique_object, MAX_BYTES
from arena_vehicle_interface.clock_bounds import MappedInterval
from arena_vehicle_interface.clock_freshness import HostTimeWindow, assess_freshness

ROOT = Path(__file__).resolve().parents[1]
SCHEMA = 'arena_clock_freshness_review_v1'
SOURCES = ('scripts/audit_clock_freshness.py', 'scripts/audit_clock_exchanges.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/clock_bounds.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/clock_freshness.py')


def review_bytes(exchange_data, review_data):
    if len(review_data) > MAX_BYTES:
        raise ValueError('review exceeds byte budget')
    specification = json.loads(review_data, object_pairs_hook=unique_object, parse_constant=invalid_constant)
    exact_keys(specification, ('schema', 'origin', 'exchange_sha256', 'host_epoch', 'clock', 'queries'))
    if (specification['schema'] != SCHEMA or specification['origin'] not in ('synthetic', 'hardware_capture')
            or specification['clock'] != 'host_monotonic_ns'):
        raise ValueError('explicit schema, origin and host monotonic clock required')
    if specification['exchange_sha256'] != hashlib.sha256(exchange_data).hexdigest():
        raise ValueError('review is bound to a different exchange capture')
    original = audit_bytes(exchange_data)
    if specification['origin'] != original['origin']:
        raise ValueError('cannot relabel synthetic input as hardware capture')
    queries = specification['queries']
    if not isinstance(queries, list) or not 1 <= len(queries) <= 64:
        raise ValueError('one through 64 freshness queries required')
    results, seen = [], set()
    for query in queries:
        exact_keys(query, ('mapped_query_index', 'boot', 'host_epoch', 'now_lower_ns', 'now_upper_ns',
                           'max_age_ns', 'limit_inclusive', 'policy_note'))
        index = query['mapped_query_index']
        if type(index) is not int or not 0 <= index < len(original['mapped']) or index in seen:
            raise ValueError('unique existing mapped query index required')
        seen.add(index)
        mapped = MappedInterval(**original['mapped'][index])
        if query['boot'] != mapped.boot:
            raise ValueError('query MCU boot mismatch')
        if not isinstance(query['policy_note'], str) or not query['policy_note'].strip():
            raise ValueError('age limit source/purpose must be documented')
        now = HostTimeWindow(query['host_epoch'], query['now_lower_ns'], query['now_upper_ns'])
        result = assess_freshness(mapped, now, mapping_host_epoch=specification['host_epoch'],
                                  max_age_ns=query['max_age_ns'], inclusive=query['limit_inclusive'])
        results.append(dict(mapped_query_index=index, mapped=asdict(mapped), now=asdict(now),
                            policy_note=query['policy_note'], **result))
    return dict(schema=SCHEMA, review_valid=True, origin=specification['origin'],
                exchange_sha256=specification['exchange_sha256'],
                queries=results, all_timestamp_policies_satisfied=all(q['timestamp_policy_satisfied'] for q in results),
                host_epoch_is_user_assertion=True, ready_to_drive=False, ros_stamp_verified=False,
                clock_mapping_applied=False, limits=[
                    'original affine/rate/timestamp bounds remain conditional, not hardware verified',
                    'host epoch cannot be inferred or physically verified from timestamps alone',
                    'no midpoint stamp, timeout change, ROS mapping, value quality or packet delivery proof'])


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('exchanges', type=Path)
    parser.add_argument('review', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.resolve() in (args.exchanges.resolve(), args.review.resolve()):
        parser.error('new output required')
    hashes = {p: digest(ROOT/p) for p in SOURCES}
    with zipfile.ZipFile(args.output.with_suffix('.source.zip'), 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in SOURCES:
            bundle.write(ROOT/path, path)
    inputs = {}
    for name, path in (('exchanges', args.exchanges), ('review', args.review)):
        with path.open('rb') as stream:
            inputs[name] = stream.read(MAX_BYTES+1)
    input_hashes = {k: hashlib.sha256(v).hexdigest() for k,v in inputs.items()}
    try:
        report = review_bytes(inputs['exchanges'], inputs['review'])
    except (ValueError, UnicodeError) as error:
        report = dict(schema=SCHEMA, review_valid=False, error=str(error), ready_to_drive=False,
                      ros_stamp_verified=False, clock_mapping_applied=False)
    unchanged = all(digest(path) == input_hashes[name] for name,path in (
        ('exchanges', args.exchanges), ('review', args.review)))
    source_unchanged = all(digest(ROOT/name) == sha for name,sha in hashes.items())
    report.update(input_sha256=input_hashes, input_unchanged=unchanged, source_sha256=hashes,
                  source_unchanged=source_unchanged)
    report['review_valid'] &= unchanged and source_unchanged
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2,allow_nan=False)
        stream.write('\n')
    print(json.dumps(report,ensure_ascii=False))
    # 잘 구성된 stale/uncertain 진단은 검사 실행 성공이며 입력 채택 성공이 아니다.
    return 0 if report['review_valid'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
