"""r406 원자료를 보존하며 JSON 문자열의 표기 차이와 실제 설정 차이를 구분한다."""
import argparse
import hashlib
import json
import math
from pathlib import Path


FIELD = ('follow_config', 'known_structure_sectors_json')


def reject_constant(value):
    raise ValueError('non-finite JSON constant: ' + value)


def unique_pairs(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('duplicate JSON key: ' + key)
        result[key] = value
    return result


def strict_equal(left, right):
    if type(left) is not type(right):
        return False
    if isinstance(left, dict):
        return left.keys() == right.keys() and all(strict_equal(left[k], right[k]) for k in left)
    if isinstance(left, list):
        return len(left) == len(right) and all(strict_equal(a, b) for a, b in zip(left, right))
    return left == right


def parse_sectors(value):
    if type(value) is not str:
        raise ValueError('structure field is not a JSON string')
    data = json.loads(value, object_pairs_hook=unique_pairs, parse_constant=reject_constant)
    if type(data) is not dict or data.keys() != {'sectors_deg', 'range_limits_m'}:
        raise ValueError('unexpected structure schema')
    sectors, ranges = data['sectors_deg'], data['range_limits_m']
    if type(sectors) is not list or type(ranges) is not list or len(sectors) != len(ranges):
        raise ValueError('unpaired sectors/ranges')
    for pair in sectors + ranges:
        if (type(pair) is not list or len(pair) != 2 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in pair)):
            raise ValueError('invalid numeric pair')
    return data


def compare_headers(actual, expected):
    """이 필드만 decode. 다른 값/형식과 모든 배열 순서는 그대로 비교한다."""
    try:
        a = parse_sectors(actual[FIELD[0]][FIELD[1]])
        b = parse_sectors(expected[FIELD[0]][FIELD[1]])
    except (KeyError, TypeError, ValueError) as error:
        return dict(equivalent=False, exact=False, error=str(error))
    normalized_a = {**actual, FIELD[0]: {**actual[FIELD[0]], FIELD[1]: a}}
    normalized_b = {**expected, FIELD[0]: {**expected[FIELD[0]], FIELD[1]: b}}
    return dict(equivalent=strict_equal(normalized_a, normalized_b),
                exact=strict_equal(actual, expected), error=None,
                normalized_field='.'.join(FIELD), values_types_and_order_preserved=True)


def summarize(rows, expected, original):
    headers = [r['data'] for r in rows if r['kind'] == 'header']
    comparison = compare_headers(headers[0], expected) if len(headers) == 1 else {
        'equivalent': False, 'exact': False, 'error': 'header count is not one'}
    factory = [r['data'] for r in rows if r['kind'] == 'runtime_factory_policy']
    submission = [r['data'] for r in rows if r['kind'] == 'decision_submission_policy']
    footer = rows[-1]['data'] if rows and rows[-1]['kind'] == 'footer' else None
    config_valid = (comparison['equivalent'] and len(factory) == 1
        and factory[0].get('policy') == 'cad_runtime_r377_v1'
        and factory[0].get('current_corner_budget') == 'r396_v1'
        and factory[0].get('motion_time_policy') == 'verified_only'
        and factory[0].get('motion_timing_verified') is True
        and len(submission) == 1 and submission[0].get('policy') == 'runtime_dual_latest_v1')
    source_complete = footer is not None and footer.get('capture_errors') == 0
    publications = original['candidate_parent_publications']
    return dict(header_comparison=comparison, configuration_equivalent=config_valid,
        original_exact_verdict=original['header_exact'], original_audit_preserved=True,
        trace_complete=source_complete, parent_publications_observed=publications,
        parent_publication_evidence_valid=bool(config_valid and source_complete and publications > 0),
        guard_or_mcu_application_attributed=False, driving_improvement_demonstrated=False,
        original_driving_failure_unchanged=True,
        note='Original candidate_effect_demonstrated meant a parent publication only; it is not a driving result.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    names = ('run1/follow_decisions.jsonl', 'expected_header.json', 'runtime_audit.json', 'run1/report.json')
    raw = {name: (args.source / name).read_bytes() for name in names}
    rows = [json.loads(line) for line in raw[names[0]].splitlines()]
    result = summarize(rows, json.loads(raw[names[1]]), json.loads(raw[names[2]]))
    report = json.loads(raw[names[3]])
    result['original_run'] = {key: report[key] for key in ('passed', 'harness_valid', 'abort', 'gates')}
    result['source_sha256'] = {name: hashlib.sha256(data).hexdigest() for name, data in raw.items()}
    result['audit_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    assert all((args.source / name).read_bytes() == data for name, data in raw.items())
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({key: result[key] for key in ('header_comparison', 'parent_publication_evidence_valid',
        'driving_improvement_demonstrated')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
