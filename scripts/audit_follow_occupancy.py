#!/usr/bin/env python3
"""G1 기록의 취득 시각/점유 여유를 독립 계산한다. 출력/물리/정지 판정은 바꾸지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import zipfile

from arena_autonomy.follow_trace import thaw

ROOT = Path(__file__).resolve().parents[1]


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def audit_events(events, expected_mode):
    if expected_mode not in ('oldest', 'per_ray_candidate'):
        raise ValueError('explicit known timing mode required')
    iterator = iter(events)
    header = next(iterator, None)
    if (not header or header.get('seq') != 0 or header.get('kind') != 'header'
            or header['data'].get('schema') != 1 or header['data'].get('scope') != 'g1_decision_stage_only'):
        raise ValueError('trace header required')
    cfg = header['data']['follow_config']
    mode = cfg.get('occupancy_timing_mode', 'oldest')
    if mode != expected_mode:
        raise ValueError('recorded timing mode mismatch')
    error, velocity = cfg['measurement_padding_m'], cfg['object_motion_bound_mps']
    if not all(finite(v) and v >= 0 for v in (error, velocity)):
        raise ValueError('invalid padding assumptions')
    fixed = .025+error
    count, calls, evaluated, footer = 1, 0, 0, None
    reasons, skipped, errors, rows = Counter(), Counter(), [], []
    unique_sources = set()
    for event in iterator:
        if event.get('seq') != count or footer is not None:
            raise ValueError('trace sequence gap or trailing data')
        count += 1
        kind, data = event['kind'], thaw(event['data'])
        if kind == 'reset':
            continue
        if kind == 'footer':
            footer = data
            continue
        if kind != 'call':
            raise ValueError('unknown trace event')
        calls += 1
        inputs, output = data['input'], data['output']
        details = output[2]
        reason = details.get('reason', 'missing')
        reasons[reason] += 1
        if 'occupied_points' not in details:
            skipped[reason] += 1
            continue
        raw, now = inputs['scan'], inputs['reference_s']
        first = raw['sec']+raw['nanosec']*1e-9
        step, amin, increment = raw['time_increment'], raw['angle_min'], raw['angle_increment']
        half = inputs['rear_half_angle_deg']
        if (not all(finite(v) for v in (now, first, step, amin, increment, half, raw['range_min'], raw['range_max']))
                or first < 0 or step <= 0 or not 0 <= half < 180
                or not 2 <= len(raw['ranges']) <= 8192):
            raise ValueError('invalid captured scan metadata')
        last = first+(len(raw['ranges'])-1)*step
        if last > now or now-first >= .3:
            raise ValueError('future or stale scan cannot claim evaluated occupancy')
        # 생산 함수를 호출하지 않고 원래 광선 각도/거리/취득 순서에서 재계산한다.
        ages = []
        for index, distance in enumerate(raw['ranges']):
            angle = abs(math.remainder(amin+index*increment, math.tau))
            masked = angle > math.radians(180-half)+1e-12
            if not masked and finite(distance) and raw['range_min'] <= distance <= raw['range_max']:
                ages.append(now-(first+index*step))
        if not ages:
            raise ValueError('no occupied returns in evaluated call')
        expected = [fixed+velocity*(age if mode == 'per_ray_candidate' else now-first) for age in ages]
        checks = {'return_count': details['occupied_points'] == len(ages),
                  'all_returns': details.get('unfiltered_occupancy') is True,
                  'maximum_padding': finite(details.get('occupancy_padding_m')) and math.isclose(
                      details['occupancy_padding_m'], max(expected), abs_tol=1e-10, rel_tol=1e-10)}
        if mode == 'per_ray_candidate':
            checks.update(mode=details.get('occupancy_timing_mode') == mode,
                          padded_count=details.get('occupancy_padding_points') == len(ages))
            for label, expected_value in (('min', min(expected)), ('max', max(expected))):
                recorded = details.get(f'occupancy_padding_{label}_m')
                checks[f'{label}_padding'] = finite(recorded) and math.isclose(
                    recorded, expected_value, abs_tol=1e-10, rel_tol=1e-10)
        failed = [name for name, passed in checks.items() if not passed]
        if failed:
            errors.append(dict(seq=event['seq'], failed=failed))
        evaluated += 1
        unique_sources.add((inputs['epoch'], first))
        rows.append(dict(seq=event['seq'], reference_s=now, source_first_s=first,
                         first_ray_age_s=now-first, newest_return_age_s=min(ages), oldest_return_age_s=max(ages),
                         returns=len(ages), minimum_padding_m=min(expected), maximum_padding_m=max(expected),
                         reason=reason, all_checks_passed=not failed))
    if (footer is None or calls == 0 or any(footer.get(k) != 0 for k in
            ('capture_errors', 'failed', 'dropped', 'pending')) or footer.get('worker_alive') is not False
            or footer.get('written') != count-1 or footer.get('accepted') != count-1):
        raise ValueError('complete nonempty trace footer required')
    return dict(schema='g1_occupancy_timing_audit_v1', passed=bool(evaluated) and not errors,
                timing_mode=mode, calls=calls, evaluated_calls=evaluated, unique_evaluated_scans=len(unique_sources),
                skipped_calls=dict(skipped), reason_counts=dict(reasons), errors=errors, rows=rows,
                maximum_first_ray_age_s=max((r['first_ray_age_s'] for r in rows), default=None),
                fixed_padding_m=fixed, motion_bound_mps=velocity, driving_authorized=False,
                original_verdict_changed=False, output_replay_verified=False,
                limits=['checks captured count and padding extrema, not every runtime memory access',
                        'projection source-index order needs separate unit tests and output replay',
                        'skipped calls are unevaluated, not zero occupancy or free space',
                        'no body stop, contact, hardware timing, unknown area or future motion certification'])


def digest(path):
    with Path(path).open('rb') as stream:
        result = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--mode', choices=('oldest', 'per_ray_candidate'), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.resolve() == args.input.resolve():
        parser.error('new output required')
    sources = [Path(__file__).resolve(), ROOT/'src/arena_autonomy/arena_autonomy/follow_trace.py']
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in sources}
    snapshot = args.output.with_suffix('.source.zip')
    with zipfile.ZipFile(snapshot, 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in sources:
            archive.write(path, path.relative_to(ROOT))
    before = digest(args.input)
    try:
        with args.input.open(encoding='utf-8') as stream:
            result = audit_events((json.loads(line) for line in stream), args.mode)
    except (ValueError, KeyError, TypeError) as error:
        result = dict(passed=False, error=str(error), driving_authorized=False, original_verdict_changed=False)
    result.update(input_sha256=before, input_unchanged=before == digest(args.input), source_sha256=hashes,
                  source_unchanged=all(digest(ROOT/p) == h for p, h in hashes.items()))
    result['passed'] &= result['input_unchanged'] and result['source_unchanged']
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('rows', 'source_sha256')}))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
