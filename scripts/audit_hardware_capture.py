#!/usr/bin/env python3
"""정규화한 MCU 엔코더/IMU 기록의 수동 감사. 드라이버·구동·시계 보정기는 아니다."""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics

from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
import arena_vehicle_interface.drive_feedback as feedback_module

SCHEMA = 'arena_hardware_capture_v1'
MAX_RECORDS = 500_000
MAX_LINE_BYTES = 16_384


def integer(value, label, low=0, high=2**63-1):
    if type(value) is not int or not low <= value <= high:
        raise ValueError(f'{label}: integer outside declared range')
    return value


def text(value, label):
    if not isinstance(value, str) or not value.strip() or len(value) > 128:
        raise ValueError(f'{label}: nonempty short string required')
    return value


def vector(value):
    if not isinstance(value, list) or len(value) != 3 or any(
            type(v) not in (float, int) or abs(v) > 1e6 or not math.isfinite(v) for v in value):
        # 수치 처리 파일 범위 제한이지 제어기의 IMU 고장 판정 한도가 아니다.
        raise ValueError('gyro_rad_s: finite three-axis vector within file numeric range required')
    return value


def summary(values):
    if any(not math.isfinite(v) for v in values):
        raise ValueError('derived file statistic is not finite')
    return None if not values else dict(minimum=min(values), median=statistics.median(values), maximum=max(values))


def validate_header(header):
    if not isinstance(header, dict):
        raise ValueError('capture header must be an object')
    if header.get('schema') != SCHEMA or header.get('kind') != 'header':
        raise ValueError('capture header/schema required')
    if header.get('origin') not in ('synthetic', 'hardware_capture'):
        raise ValueError('origin must distinguish synthetic and hardware_capture')
    required = dict(device_clock='mcu_monotonic_us', receive_clock='host_monotonic_ns',
                    encoder_unit='extended_signed_motor_counts', gyro_unit='rad/s')
    if any(header.get(k) != v for k, v in required.items()):
        raise ValueError('explicit clock/count/gyro units do not match schema')
    text(header.get('gyro_frame'), 'gyro_frame')
    # 값만 맞는다고 실제 축 방향이 검증되는 것은 아니다. 장착 기준을 기록한다.
    text(header.get('mounting_note'), 'mounting_note')
    try:
        spec = MotorEncoderSpec(**header['motor_spec'])
        derived = (spec.counts_per_wheel_revolution, spec.mean_wheel_travel_m_per_count)
        if any(not math.isfinite(v) or v <= 0 for v in derived):
            raise ValueError('derived encoder conversion is not finite and positive')
    except (KeyError, TypeError, ZeroDivisionError, OverflowError) as error:
        raise ValueError('explicit finite motor spec required') from error
    windows = header.get('externally_stationary_windows', [])
    if not isinstance(windows, list):
        raise ValueError('stationary windows must be a list')
    for window in windows:
        if not isinstance(window, dict):
            raise ValueError('stationary window must be an object')
        text(window.get('boot'), 'window boot')
        text(window.get('basis'), 'external stationary basis')
        start = integer(window.get('first_us'), 'window first')
        end = integer(window.get('last_us'), 'window last')
        if end <= start:
            raise ValueError('stationary window must have positive duration')
        requirements = window.get('review_requirements')
        if requirements is not None:
            if not isinstance(requirements, dict):
                raise ValueError('review requirements must be an object')
            integer(requirements.get('minimum_span_us'), 'minimum span', 1, end-start)
            integer(requirements.get('max_sample_gap_us'), 'maximum sample gap', 1)
            integer(requirements.get('minimum_imu_samples'), 'minimum IMU samples', 3, MAX_RECORDS)
            integer(requirements.get('minimum_encoder_samples'), 'minimum encoder samples', 2, MAX_RECORDS)
    return spec, windows


def stationary_candidate(window, imu, encoder, issues):
    """교정값 채택이 아닌, 사용자가 명시한 자료량 기준에 대한 후보 통계다."""
    requirements = window.get('review_requirements')
    rejected, coverage = [], {}
    if requirements is None:
        rejected.append('review_requirements_missing')
    if issues:
        rejected.append('capture_integrity_issue')
    for kind, rows in (('imu', imu), ('encoder', encoder)):
        capture = [r['capture_us'] for r in rows]
        intervals = [b-a for a, b in zip(capture, capture[1:])]
        gaps = sum(max(0, b['seq']-a['seq']-1) for a, b in zip(rows, rows[1:]))
        span = capture[-1]-capture[0] if capture else 0
        first_gap = capture[0]-window['first_us'] if capture else None
        last_gap = window['last_us']-capture[-1] if capture else None
        coverage[kind] = dict(samples=len(rows), observed_span_us=span,
                              first_observed_us=capture[0] if capture else None,
                              last_observed_us=capture[-1] if capture else None,
                              start_unobserved_us=first_gap, end_unobserved_us=last_gap,
                              max_capture_gap_us=max(intervals) if intervals else None,
                              missing_sequence_numbers=gaps)
        if gaps:
            rejected.append(f'{kind}_sequence_gap')
        if requirements is not None:
            if len(rows) < requirements[f'minimum_{kind}_samples']:
                rejected.append(f'{kind}_insufficient_samples')
            if span < requirements['minimum_span_us']:
                rejected.append(f'{kind}_insufficient_observed_span')
            maximum_gap = requirements['max_sample_gap_us']
            if (first_gap is None or last_gap is None or first_gap > maximum_gap
                    or last_gap > maximum_gap or any(dt > maximum_gap for dt in intervals)):
                rejected.append(f'{kind}_coverage_gap')
    count_span = (max(r['motor_counts'] for r in encoder)-min(r['motor_counts'] for r in encoder)
                  if encoder else None)
    if count_span is not None and count_span != 0:
        rejected.append('motor_counts_changed')
    eligible = not rejected
    return dict(window=window, imu_samples=len(imu), encoder_samples=len(encoder),
                motor_count_span=count_span, coverage=coverage,
                candidate_only=eligible, rejection_reasons=rejected,
                candidate_bias_rad_s=[statistics.fmean(r['gyro_rad_s'][i] for r in imu)
                                      for i in range(3)] if eligible else None,
                gyro_std_rad_s=[statistics.pstdev(r['gyro_rad_s'][i] for r in imu)
                               for i in range(3)] if eligible else None,
                applied=False, calibration_ready=False, chassis_stationarity_verified=False,
                review_scope='declared_sample_quality_only_not_sensor_calibration_acceptance')


def audit(header, records):
    spec, windows = validate_header(header)
    groups, last, issues, counts = defaultdict(list), {}, [], Counter()
    sequence_gaps = Counter()
    seen_boots, closed_boots, current_boot = [], set(), None
    receive_previous = None
    for index, row in enumerate(records, 1):
        if index > MAX_RECORDS:
            raise ValueError('record limit exceeded')
        if not isinstance(row, dict):
            raise ValueError(f'row {index}: capture row must be an object')
        kind = row.get('kind')
        if kind not in ('encoder', 'imu'):
            raise ValueError(f'row {index}: unknown kind')
        boot = text(row.get('boot'), 'boot')
        seq = integer(row.get('seq'), 'seq')
        device = integer(row.get('capture_us'), 'capture_us')
        receive = integer(row.get('receive_ns'), 'receive_ns')
        if kind == 'encoder':
            integer(row.get('motor_counts'), 'motor_counts', -(2**63))
        else:
            vector(row.get('gyro_rad_s'))
        if receive_previous is not None and receive < receive_previous:
            issues.append(dict(row=index, reason='host_receive_clock_regressed'))
        receive_previous = receive
        if current_boot != boot:
            if current_boot is not None:
                closed_boots.add(current_boot)
            if boot in closed_boots:
                issues.append(dict(row=index, reason='old_boot_reappeared', boot=boot))
            if boot not in seen_boots:
                seen_boots.append(boot)
            current_boot = boot
        key = (boot, kind)
        prior = last.get(key)
        if prior:
            if seq <= prior['seq']:
                issues.append(dict(row=index, reason='sequence_not_increasing', stream=kind, boot=boot))
            elif seq > prior['seq']+1:
                sequence_gaps[key] += seq-prior['seq']-1
            if device <= prior['capture_us']:
                issues.append(dict(row=index, reason='device_capture_not_increasing', stream=kind, boot=boot))
        groups[key].append(row)
        last[key] = row
        counts[kind] += 1
    if not counts:
        issues.append(dict(reason='empty_capture'))
    for kind in ('encoder', 'imu'):
        if not counts[kind]:
            issues.append(dict(reason='missing_stream', stream=kind))
    for boot in seen_boots:
        for kind in ('encoder', 'imu'):
            if not groups.get((boot, kind)):
                issues.append(dict(reason='missing_stream_in_boot', stream=kind, boot=boot))
    streams = []
    for (boot, kind), rows in groups.items():
        pairs = [(a, b) for a, b in zip(rows, rows[1:])
                 if b['capture_us'] > a['capture_us'] and b['seq'] > a['seq']]
        dt = [(b['capture_us']-a['capture_us'])*1e-6 for a, b in pairs]
        received_dt = [(b['receive_ns']-a['receive_ns'])*1e-9 for a, b in pairs]
        entry = dict(boot=boot, kind=kind, samples=len(rows), missing_sequence_numbers=sequence_gaps[(boot, kind)],
                     capture_interval_s=summary(dt), receive_interval_s=summary(received_dt))
        if len(rows) >= 2:
            device_elapsed = rows[-1]['capture_us']-rows[0]['capture_us']
            receive_elapsed = (rows[-1]['receive_ns']-rows[0]['receive_ns'])/1000
            entry['host_minus_device_elapsed_ms'] = (receive_elapsed-device_elapsed)/1000
        if kind == 'encoder':
            speeds = [(b['motor_counts']-a['motor_counts'])*spec.mean_wheel_travel_m_per_count/t
                      for (a, b), t in zip(pairs, dt)]
            entry.update(mean_wheel_speed_mps=summary(speeds),
                         one_count_speed_quantum_mps=summary([spec.speed_quantum_mps(t) for t in dt]),
                         positive_count_steps=sum(b['motor_counts'] > a['motor_counts'] for a, b in pairs),
                         negative_count_steps=sum(b['motor_counts'] < a['motor_counts'] for a, b in pairs),
                         unchanged_count_steps=sum(b['motor_counts'] == a['motor_counts'] for a, b in pairs))
        streams.append(entry)
    stationary = []
    for window in windows:
        imu = [r for r in groups.get((window['boot'], 'imu'), [])
               if window['first_us'] <= r['capture_us'] <= window['last_us']]
        encoder = [r for r in groups.get((window['boot'], 'encoder'), [])
                   if window['first_us'] <= r['capture_us'] <= window['last_us']]
        stationary.append(stationary_candidate(window, imu, encoder, issues))
    return dict(schema=SCHEMA, scope='passive_record_audit_not_driver_calibration_or_safety_authorization',
                origin=header['origin'], record_integrity_valid=not issues,
                recorded_sequence_complete=not issues and not any(sequence_gaps.values()),
                complete_capture_verified=False,
                issues=issues, counts=dict(counts), boots=seen_boots, streams=streams,
                boot_changes_require_explicit_rearm=len(seen_boots) > 1,
                mean_wheel_travel_mm_per_count=1000*spec.mean_wheel_travel_m_per_count,
                counts_per_mean_wheel_revolution=spec.counts_per_wheel_revolution,
                stationary_windows=stationary, absolute_transport_delay_measured=False,
                automatic_clock_mapping_or_bias_application=False, ready_to_drive=False,
                limits=['Unequal clock epochs cannot be subtracted as transport latency.',
                        'Elapsed-clock difference includes transport variation and drift; it is not a drift estimate.',
                        'Missing sequence numbers identify record gaps, not sensor vs transport loss cause.',
                        'No gaps inside the file does not prove that capture started or ended without loss.',
                        'Counter wrap must already be extended by the producer; no guessed unwrap is applied.',
                        'Motor mean and unchanged counts do not prove chassis rest or both-wheel rest.',
                        'Gyro candidates need independent immobilization, axis/temperature and hardware verification.'])


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'duplicate JSON field: {key}')
        result[key] = value
    return result


def read_capture(path):
    with path.open('rb') as stream:
        def rows():
            for index in range(MAX_RECORDS+2):
                raw = stream.readline(MAX_LINE_BYTES+1)
                if not raw:
                    return
                if len(raw) > MAX_LINE_BYTES:
                    raise ValueError('capture line exceeds file limit')
                row = json.loads(raw.decode('utf-8'), object_pairs_hook=unique_object,
                                 parse_constant=lambda _: (_ for _ in ()).throw(ValueError('nonfinite JSON')))
                if not isinstance(row, dict):
                    raise ValueError('capture rows must be JSON objects')
                yield row
            raise ValueError('capture record limit exceeded')
        iterator = rows()
        header = next(iterator, None)
        if header is None:
            raise ValueError('capture has no header')
        return audit(header, iterator)


def digest(path):
    # Jetson Humble/Python 3.10에는 hashlib.file_digest가 없다.
    checksum = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            checksum.update(chunk)
    return checksum.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists() or args.output.resolve() == args.input.resolve():
        parser.error('output must be a new file, separate from input')
    before = digest(args.input)
    result = read_capture(args.input)
    result.update(input_sha256=before, input_unchanged=before == digest(args.input),
                  source_sha256={'audit_hardware_capture.py': digest(Path(__file__)),
                                 'drive_feedback.py': digest(Path(feedback_module.__file__))})
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({k: result[k] for k in ('record_integrity_valid', 'counts', 'boots', 'input_unchanged', 'ready_to_drive')}))
    return 0 if result['record_integrity_valid'] and result['input_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
