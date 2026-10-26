"""G1 시험 전용 C1 오차/지연 입력 검증. 제어·보호 한도는 바꾸지 않는다."""
from bisect import bisect_left
from dataclasses import asdict
import json
import math
from pathlib import Path

from arena_vehicle_interface.lidar_impairment import ImpairmentConfig


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def load_test_profile(path, repo):
    """시험 입력만 저장소 내부에서 읽는다. 기본값에는 주입 설정을 만들지 않는다."""
    if path is None:
        return None, None
    resolved = Path(path).resolve()
    resolved.relative_to(Path(repo).resolve())
    config = asdict(ImpairmentConfig(**json.loads(resolved.read_text(encoding='utf-8'))))
    if config['mode'] == 'gaussian':
        raise ValueError('G1 stress profile requires a preregistered bounded error')
    return resolved, config


def require_injection_evidence(report, evidence):
    """기존 판정을 보존하고 주입 무결성만 추가 요구한다. 실패를 성공으로 바꾸지 않는다."""
    verified = (evidence.get('verified') is True
                and '/scan' in report.get('autonomy_subscriptions', [])
                and '/scan' in report.get('safety_subscriptions', []))
    report['lidar_stress_evidence'] = evidence
    report['integrity']['lidar_stress_verified'] = verified
    report['run_valid'] = report['run_valid'] and verified
    report['passed'] = report['passed'] and verified
    return report


def audit_injection(records, received, requested):
    """발행 계측과 받은 스캔을 취득 시각으로 대조. 지정값만으로 주입 성공을 주장하지 않는다."""
    config = asdict(ImpairmentConfig(**requested))
    if config['mode'] == 'gaussian':
        raise ValueError('this G1 bounded stress audit does not authorize Gaussian experiments')
    checks = dict(acquisition_present=bool(records), received_present=bool(received))
    errors, delays, times = [], [], []
    previous, last_count, last_scans = -math.inf, -1, -1
    for index, row in enumerate(records):
        keys = ('first_ray_stamp_s', 'last_ray_stamp_s', 'completion_stamp_s',
                'requested_release_stamp_s', 'release_source_stamp_s', 'injected_delay_s',
                'max_capture_time_error_s')
        if any(not finite(row.get(k)) for k in keys):
            errors.append(dict(index=index, reason='missing_or_nonfinite_time'))
            continue
        first, last, complete, due, release, delay, capture_error = (row[k] for k in keys)
        stats = row.get('impairment', {})
        values = [stats.get(k) for k in ('mean_error_m', 'rms_error_m', 'max_abs_error_m')]
        counters = [stats.get(k) for k in ('samples', 'scans', 'invalidated')]
        if (stats.get('config') != config or any(not finite(v) for v in values)
                or any(type(v) is not int or v < 0 for v in counters)):
            errors.append(dict(index=index, reason='missing_or_mismatched_impairment'))
            continue
        samples, scans, invalid = counters
        bound = abs(config['bias_m'])+config['amplitude_m']
        mean, rms, peak = values
        # 0.0021001 s는 기존 순차 취득 프레임 허용 한도. 새 지연 허용 완화가 아니다.
        valid = (0 <= first < last <= complete <= due <= release+1e-8
                 and first > previous and row.get('model') == 'sequential'
                 and row.get('discarded_source_gaps') == 0
                 and 0 <= capture_error <= .0021001
                 and abs(last-first-.0998) <= 1e-7
                 and abs(delay-(release-complete)) <= 1e-8
                 and config['delay_s']-config['jitter_s']-1e-8 <= due-complete
                 <= config['delay_s']+config['jitter_s']+1e-8
                 and -1e-8 <= release-due <= .0021001
                 and samples >= last_count and scans >= last_scans and samples > 0 and scans > 0
                 and invalid <= samples and 0 <= rms <= peak+1e-9
                 and abs(mean) <= rms+1e-9 and peak <= bound+1e-8)
        if not valid:
            errors.append(dict(index=index, reason='time_or_statistics_contract'))
        if config['mode'] == 'none' and any(abs(actual-expected) > 1e-8 for actual, expected in (
                (mean, config['bias_m']), (rms, abs(config['bias_m'])), (peak, abs(config['bias_m'])))):
            errors.append(dict(index=index, reason='fixed_bias_statistics_mismatch'))
        previous, last_count, last_scans = first, samples, scans
        times.append(first)
        delays.append(delay)
    checks['all_acquisition_records_verified'] = len(times) == len(records) and not errors
    missing, invalid_received, before = [], [], -math.inf
    times = sorted(times)
    for index, row in enumerate(received):
        t = row.get('stamp_s')
        if not finite(t) or t <= before or not finite(row.get('time_increment')):
            invalid_received.append(index)
            continue
        before = t
        j = bisect_left(times, t-1e-8)
        if j == len(times) or abs(times[j]-t) > 1e-8:
            missing.append(index)
        if (abs(row['time_increment']-.0002) > 1e-9
                or not isinstance(row.get('ranges'), list) or len(row['ranges']) != 500):
            invalid_received.append(index)
    checks['received_header_matches_acquisition'] = bool(received) and not missing and not invalid_received
    if config['amplitude_m'] or config['bias_m']:
        checks['nonzero_distortion_observed'] = bool(records) and (
            records[-1].get('impairment', {}).get('max_abs_error_m', 0) > 0)
    checks['source_records_one_to_one'] = len(times) == len(set(times))
    return dict(schema='g1_lidar_stress_v1', verified=all(checks.values()), checks=checks,
                requested=config, acquisition_records=len(records), received_scans=len(received),
                source_errors=errors[:20], unmatched_received_indices=missing[:20],
                invalid_received_indices=invalid_received[:20],
                minimum_added_delay_s=min(delays, default=None), maximum_added_delay_s=max(delays, default=None),
                last_statistics=records[-1].get('impairment') if records else None,
                limits=['synthetic bounded radial error and publish delay, not measured C1 noise or transport',
                        'both controller and safety must separately be checked to subscribe to the same /scan',
                        'received header/acquisition match does not prove every downstream callback processed each scan',
                        'not a driving, braking, real-time Jetson or hardware safety verdict'])
