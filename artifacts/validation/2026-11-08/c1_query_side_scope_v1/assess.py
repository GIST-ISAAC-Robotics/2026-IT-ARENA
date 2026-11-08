"""보충 판정(원 report.passed와 별개). r27 12조건 + 주입 상태·1:1 무결성·판단 스캔 누락 검사. 사용: assess.py A|B

r27 assess.py의 12조건과 한계값을 그대로 쓴다. A는 무주입이어야 하고 B는 등록 C1 주입이 검증돼야 한다.
판단 시점 스캔 나이 분포는 기록만 하며 판정선이 아니다.
"""
import json, math, statistics, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROFILE = 'config/tests/lidar_uniform30_delay50_jitter20.json'
REQUESTED = dict(mode='uniform', amplitude_m=0.03, bias_m=0., delay_s=0.05, jitter_s=0.02, seed=23)
TOL = 1e-8  # g1_lidar_stress.audit_injection과 같은 수치 허용오차(부동소수 표현만 흡수)
FOLDERS = {'A': 'A_no_injection', 'B': 'B_c1_injection'}
read = lambda p: json.loads(p.read_bytes())
jsonl = lambda p: [json.loads(l) for l in p.read_text(encoding='utf-8').splitlines() if l.strip()]


def r27_gates(f, run, r, s):
    e = r['evaluation']; extra = r.get('additional_actors', {}); course = e.get('full_course_progress', {})
    gap = e['gap']; stop = r.get('stop_checks', {})
    t0, end = (r['case_window'][k] for k in ('green_sim_s', 'end_sim_s'))
    tr = [row for row in read(run/'trace.json') if t0 <= row['t'] <= end]
    start = next((row['t'] for row in tr if row['ground_speed_mps'] > .10), None)
    duration = longest = 0.
    for a, b in zip(tr, tr[1:]):
        if start is None or a['t'] < start:
            continue
        duration = duration+b['t']-a['t'] if a['ground_speed_mps'] < .05 else 0.
        longest = max(longest, duration)
    integrity = read(f/'postflight_integrity_run1.json')
    registered = read(HERE/'registered.json')
    gates = dict(valid=r.get('run_valid') is True and r.get('harness_valid') is True,
        one_lap=course.get('valid') is True and course.get('progress_m', 0) >= 44.64,
        no_contacts=gap.get('actor_contact_messages_with_ego') == 0 and extra.get('combined_contacts', {}).get('valid') is True and not extra.get('combined_contacts', {}).get('events'),
        minimum_gap=gap.get('sampled_finite_complete') is True and gap['min_sampled_gap_m'] >= .08,
        swept_gap=gap.get('swept_missing_actor_intervals') == 0 and gap.get('min_swept_lower_bound_m') is not None and gap['min_swept_lower_bound_m'] >= .08,
        additional_valid=extra.get('harness_valid') is True and extra.get('behavior_gates_passed') is True and extra.get('actor_count') == 1,
        no_overlap=gap.get('sampled_overlap_samples') == 0,
        surface=s.get('samples', 0) > 0 and s.get('sample_gaps_over_limit') == 0 and s.get('wall_wheel_samples') == 0
            and set(s['interpretations']) == {'any', 'center', 'full'} and all(v['max_off_wheels'] <= 2 for v in s['interpretations'].values()),
        stop_and_shutdown=bool(stop) and all(stop.values()) and r.get('shutdown_clean') is True,
        source_unchanged=r.get('source_unchanged') is True and not integrity['changed']
            and integrity.get('snapshot_unchanged') is True and integrity['inputs'] == len(registered['sources']),
        process_clean=not (f/'postflight_processes_run1.txt').read_text().strip(),
        sustained_motion=start is not None and longest <= 2.5)
    summary = dict(course=course, longest_low_speed_s=longest, peak_mps=max((row['ground_speed_mps'] for row in tr), default=None),
                   min_gap=gap.get('min_sampled_gap_m'), min_swept_gap=gap.get('min_swept_lower_bound_m'))
    return gates, summary


def injection_gates(key, run, r, acquisition, received):
    plain = not any('impairment' in row or 'injected_delay_s' in row for row in acquisition)
    if key == 'A':
        return dict(injection_state=r.get('lidar_test_profile') is None and not (run/'lidar_test_profile.json').exists() and plain,
                    acquisition_received_one_to_one=len(acquisition) == len(received) > 0), None
    profile, evidence = r.get('lidar_test_profile') or {}, r.get('lidar_stress_evidence') or {}
    stats = evidence.get('last_statistics') or {}
    low, high, error = evidence.get('minimum_added_delay_s'), evidence.get('maximum_added_delay_s'), stats.get('max_abs_error_m')
    gates = dict(
        injection_state=profile.get('path') == PROFILE and profile.get('requested') == REQUESTED
            and r.get('integrity', {}).get('lidar_stress_verified') is True and evidence.get('verified') is True
            and len(evidence.get('checks', {})) == 6 and all(evidence['checks'].values())
            and evidence.get('runtime_profile_unchanged') is True and stats.get('config') == REQUESTED,
        added_delay_range=all(isinstance(v, float) and math.isfinite(v) for v in (low, high)) and .03-TOL <= low <= high <= .07+TOL,
        bounded_error=isinstance(error, float) and 0 < error <= .03+TOL,
        acquisition_received_one_to_one=evidence.get('acquisition_records') == evidence.get('received_scans') == len(acquisition) == len(received) > 0
            and not evidence.get('unmatched_received_indices') and not evidence.get('invalid_received_indices') and not evidence.get('source_errors'))
    return gates, dict(minimum_added_delay_s=low, maximum_added_delay_s=high, max_abs_error_m=error)


def decision_gates(run, acquisition):
    rows = jsonl(run/'follow_decisions.jsonl')
    footer = [row['data'] for row in rows if row['kind'] == 'footer']
    calls = [row['data']['input'] for row in rows if row['kind'] == 'call']
    window = read(run/'report.json')['case_window']
    all_calls = calls
    calls = [c for c in calls if window['green_sim_s'] <= c['reference_s'] <= window['end_sim_s']]
    stamps = {round(row['first_ray_stamp_s'], 6) for row in acquisition}
    ages, missing, scans = [], 0, set()
    for call in calls:
        stamp = round(call['scan']['sec']+call['scan']['nanosec']*1e-9, 6)
        scans.add(stamp); missing += stamp not in stamps
        ages.append(call['reference_s']-stamp)
    gates = dict(decision_scans_acquired=bool(calls) and missing == 0,
                 decision_log_complete=len(footer) == 1 and footer[0].get('dropped') == 0 and footer[0].get('failed') == 0
                     and footer[0].get('capture_errors') == 0 and footer[0].get('accepted') == footer[0].get('written'))
    q = sorted(ages)
    age = dict(calls=len(calls), unique_scans=len(scans), not_in_acquisition=missing,
               min=q[0] if q else None, p50=statistics.median(q) if q else None,
               p95=q[min(len(q)-1, int(.95*len(q)))] if q else None, max=q[-1] if q else None,
               scope='case-window decision reference_s minus first-ray stamp; footer completeness covers all calls',
               excluded_outside_case_calls=len(all_calls)-len(calls),
               all_calls_not_in_acquisition=sum(round(c['scan']['sec']+c['scan']['nanosec']*1e-9,6) not in stamps for c in all_calls))
    return gates, age


def main(key):
    f = HERE/FOLDERS[key]; run = f/'run1'
    r = read(run/'report.json'); s = read(f/'surface_run1.json')
    acquisition, received = read(run/'acquisition.json'), jsonl(run/'scans_received.jsonl')
    gates, summary = r27_gates(f, run, r, s)
    more, injection = injection_gates(key, run, r, acquisition, received)
    decisions, age = decision_gates(run, acquisition)
    gates.update(more); gates.update(decisions)
    exit_line = (f/'run1.exit').read_text().strip() if (f/'run1.exit').exists() else None
    out = dict(run=key, gates=gates, passed=all(gates.values()), **summary, injection=injection, scan_age_s=age,
               raw_pass=r.get('passed'), raw_gates=r.get('gates'), raw_exit_line_at_assessment=exit_line,
               scope='supplementary; raw report.passed and HARNESS_EXIT are preserved separately and not overridden')
    (f/'assessment_run1.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(out, ensure_ascii=False))
    return 0 if out['passed'] else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
