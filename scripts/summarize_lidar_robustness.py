#!/usr/bin/env python3
"""보존한 실행 결과의 오차/주행/정지/종료를 재집계한다. 제어기로 전달하지 않는다."""
import argparse
import json
import math
from collections import Counter
from pathlib import Path


def audit_expected_stale_rejection(report, trajectory):
    """시작부터 너무 오래된 스캔만 오는 시험. 주행 중 제동 시험과 구분한다."""
    cfg = report.get('requested_lidar_impairment', {})
    if cfg.get('delay_s', 0.) - cfg.get('jitter_s', 0.) < .3:
        return None
    active = [r for r in trajectory if r['autonomy'].get('started')]
    duration = active[-1]['time']-active[0]['time'] if active else 0.
    commands = [r.get('safe_drive_speed_mps') for r in active]
    ground = [math.hypot(r['dynamics']['truth_longitudinal_mps'],
                        r['dynamics']['truth_lateral_mps']) for r in active]
    output_zero = bool(commands) and all(v is not None and math.isfinite(v) and abs(v) < 1e-6 for v in commands)
    stale = bool(active) and all((r.get('safety') or {}).get('reason') == 'stale_input' for r in active)
    quiet = bool(ground) and all(math.isfinite(v) and v < .01 for v in ground)
    no_collision = bool(active) and not any(r['collision'] for r in active)
    accepted = (duration >= 5. and output_zero and stale and quiet and no_collision and
                report.get('lidar_impairment_audit', {}).get('verified', False) and
                report.get('sequential_acquisition_verified', False) and
                report.get('shutdown_clean', False) and report.get('remaining_test_pids') == [] and
                not report.get('passed', True))
    return {'protection_passed': bool(accepted), 'active_samples': len(active), 'duration_sim_s': duration,
            'all_safe_commands_zero': output_zero, 'all_reasons_stale_input': stale,
            'ground_speed_under_1cm_s': quiet, 'collision_free_samples': no_collision,
            'scope': 'startup refusal of stale input, not lap completion or moving-vehicle braking; original passed preserved'}


def summarize(run):
    report = json.loads((run/'report.json').read_text())
    trajectory = [json.loads(line) for line in (run/'trajectory.jsonl').read_text().splitlines()]
    unique = {row['autonomy']['sim_time_s']: row['autonomy'] for row in trajectory
              if row['autonomy'].get('started')}
    states = Counter(row['state'] for row in unique.values())
    safety = Counter(row.get('safety', {}).get('reason', 'missing') for row in trajectory
                     if row['autonomy'].get('started'))
    road = json.loads((run/'road_audit.json').read_text()) if (run/'road_audit.json').exists() else {}
    ages = [row['autonomy']['motion']['oldest_observation_age_s'] for row in trajectory
            if row['autonomy'].get('motion', {}).get('reason') == 'ok']
    positive_safe = [abs(row.get('safe_drive_speed_mps', 0.)) for row in trajectory
                     if row['autonomy'].get('started')]
    return {'run':run.name, 'passed':report['passed'], 'error':report.get('error'),
        'progress_m':report.get('progress_m'), 'peak_kmh':report.get('peak_ground_speed_mps', 0.)*3.6,
        'corner_median_kmh':(report.get('corner_speed_metrics', {}).get('median_mps') or 0)*3.6,
        'tracking_metrics':report.get('tracking_metrics'), 'states_unique_status_samples':dict(states),
        'safety_trajectory_samples':dict(safety), 'status_scope':'sample counts, not timer callbacks or time fractions',
        'max_safe_command_mps':max(positive_safe, default=0.),
        'max_valid_observation_age_s':max(ages, default=None),
        'collision_samples':report.get('collision_samples'), 'stop_stable':report.get('stop_stable'),
        'stop_collision_samples':report.get('stop_collision_samples'),
        'shutdown_clean':report.get('shutdown_clean'), 'remaining_test_pids':report.get('remaining_test_pids'),
        'impairment':report.get('lidar_impairment_audit'),
        'expected_stale_rejection':audit_expected_stale_rejection(report, trajectory),
        'road':{k:v for k,v in road.items() if k != 'outside_samples'}}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+', type=Path)
    args = parser.parse_args()
    for run in args.runs:
        result = summarize(run)
        (run/'robustness_summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
        print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
