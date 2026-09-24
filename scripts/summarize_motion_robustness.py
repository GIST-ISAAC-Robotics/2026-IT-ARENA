#!/usr/bin/env python3
"""주행 중 오류의 정지/복구를 평가 표본으로 분리한다. 판정용 정답은 제어기에 전달하지 않는다."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


def speed(row):
    d = row['dynamics']
    return math.hypot(d['truth_longitudinal_mps'], d['truth_lateral_mps'])


def dynamic_audit(trace, config):
    faults = [v for v in config.values() if isinstance(v, dict) and v.get('end_s') is not None and
              (v.get('drop') or v.get('delay_s', 0) > .03)]
    if not faults:
        return None
    start, end = faults[0]['start_s'], faults[0]['end_s']
    pre = [r for r in trace if start-.5 <= r['time'] < start]
    during = [r for r in trace if start <= r['time'] < end]
    blocked = [r for r in during if r['safe_drive_speed_mps'] == 0 and
               (r.get('safety') or {}).get('reason') in ('motion_stale', 'motion_gap')]
    moving_before = bool(pre) and speed(pre[-1]) > .3
    stable, stop_row, began = False, None, None
    for r in during:
        if speed(r) < .01 and r['safe_drive_speed_mps'] == 0:
            if began is None:
                began, stop_row = r['time'], r
            if r['time']-began >= .3:
                stable = True
                break
        else:
            began, stop_row = None, None
    travel = None
    if stable and pre:
        segment = [pre[-1], *[r for r in during if r['time'] <= stop_row['time']]]
        travel = sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(segment, segment[1:]))
    later = [r for r in trace if r['time'] > end+.5 and speed(r) > .3]
    recovered = bool(later and during) and later[-1]['progress_m']-during[-1]['progress_m'] > 1.
    response = blocked[0]['time']-start if blocked else None
    return {'passed': bool(moving_before and blocked and response <= .25 and stable and recovered),
            'fault_start_s': start, 'fault_end_s': end, 'moving_before': moving_before,
            'speed_before_mps': speed(pre[-1]) if pre else None,
            'first_zero_safety_after_fault_s': response, 'stable_stop_during_fault': stable,
            'stop_after_fault_s': stop_row['time']-start if stable else None,
            'sampled_distance_until_stop_m': travel, 'resumed_more_than_1m': recovered,
            'scope': 'about 10 Hz pose/status samples; up to one sample timing uncertainty; not hardware braking guarantee'}


def summarize(run):
    report = json.loads((run/'report.json').read_text())
    trace = [json.loads(s) for s in (run/'trajectory.jsonl').read_text().splitlines()]
    road = json.loads((run/'road_audit.json').read_text()) if (run/'road_audit.json').exists() else {}
    dynamic = dynamic_audit(trace, report.get('requested_motion_impairment', {}))
    accepted = (report['passed'] and report.get('motion_impairment_audit', {}).get('verified') and
                report.get('motion_relay_exclusive') and road.get('driving_samples', 0) > 0 and
                road.get('stop_samples', 0) > 0 and road.get('outside_samples_over_1cm2') == 0 and
                road.get('stop_outside_samples_over_1cm2') == 0 and (dynamic is None or dynamic['passed']))
    unique = {r['autonomy']['sim_time_s']: r['autonomy'] for r in trace if r['autonomy'].get('started')}
    return {'run': run.name, 'accepted': bool(accepted), 'original_passed': report['passed'],
            'error': report.get('error'), 'progress_m': report.get('progress_m'),
            'peak_kmh': report.get('peak_ground_speed_mps', 0)*3.6,
            'corner_median_kmh': (report.get('corner_speed_metrics', {}).get('median_mps') or 0)*3.6,
            'tracking': report.get('tracking_metrics'),
            'states': dict(Counter(r['state'] for r in unique.values())),
            'safety': dict(Counter((r.get('safety') or {}).get('reason') for r in trace if r['autonomy'].get('started'))),
            'stop_stable': report.get('stop_stable'), 'shutdown_clean': report.get('shutdown_clean'),
            'remaining_pids': report.get('remaining_test_pids'), 'injection': report.get('motion_impairment_audit'),
            'dynamic_fault': dynamic, 'road': {k:v for k,v in road.items() if k != 'outside_samples'},
            'counts_scope': 'recorded status/pose samples, not all callbacks or separate failure events'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    result = summarize(args.run)
    (args.run/'motion_summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))
