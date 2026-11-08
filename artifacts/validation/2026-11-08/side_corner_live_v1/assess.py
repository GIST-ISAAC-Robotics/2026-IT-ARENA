"""실제 첫 코너에서 옆차와 함께 있었는지 사전 조건으로 평가한다."""
from pathlib import Path
from bisect import bisect_right
import json
import math
import sys

HERE = Path(__file__).resolve().parent
read = lambda p: json.loads(p.read_bytes())
wrap = lambda a: math.atan2(math.sin(a), math.cos(a))


def main(name):
    folder = HERE/name
    run = folder/'run1'
    report = read(run/'report.json')
    surface = read(folder/'surface_run1.json')
    first, end = (report['case_window'][k] for k in ('green_sim_s', 'end_sim_s'))
    trace = [r for r in read(run/'trace.json') if first <= r['t'] <= end]
    actors = [json.loads(l) for l in (run/'actor_poses.jsonl').read_text().splitlines()]
    times = [r[0] for r in actors]
    current = longest_parallel = low = longest_low = 0.
    began = None
    rows = []
    previous = None
    for row in trace:
        if began is None and row['ground_speed_mps'] > .1:
            began = row['t']
        i = bisect_right(times, row['t'])-1
        paired = i >= 0 and row['t']-times[i] <= .05+1e-9
        longitudinal = lateral = difference = None
        if paired:
            _, x, y, yaw = actors[i]
            dx, dy = x-row['x'], y-row['y']
            c, s = math.cos(row['yaw']), math.sin(row['yaw'])
            longitudinal, lateral = c*dx+s*dy, -s*dx+c*dy
            difference = math.degrees(abs(wrap(yaw-row['yaw'])))
        angle = math.degrees(wrap(row['yaw']-math.pi/2))
        parallel = bool(paired and 20 <= angle <= 70 and abs(longitudinal) <= .30
                        and .15 <= abs(lateral) <= .55 and difference <= 30)
        if previous is not None:
            dt = row['t']-previous['t']
            continuous = 0 < dt <= .20+1e-9
            current = current+dt if continuous and parallel and previous['parallel'] else 0.
            longest_parallel = max(longest_parallel, current)
            low = low+dt if continuous and began is not None and previous['t'] >= began and previous['speed'] < .05 else 0.
            longest_low = max(longest_low, low)
        previous = dict(t=row['t'], parallel=parallel, speed=row['ground_speed_mps'])
        if 10 <= angle <= 100:
            rows.append(dict(t=row['t'], heading_deg=angle, paired=paired, longitudinal_m=longitudinal,
                             lateral_m=lateral, relative_heading_deg=difference, parallel=parallel))
    gap = report['evaluation']['gap']
    gates = dict(valid=report.get('run_valid') is True and report.get('harness_valid') is True,
        curve=all(report['gates'].get(k) is True for k in ('curve_exit_reached', 'curve_heading_changed')),
        progress=trace[-1]['progress_m'] >= 10.5,
        actual_parallel=longest_parallel >= .5,
        sustained=began is not None and longest_low <= 2.5,
        no_contacts=gap.get('actor_contact_messages_with_ego') == 0,
        gap=gap.get('sampled_finite_complete') is True and gap['min_sampled_gap_m'] >= .08,
        swept=gap.get('swept_missing_actor_intervals') == 0 and gap.get('min_swept_lower_bound_m') is not None and gap['min_swept_lower_bound_m'] >= .08,
        surface=surface.get('samples', 0)>0 and surface.get('sample_gaps_over_limit') == 0
            and surface.get('wall_wheel_samples') == 0 and set(surface['interpretations']) == {'any','center','full'}
            and all(r['max_off_wheels'] <= 2 for r in surface['interpretations'].values()),
        stop_shutdown=bool(report.get('stop_checks')) and all(report['stop_checks'].values()) and report.get('shutdown_clean') is True,
        source=report.get('source_unchanged') is True and not read(folder/'postflight_integrity_run1.json')['changed'],
        process_clean=not (folder/'postflight_processes_run1.txt').read_text().strip(),
        header=report.get('handoff_execution',{}).get('passed') is True)
    result = dict(passed=all(gates.values()), gates=gates, progress_m=trace[-1]['progress_m'],
        longest_corner_parallel_s=longest_parallel, longest_low_speed_s=longest_low,
        min_gap_m=gap['min_sampled_gap_m'], min_swept_gap_m=gap.get('min_swept_lower_bound_m'),
        raw_pass=report['passed'], raw_gates=report['gates'])
    (folder/'corner_pairs.json').write_text(json.dumps(rows, indent=1)+'\n')
    (folder/'assessment_run1.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1]))
