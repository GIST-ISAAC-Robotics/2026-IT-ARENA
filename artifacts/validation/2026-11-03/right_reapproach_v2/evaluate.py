"""실제 반환과 평가 자세로 오른쪽 관측→후방 사각→재접근을 구별한다."""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from audit_actor_outline_returns import read_poses, interpolate
from arena_autonomy.follow_trace import thaw


def actor_relative(ep, ap, mount):
    dx, dy = ap[:, 0]-ep[:, 0], ap[:, 1]-ep[:, 1]
    c, s = np.cos(ep[:, 2]), np.sin(ep[:, 2])
    return np.column_stack((c*dx+s*dy-mount['lidar_x_m'],
                            -s*dx+c*dy-mount['lidar_y_m'], ap[:, 2]-ep[:, 2]))


def wholly_in_rear_mask(relative):
    corners = np.array([[-.10, -.075], [-.10, .075], [.10, -.075], [.10, .075]])
    for x, y, yaw in relative:
        c, s = math.cos(yaw), math.sin(yaw)
        transformed = corners@np.array([[c, s], [-s, c]])+np.array([x, y])
        bearings = np.arctan2(transformed[:, 1], transformed[:, 0])
        if not np.all(np.abs(bearings) >= math.radians(150.5)):
            return False
    return True


def phases(rows):
    """이전/이후 관측을 같은 스캔으로 채우거나 사각 기하만으로 검출을 주장하지 않는다."""
    if any(b['stamp'] <= a['stamp'] for a, b in zip(rows, rows[1:])):
        return dict(established=False, reason='nonincreasing_scan_order')
    visible = []
    for row in rows:
        if row['hits'] >= 3:
            visible.append(row)
        if row['whole_rear_mask'] and row['hits'] == 0 and len(visible) >= 2:
            later = [r for r in rows if r['stamp'] > row['stamp'] and r['hits'] >= 3 and r['right']
                     and r['relative_x_m'] >= row['relative_x_m']+.2]
            if len(later) >= 2:
                return dict(established=True, initial_observed=visible[:2], blind=row, returned=later[:2])
    return dict(established=False, observed_scans=sum(r['hits'] >= 3 for r in rows),
                blind_scans=sum(r['whole_rear_mask'] and r['hits'] == 0 for r in rows))


def scan_evidence(run, green, end):
    ego, actor = read_poses(run/'ego_poses.jsonl'), read_poses(run/'actor_poses.jsonl')
    seen, rows = set(), []
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        mount = header['motion_config']
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            scan = thaw(event['data']['input']['scan'])
            stamp = scan['sec']+scan['nanosec']*1e-9
            if stamp in seen:
                continue
            seen.add(stamp)
            ranges = np.asarray(scan['ranges'], dtype=float)
            indices = np.arange(len(ranges))
            times = stamp+indices*scan['time_increment']
            if (not len(times) or times[0] < max(green, ego[0, 0], actor[0, 0])
                    or times[-1] > min(end, ego[-1, 0], actor[-1, 0])):
                continue
            # 처음/끝 및 중간 세 시점의 전체 외곽이 마스크 안인지 확인한다.
            checkpoints = np.linspace(times[0], times[-1], 5)
            rel = actor_relative(interpolate(ego, checkpoints), interpolate(actor, checkpoints), mount)
            whole = wholly_in_rear_mask(rel)
            angle = scan['angle_min']+indices*scan['angle_increment']+mount['lidar_yaw_rad']
            wrapped = (angle+math.pi) % (2*math.pi)-math.pi
            finite = np.isfinite(ranges) & (ranges >= scan['range_min']) & (ranges <= scan['range_max'])
            rear_finite = int(np.sum(finite & (np.abs(wrapped) >= math.radians(150.5))))
            times, indices, rr, angle = times[finite], indices[finite], ranges[finite], angle[finite]
            ep, ap = interpolate(ego, times), interpolate(actor, times)
            lx, ly = mount['lidar_x_m']+rr*np.cos(angle), mount['lidar_y_m']+rr*np.sin(angle)
            dx = ep[:, 0]+np.cos(ep[:, 2])*lx-np.sin(ep[:, 2])*ly-ap[:, 0]
            dy = ep[:, 1]+np.sin(ep[:, 2])*lx+np.cos(ep[:, 2])*ly-ap[:, 1]
            x = np.cos(ap[:, 2])*dx+np.sin(ap[:, 2])*dy
            y = -np.sin(ap[:, 2])*dx+np.cos(ap[:, 2])*dy
            # 20mm 대응 여유. 상대 주변 다른 표면까지의 거리를 별도로 확인한다.
            near = ((np.abs(x) <= .12) & (np.abs(y) <= .095)
                    & ((np.abs(np.abs(x)-.10) <= .02) | (np.abs(np.abs(y)-.075) <= .02)))
            rows.append(dict(stamp=stamp, elapsed_s=stamp-green, hits=int(near.sum()),
                             whole_rear_mask=whole, rear_mask_finite_returns=rear_finite,
                             relative_x_m=float(rel[2, 0]), relative_y_m=float(rel[2, 1]),
                             right=bool(rel[2, 1] < 0)))
    return rows


def main():
    run = HERE/'run1'
    report = json.loads((run/'report.json').read_bytes())
    surface = json.loads((HERE/'run1_surface.json').read_bytes())
    registered = json.loads((HERE/'registered_final.json').read_bytes())
    hashes_ok = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == digest
                    for p, digest in registered['source_sha256'].items())
    evaluation = report['evaluation']
    green, end = evaluation['t_green'], report['case_window']['end_sim_s']
    rows = scan_evidence(run, green, end)
    phase = phases(rows)
    trace = [r for r in json.loads((run/'trace.json').read_bytes()) if green <= r['t'] <= end]
    first = next((r['t'] for r in trace if r['ground_speed_mps'] >= .1), None)
    span, stall = 0., 0.
    for a, b in zip(trace, trace[1:]):
        if first is not None and a['t'] >= first:
            span = span+b['t']-a['t'] if a['ground_speed_mps'] < .05 else 0.
            stall = max(stall, span)
    progress = evaluation['progress']['at_window_end_m']
    gap = evaluation['gap']
    wheels = surface['interpretations']
    checks = dict(candidate_recorded=report.get('handoff_execution',{}).get('checks',{}).get('side_departure') is True, scene=phase['established'], run_valid=report.get('run_valid') is True,
                  sources=hashes_ok and report.get('source_unchanged') is True,
                  stop=bool(report.get('stop_checks')) and all(report['stop_checks'].values()),
                  shutdown=report.get('shutdown_clean') is True,
                  mask_has_no_finite_returns=bool(rows) and all(r['rear_mask_finite_returns'] == 0 for r in rows),
                  contact=gap['actor_contact_messages_with_ego'] == 0,
                  overlap=gap['sampled_overlap_samples'] == 0,
                  minimum_gap=gap['min_sampled_gap_m'] is not None and gap['min_sampled_gap_m'] >= .08,
                  minimum_swept_gap=gap.get('min_swept_lower_bound_m') is not None
                      and gap['min_swept_lower_bound_m'] >= .08 and gap.get('swept_missing_actor_intervals') == 0,
                  wheel_evidence=surface['samples'] > 0 and surface['sample_gaps_over_limit'] == 0
                      and set(wheels) == {'any', 'center', 'full'},
                  wheel_wall=surface['wall_wheel_samples'] == 0,
                  at_most_two_wheels=bool(wheels) and all(v['max_off_wheels'] <= 2 for v in wheels.values()),
                  progress=progress >= 2.5, no_long_stall=first is not None and stall <= 2.5)
    result = dict(passed=all(checks.values()), checks=checks, phase=phase,
                  metrics=dict(progress_m=progress, longest_stationary_after_move_s=stall,
                               minimum_gap_m=gap['min_sampled_gap_m'], minimum_swept_gap_m=gap.get('min_swept_lower_bound_m'),
                               peak_speed_mps=max(r['ground_speed_mps'] for r in trace)),
                  original_passed=report['passed'], original_gates=evaluation['gates'], scans=rows,
                  scope='센서 반환과 평가 자세의 제한 비교. 상대 각본·공식 직선 한 번이며 실제 독립 상대/실차 보장 아님.')
    (HERE/'run1_goal.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'scans'}, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
