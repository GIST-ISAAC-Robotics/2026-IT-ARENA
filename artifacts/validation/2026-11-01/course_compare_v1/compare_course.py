"""대표 코스 기준과 후보를 같은 정의로 비교한다. 원래 판정 실패는 그대로 옮긴다(사후 평가).

한 랩 완료 시점과 155초 각본 종료를 구분한다. 속도는 고밀도 자세의 ±30 ms 중심 차분을 20 ms
격자에 놓은 값으로, 10 Hz 궤적 기반 수치와 직접 비교하지 않는다.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(BASE.parent / 'follow_continuity_v1'))
from slip_events import body_speed, events

LAP = json.loads((BASE.parent / 'signal_marker_offset_v1/fixture/scene.json').read_bytes())['track']['lap_length_m']


def transitions(run, green, end):
    out = []
    with (run / 'follow_decisions.jsonl').open('rb') as f:
        for line in f:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            d = e['data']
            s = d['input']['reference_s']
            state = d['output'][2].get('handoff_state')
            if green <= s <= end and (not out or out[-1][0] != state):
                out.append((state, round(s - green, 3)))
    return out


def bands(t, v, cuts):
    dt = np.diff(t, append=t[-1])
    rows = []
    for name, a, b in cuts:
        i = (t >= a) & (t < b)
        if not i.any():
            continue
        grid = t[i & (t >= a + .3)]
        worst = dict(mps=None, start_s=None)
        if len(grid):
            d = np.interp(grid - .3, t, v) - np.interp(grid, t, v)
            k = int(np.argmax(d))
            worst = dict(mps=max(0., float(d[k])), start_s=float(grid[k] - .3))
        rows.append(dict(name=name, window_s=[float(a), float(b)], mean_mps=float(np.sum(v[i] * dt[i]) / np.sum(dt[i])),
                         min_mps=float(np.min(v[i])), max_mps=float(np.max(v[i])), worst_drop_030=worst))
    return rows


def stalls(t, v, limit=.05):
    first = t[np.flatnonzero(v >= limit)[0]]
    out, i = [], 0
    while i < len(t):
        if t[i] >= first and v[i] < limit:
            j = i
            while j + 1 < len(t) and v[j + 1] < limit:
                j += 1
            out.append([round(float(t[i]), 2), round(float(t[j]), 2)])
            i = j + 1
        else:
            i += 1
    return out


def summary(run, surface, dense, audit, measure, command):
    r = json.loads((run / 'report.json').read_bytes())
    green, end = r['case_window']['green_sim_s'], r['case_window']['end_sim_s']
    duration = r['case_window']['duration_s']
    with (run / 'trajectory.jsonl').open(encoding='utf-8') as f:
        rows = [x for x in map(json.loads, f) if x.get('elapsed_since_green_s') is not None]
    tp = np.array([x['t'] - green for x in rows])
    progress = np.array([x['progress_m'] for x in rows], dtype=float)
    gap = np.array([x['sampled_gap_m'] if x['sampled_gap_m'] is not None else np.nan for x in rows], dtype=float)
    reasons = [str(x['safety_reason']) for x in rows if 0 <= x['t'] - green <= duration]
    t, v, _, _ = body_speed(run, green)
    keep = (t >= 0) & (t <= duration)
    t, v = t[keep], v[keep]
    done = np.flatnonzero(progress >= LAP)
    lap_s = float(np.interp(LAP, progress[:done[0] + 1], tp[:done[0] + 1])) if len(done) else None
    states = transitions(run, green, end)
    entry = next((s for n, s in states if n == 'BYPASS'), None)
    back = next((s for n, s in states if n == 'FOLLOW' and entry is not None and s > entry), None)
    cuts = [('출발–앞차 정지', 0., 4.), ('앞차 정지', 4., 9.)]
    if back is not None:
        cuts += [('재출발–복귀', 9., back)]
    last = lap_s or duration
    cuts += [('복귀–한 랩', back if back is not None else 9., last), ('한 랩 뒤–155초', last, duration)]
    wheels = json.loads(surface.read_bytes())['interpretations']['any']
    d = json.loads(dense.read_bytes())
    m = json.loads(measure.read_bytes())
    add = r['additional_actors']
    signal = r.get('team_signal_evidence') or {}
    window = (tp >= (back if back is not None else 9.)) & (tp <= last)
    return dict(
        run=run.relative_to(ROOT).as_posix(), reserve=r.get('handoff_reserve'), run_valid=r['run_valid'],
        harness_valid=r['harness_valid'], source_unchanged=r['source_unchanged'], shutdown_clean=r['shutdown_clean'],
        abort=r['abort'], passed=r['passed'], failed_gates=[k for k, x in r['gates'].items() if x is not True],
        stop_all_passed=all(r['stop_checks'].values()), stop_checks=r['stop_checks'],
        signal=dict(passed=signal.get('passed'), green_applied_sim_s=signal.get('green_applied_sim_s'),
                    first_start_sim_s=signal.get('first_start_sim_s'), start_after_actual_green=signal.get('start_after_actual_green')),
        lap_length_m=LAP, lap_complete_s=lap_s, duration_s=duration,
        progress_at_155_m=r['evaluation']['progress']['at_window_end_m'], full_lap=r['evaluation']['gates'].get('full_lap'),
        full_course_progress=r['evaluation'].get('full_course_progress'), states=states, bypass_entry_s=entry, follow_return_s=back,
        bypass_duration_s=None if entry is None or back is None else back - entry,
        final_gap_m=r['evaluation']['progress']['sampled_gap_at_window_end_m'], min_gap_m=r['evaluation']['gap']['min_sampled_gap_m'],
        min_swept_m=r['evaluation']['gap']['min_swept_lower_bound_m'], overlap_samples=r['evaluation']['gap']['sampled_overlap_samples'],
        contact_messages=r['evaluation']['gap']['actor_contact_messages_with_ego'], combined_contacts=add['combined_contacts'],
        side_actors={k: dict(min_gap_m=a['min_sampled_gap_m'], min_swept_m=a['min_swept_lower_bound_m']) for k, a in add['actors'].items()},
        wall_collision_samples=r['wall_collision_samples'], guard_samples={k: reasons.count(k) for k in sorted(set(reasons))},
        peak_body_mps=float(np.max(v)), below_005_s=float(np.sum(np.diff(t, append=t[-1])[v < .05])),
        below_020_s=float(np.sum(np.diff(t, append=t[-1])[v < .20])), stall_windows_s=stalls(t, v),
        bands=bands(t, v, cuts), mean_gap_after_return_to_lap_m=float(np.nanmean(gap[window])) if window.any() else None,
        wheel_body_events=events(run), max_nonprogress=d['max_nonprogress'], below_10s=d['below_10s'], dense_progress_m=d['progress_m'],
        projection_jumps=d['projection_jumps'], max_off_road_wheels=wheels['max_off_wheels'], penalty_events=wheels['penalty_events_merged'],
        longest_grass_s=wheels['longest_continuous_grass_contact_s'], off_time_by_count_s=wheels['time_by_off_wheel_count_s'],
        wall_wheel_samples=json.loads(surface.read_bytes())['wall_wheel_samples'], motor_audit_passed=json.loads(audit.read_bytes())['passed'],
        opponent_restart=m.get('opponent_restart'), nonprogress_10_31=m.get('nonprogress_10_31_definition'),
        command_time=json.loads(command.read_bytes())['categories'], wall_elapsed_s=r['wall_elapsed_s'],
        follow_decisions_sha256=r['follow_decisions']['sha256'])


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('out', type=Path)
    p.add_argument('--check-base-only', action='store_true')
    a = p.parse_args()
    M = BASE.parent / 'mixed_course_v1'
    base = summary(M / 'run1', M / 'run1_surface.json', M / 'run1_dense_progress.json', M / 'run1_motor_audit.json',
                   M / 'summary.json', M / 'command_time.json')
    result = dict(base=base, limits='각 설정 한 번씩의 시뮬레이션 실행. 궤적·스캔 위상이 달라 통계 우열이나 실차 성능을 증명하지 않는다.')
    if not a.check_base_only:
        result['candidate'] = summary(BASE / 'run_paced', BASE / 'run_paced_surface.json', BASE / 'run_paced_dense_progress.json',
                                      BASE / 'run_paced_motor_audit.json', BASE / 'run_paced_summary.json',
                                      BASE / 'run_paced_command_time.json')
    with a.out.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    for k in ('base', 'candidate'):
        x = result.get(k)
        if not x:
            continue
        print('==', k, x['reserve'], 'valid', x['run_valid'], 'src', x['source_unchanged'], 'shut', x['shutdown_clean'], 'stop', x['stop_all_passed'],
              'failed', x['failed_gates'], 'signal', x['signal'])
        print(f"  lap {x['lap_length_m']} at {x['lap_complete_s']} | prog155 {x['progress_at_155_m']:.3f} full {x['full_lap']} | entry {x['bypass_entry_s']} return {x['follow_return_s']} states {x['states']}")
        print(f"  gaps final {x['final_gap_m']:.3f} min {x['min_gap_m']:.3f} swept {x['min_swept_m']:.3f} overlap {x['overlap_samples']} contact {x['contact_messages']} side {x['side_actors']} wall {x['wall_collision_samples']} wallwheel {x['wall_wheel_samples']}")
        print(f"  wheels {x['max_off_road_wheels']} penalty {x['penalty_events']} grass {x['longest_grass_s']:.2f} bycount {x['off_time_by_count_s']}")
        print(f"  peak {x['peak_body_mps']:.3f} <.05 {x['below_005_s']:.2f} <.20 {x['below_020_s']:.2f} stalls {x['stall_windows_s']} 5cm {x['max_nonprogress']} <10s {x['below_10s']} guard {x['guard_samples']}")
        for b in x['bands']:
            print(f"  {b['name']} {b['window_s']} mean {b['mean_mps']:.3f} min {b['min_mps']:.3f} max {b['max_mps']:.3f} drop {b['worst_drop_030']}")
        print('  mismatch', [(round(e['start'], 2), round(e['end'], 2), round(e['decel_mps2'], 2)) for e in x['wheel_body_events']], 'audit', x['motor_audit_passed'])
        print('  cmd', {k2: round(v['seconds'], 2) for k2, v in x['command_time'].items()})
