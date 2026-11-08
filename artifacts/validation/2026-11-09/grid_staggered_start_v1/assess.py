"""차례 출발 5대 그리드 보충 판정. 원 report 판정은 고치지 않고 함께 보존한다.

사용: assess.py                       (이 폴더 run1 → assessment_run1.json)
      assess.py --selftest RUN SURFACE INTEGRITY PROCESSES GEOMETRY OUT   (다른 실행에 읽기 전용으로 시험)
초기 공식 배치 간격은 기록만 하고, 자차가 처음 움직인 뒤 간격을 80 mm로 판정한다.
"""
import itertools
import json
from pathlib import Path
import sys

import g1_traffic_eval as ev
from g1_traffic_eval import rectangle

HERE = Path(__file__).resolve().parent
STARTS = {'eval_grid_2': .20, 'eval_grid_1': .80, 'eval_grid_0': 1.40, 'eval_g1b_actor': 2.00}
START_TOLERANCE_S = .10
MOTION_THRESHOLD_M = .005
INITIAL_POSE_TOLERANCE_M = .001
read = lambda p: json.loads(Path(p).read_bytes())
rows = lambda p: [json.loads(line) for line in Path(p).read_bytes().splitlines() if line.strip()]


def first_motion(poses, green):
    x0, y0 = poses[0][1], poses[0][2]
    for t, x, y, _ in poses:
        if ((x-x0)**2+(y-y0)**2)**.5 >= MOTION_THRESHOLD_M:
            return t-green
    return None


def pair_gaps(actors, specs, first, end):
    """같은 기록 시각(1 ms 반올림)의 상대끼리 외곽 간격. 접촉 센서와 별도인 기하 근거다."""
    keyed = {n: {round(p[0], 3): p for p in poses if first <= p[0] <= end} for n, poses in actors.items()}
    out = {}
    for a, b in itertools.combinations(sorted(actors), 2):
        common = sorted(set(keyed[a]) & set(keyed[b]))
        gaps, overlaps = [], 0
        for t in common:
            pa, pb = keyed[a][t], keyed[b][t]
            ra = rectangle(pa[1], pa[2], pa[3], specs[a][0], specs[a][1])
            rb = rectangle(pb[1], pb[2], pb[3], specs[b][0], specs[b][1])
            overlaps += ra.intersection(rb).area > 1e-9
            gaps.append(ra.distance(rb))
        expected = min(len(keyed[a]), len(keyed[b]))
        out[f'{a}|{b}'] = dict(samples=len(common), expected_min=expected, minimum_gap_m=min(gaps, default=None),
                               overlap_samples=overlaps, complete=bool(common) and len(common) >= .95*expected)
    return out


def main(run, surface_path, integrity_path, processes_path, geometry_path, out_path, runtime_path):
    r = read(run/'report.json'); s = read(surface_path)
    first, end = (r['case_window'][k] for k in ('green_sim_s', 'end_sim_s'))
    tr = [row for row in read(run/'trace.json') if first <= row['t'] <= end]
    moving = next((row['t'] for row in tr if row['ground_speed_mps'] >= .10), None)
    raw = read(run/'additional_actors.json')
    actors = {**raw['poses'], 'eval_g1b_actor': rows(run/'actor_poses.jsonl')}
    ego = rows(run/'ego_poses.jsonl')
    geometry = read(geometry_path)
    by_actor, timing, initial = {}, {}, {}
    for name, poses in actors.items():
        span = ev.window_tuples(ego, moving or end, end)
        sampled = ev.sampled_gaps(span, poses, .20, .15, ev.POSE_MAX_GAP_S)
        swept = ev.swept_gap_lower_bounds(span, poses, .20, .15, .01, 3., ev.POSE_MAX_GAP_S)
        vals = [v['gap_m'] for v in sampled if v['gap_m'] is not None]
        lows = [v['lower_bound_m'] for v in swept if v['lower_bound_m'] is not None]
        by_actor[name] = dict(samples=len(sampled), intervals=len(swept), minimum_gap_m=min(vals, default=None),
                              minimum_swept_gap_m=min(lows, default=None), complete=bool(vals and lows)
                              and len(vals) == len(sampled) and len(lows) == len(swept))
        started = first_motion([p for p in poses if p[0] >= first-.05], first)
        timing[name] = dict(registered_start_s=STARTS.get(name), observed_first_motion_s=started,
                            error_s=None if started is None or name not in STARTS else started-STARTS[name])
        slot_pose = geometry['placements'][name]['pose']
        offset = ((poses[0][1]-slot_pose[0])**2+(poses[0][2]-slot_pose[1])**2)**.5
        initial[name] = dict(slot=geometry['placements'][name]['slot'], first_recorded_offset_m=offset,
                             initial_official_ego_gap_m=geometry['placements'][name]['initial_ego_gap_m'])
    order = [n for n, _ in sorted(((n, v['observed_first_motion_s']) for n, v in timing.items()),
                                  key=lambda item: float('inf') if item[1] is None else item[1])]
    pairs = pair_gaps(actors, {n: (.20, .15) for n in actors}, first, end)
    span = longest = 0.
    for a, b in zip(tr, tr[1:]):
        if moving is None or a['t'] < moving:
            continue
        span = span+b['t']-a['t'] if a['ground_speed_mps'] < .05 else 0.
        longest = max(longest, span)
    extra = r['additional_actors']; gap = r['evaluation']['gap']; stop = r.get('stop_checks', {})
    integrity = read(integrity_path); contacts = extra.get('combined_contacts', {})
    signal = r.get('official_signal_evidence', {})
    harnesses = {n: v['harness']['valid'] for n, v in extra['actors'].items()}
    harnesses['eval_g1b_actor'] = r['evaluation']['harness']['valid']
    runtime = read(runtime_path) if runtime_path and Path(runtime_path).is_file() else {}
    gates = dict(
        actual_four_actors=len(actors) == 4 and extra.get('actor_count') == 3 and all(harnesses.values())
            and all(v['first_recorded_offset_m'] <= INITIAL_POSE_TOLERANCE_M for v in initial.values()),
        scheduled_starts=all(v['error_s'] is not None and abs(v['error_s']) <= START_TOLERANCE_S for v in timing.values())
            and order == sorted(STARTS, key=STARTS.get),
        runtime_valid=r.get('run_valid') is True and r.get('harness_valid') is True,
        rgb_first_start=signal.get('passed') is True and signal.get('start_after_actual_green') is True
            and r.get('false_start') is False,
        progress=bool(tr and tr[-1]['progress_m'] >= 2.), moving=moving is not None, no_long_stall=longest <= 2.5,
        no_contacts=gap.get('actor_contact_messages_with_ego') == 0 and contacts.get('valid') is True
            and not contacts.get('events') and not extra.get('contacts', {}).get('events'),
        no_overlap=gap.get('sampled_overlap_samples') == 0
            and all(v['complete'] and v['overlap_samples'] == 0 for v in pairs.values()),
        moving_clearance=all(v['complete'] and v['minimum_gap_m'] >= .08 and v['minimum_swept_gap_m'] >= .08
                             for v in by_actor.values()),
        surface=s.get('samples', 0) > 0 and s.get('sample_gaps_over_limit') == 0 and s.get('wall_wheel_samples') == 0
            and set(s['interpretations']) == {'any', 'center', 'full'}
            and all(v['max_off_wheels'] <= 2 for v in s['interpretations'].values()),
        stop_and_shutdown=bool(stop) and all(stop.values()) and r.get('shutdown_clean') is True,
        unchanged=r.get('source_unchanged') is True and not integrity['changed']
            and integrity.get('snapshot_unchanged', True) is True,
        process_clean=not Path(processes_path).read_text().strip(),
        runtime_provenance=runtime.get('ok') is True)
    primary = by_actor.get('eval_g1b_actor', {})
    out = dict(scope='staggered five-car official grid representative; supplementary gates only; raw report gates preserved; '
                     'RGB occlusion ratio not measured',
               gates=gates, passed=all(gates.values()), progress_m=tr[-1]['progress_m'] if tr else None,
               window_s=end-first, longest_low_speed_after_motion_s=longest,
               ego_first_motion_after_green_s=moving-first if moving else None,
               ego_first_motion_after_grid4_start_s=(moving-first-STARTS['eval_g1b_actor']) if moving else None,
               signal_first_start_sim_s=signal.get('first_start_sim_s'), signal_green_applied_sim_s=signal.get('green_applied_sim_s'),
               actor_start_timing=timing, observed_start_order=order,
               by_actor_after_first_motion=by_actor, primary_against_raw_150mm=dict(
                   minimum_gap_m=primary.get('minimum_gap_m'), minimum_swept_gap_m=primary.get('minimum_swept_gap_m'),
                   note='informational; raw report applies 0.15 m to the primary actor over the whole window'),
               actor_pair_gaps=pairs, initial_official_placement=initial,
               behavior=r['evaluation'].get('controller_mode'), raw_gates=r['gates'], raw_pass=r['passed'],
               raw_additional_actor_gates={n: v['gates'] for n, v in extra['actors'].items()},
               contact_note='actors are evaluator-teleported static models; actor-actor contact events may not be generated, so the recorded-pose pair geometry is the actor-actor evidence')
    Path(out_path).write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(dict(passed=out['passed'], gates=gates, progress_m=out['progress_m'],
                          ego_first_motion_after_green_s=out['ego_first_motion_after_green_s'], order=order,
                          raw_pass=out['raw_pass']), ensure_ascii=False))
    return 0 if out['passed'] else 1


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--selftest':
        run, surface, integrity, processes, geometry, out = map(Path, sys.argv[2:8])
        sys.exit(main(run, surface, integrity, processes, geometry, out, None))
    sys.exit(main(HERE/'run1', HERE/'surface_run1.json', HERE/'postflight_integrity_run1.json',
                  HERE/'postflight_processes_run1.txt', HERE/'geometry.json', HERE/'assessment_run1.json',
                  HERE/'runtime_check.json'))
