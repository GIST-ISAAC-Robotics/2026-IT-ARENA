"""C1 A/B 쌍 원본 증거 감사(읽기 전용). 사용: python -I audit_pair_evidence.py

1) 판단 호출 스캔이 취득 기록에 없는 경우를 반올림·기록 범위·출처 불일치 중 무엇인지 원시 시각으로 대조한다.
2) 추가 actor 접촉의 첫 시각·사건 묶음·표본 수와 정체 시작·자차/상대 이동 순서를 원시 평가 로그로 대조한다.
원 report·assessment·pair_comparison은 읽기만 하고 판정선은 바꾸지 않는다. 결과는 audit/pair_evidence_audit.json.
"""
import bisect, hashlib, json, math
from pathlib import Path

HERE = Path(__file__).resolve().parent
PAIR = HERE.parent
FOLDERS = {'A': 'A_no_injection', 'B': 'B_c1_injection'}
EPISODE_GAP_S = 0.1  # 접촉 메시지 시각 간격이 이보다 크면 다른 사건으로 묶는다(기록용, 판정선 아님).
read = lambda p: json.loads(p.read_bytes())
jsonl = lambda p: [json.loads(l) for l in p.read_text(encoding='utf-8').splitlines() if l.strip()]
nonfinite = lambda v: not isinstance(v, (int, float)) or not math.isfinite(v)
stamp_of = lambda scan: round(scan['sec']+scan['nanosec']*1e-9, 6)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def recording_boundary(run, report):
    acquisition = read(run/'acquisition.json')
    received = {round(r['stamp_s'], 6): r for r in jsonl(run/'scans_received.jsonl')}
    acq = {round(r['first_ray_stamp_s'], 6): r for r in acquisition}
    acq_sorted = sorted(acq)
    calls = [(row['seq'], row['data']['input']) for row in jsonl(run/'follow_decisions.jsonl') if row['kind'] == 'call']
    # 판단 단계의 고정 차폐 광선: 수신 원본은 유한인데 판단 입력만 비유한인 광선 색인의 합집합
    mask = set()
    for _, inp in calls:
        stamp = stamp_of(inp['scan'])
        if stamp in received:
            mask |= {i for i, (x, y) in enumerate(zip(inp['scan']['ranges'], received[stamp]['ranges'])) if nonfinite(x) and not nonfinite(y)}
    headers, matched_model_ok, first_ref, prev_scan_last_ref = set(), True, {}, {}
    missing = {}
    last_stamp = None
    for seq, inp in calls:
        scan, ref = inp['scan'], inp['reference_s']
        stamp = stamp_of(scan)
        headers.add(tuple((k, v) for k, v in scan.items() if k not in ('sec', 'nanosec', 'ranges')))
        first_ref.setdefault(stamp, ref)
        if last_stamp is not None and stamp != last_stamp:
            prev_scan_last_ref[stamp] = last_ref
        last_stamp, last_ref = stamp, ref
        decided = {i for i, v in enumerate(scan['ranges']) if nonfinite(v)}
        if stamp in received:
            rec = received[stamp]['ranges']
            same_finite = all(x == y for x, y in zip(scan['ranges'], rec) if not nonfinite(x) and not nonfinite(y))
            matched_model_ok &= same_finite and decided == ({i for i, v in enumerate(rec) if nonfinite(v)} | mask)
        if stamp not in acq:
            missing.setdefault(stamp, []).append(dict(seq=seq, reference_s=ref, timing_verified=inp.get('timing_verified'),
                nonfinite_rays=len(decided), contains_fixed_mask=None))
            missing[stamp][-1]['_decided'] = decided
    for rows in missing.values():
        for row in rows:
            row['contains_fixed_mask'] = bool(mask) and mask <= row.pop('_decided')
    latency = sorted(round(first_ref[s]-acq[s].get('release_source_stamp_s', acq[s]['source_stamp_at_publish_s']), 4)
                     for s in first_ref if s in acq)
    last = acquisition[-1]
    tail = report.get('actor_tail_drain') or {}
    out = dict(
        decision_calls=len(calls), unique_decision_scans=len(first_ref),
        acquisition_records=len(acquisition), received_scans=len(received),
        acquisition_first_ray_range_s=[acquisition[0]['first_ray_stamp_s'], last['first_ray_stamp_s']],
        last_acquisition=dict(first_ray_stamp_s=last['first_ray_stamp_s'], completion_or_publish_s=last['source_stamp_at_publish_s'],
                              release_source_stamp_s=last.get('release_source_stamp_s')),
        first_ray_interval_s=sorted({round(b['first_ray_stamp_s']-a['first_ray_stamp_s'], 4) for a, b in zip(acquisition, acquisition[1:])}),
        stop_request_sim_s=report.get('stop_request_sim_s'), case_end_sim_s=report['case_window']['end_sim_s'],
        observation_last_ego_sim_s=report.get('observation_last_ego_sim_s'),
        evaluator_tail_drain_after_sim_s=tail.get('after_sim_s'),
        last_decision_reference_s=calls[-1][1]['reference_s'] if calls else None,
        decision_header_variants=len(headers),
        fixed_decision_mask_rays=sorted(mask),
        matched_scans_decision_equals_received_plus_fixed_mask=matched_model_ok,
        first_decision_minus_release_s=dict(min=latency[0], max=latency[-1], median=latency[len(latency)//2]) if latency else None,
        missing=[])
    for stamp, rows in sorted(missing.items()):
        i = bisect.bisect_left(acq_sorted, stamp)
        neighbours = acq_sorted[max(0, i-1):i+1]
        out['missing'].append(dict(
            first_ray_stamp_s=stamp, decision_calls=len(rows), calls=rows,
            nearest_acquisition_first_ray_s=min(neighbours, key=lambda s: abs(s-stamp)) if neighbours else None,
            nearest_abs_difference_s=min((abs(s-stamp) for s in neighbours), default=None),
            after_last_acquisition=stamp > acq_sorted[-1],
            continues_first_ray_cadence=abs(stamp-acq_sorted[-1]-0.1) < 1e-6,
            in_received=stamp in received,
            release_upper_bound_s=rows[0]['reference_s'],
            release_lower_bound_exclusive_s=prev_scan_last_ref.get(stamp),
            scope='release bounds from the first decision using this scan and the last decision still using the previous scan'))
    return out


def episodes(times):
    groups = []
    for t in times:
        if groups and t-groups[-1][-1] <= EPISODE_GAP_S+1e-9:
            groups[-1].append(t)
        else:
            groups.append([t])
    return [dict(first_s=g[0], last_s=g[-1], duration_s=round(g[-1]-g[0], 6), messages=len(g)) for g in groups]


_TIMES = {}


def nearest(rows, t):
    times = _TIMES.setdefault(id(rows), [r[0] for r in rows])
    i = bisect.bisect_left(times, t)
    return min(rows[max(0, i-1):i+1], key=lambda r: abs(r[0]-t))


def displacement(rows, t0, t1):
    a, b = nearest(rows, t0), nearest(rows, t1)
    return math.hypot(b[1]-a[1], b[2]-a[2])


def in_ego_frame(ego, other):
    dx, dy = other[1]-ego[1], other[2]-ego[2]
    c, s = math.cos(ego[3]), math.sin(ego[3])
    return dict(forward_m=round(c*dx+s*dy, 4), left_m=round(-s*dx+c*dy, 4), center_distance_m=round(math.hypot(dx, dy), 4),
                heading_difference_rad=round(math.remainder(other[3]-ego[3], math.tau), 4))


def contact_and_stall(run, report):
    extra = read(run/'additional_actors.json')
    ego = [tuple(r) for r in jsonl(run/'ego_poses.jsonl')]
    lead = [tuple(r) for r in jsonl(run/'actor_poses.jsonl')]
    t0, end = report['case_window']['green_sim_s'], report['case_window']['end_sim_s']
    trace = [r for r in read(run/'trace.json') if t0 <= r['t'] <= end]
    start = next((r['t'] for r in trace if r['ground_speed_mps'] > .10), None)
    duration, longest, run_start, best = 0., 0., None, (None, None)
    for a, b in zip(trace, trace[1:]):  # assess.py와 같은 저속(<0.05 m/s) 구간 규칙
        if start is None or a['t'] < start:
            continue
        if a['ground_speed_mps'] < .05:
            run_start = a['t'] if duration == 0. else run_start
            duration += b['t']-a['t']
            if duration > longest:
                longest, best = duration, (run_start, b['t'])
        else:
            duration = 0.
    stall_row = next((r for r in trace if best[0] is not None and r['t'] >= best[0]), None)
    out = dict(motion_start_s=start, longest_low_speed=dict(start_s=best[0], end_s=best[1], duration_s=longest),
               primary_actor_contact_lines=len((run/'actor_contacts.jsonl').read_text(encoding='utf-8').splitlines()),
               primary_actor_raw_gate_no_actor_contact=report['gates'].get('no_actor_contact'))
    if stall_row:
        auto = stall_row.get('autonomy') or {}
        out['trace_at_stall_start'] = dict(t=stall_row['t'], s=stall_row['s'], progress_m=stall_row['progress_m'],
            state=auto.get('state'), behavior=auto.get('behavior'), reason=auto.get('reason'),
            drive_speed_mps=stall_row['drive_speed_mps'], safe_speed_mps=stall_row['safe_speed_mps'],
            safety_reason=stall_row['safety_reason'], primary_sampled_gap_m=stall_row['sampled_gap_m'],
            scope='recorded state only; cause analysis is out of this audit')
        out['ego_displacement_stall_to_case_end_m'] = round(displacement(ego, best[0], end), 4)
        out['primary_actor_in_ego_frame_at_stall_start'] = in_ego_frame(nearest(ego, best[0]), nearest(lead, best[0]))
    actors = {}
    for name, poses in extra['poses'].items():
        poses = [tuple(p) for p in poses]
        cmds = extra['commands'][name]
        speeds = sorted({round(math.hypot(b['x_m']-a['x_m'], b['y_m']-a['y_m'])/(b['elapsed_s']-a['elapsed_s']), 3)
                         for a, b in zip(cmds, cmds[1:]) if b['elapsed_s']-a['elapsed_s'] > 1e-6})
        events = [e for e in report['additional_actors']['combined_contacts']['events'] if name in e['sources']]
        raw = [m for m in extra['contacts'] if m['actor'] == name]
        times = sorted({m['t'] for m in raw})
        info = dict(command_count=len(cmds), command_speed_mps_set=[speeds[0], speeds[-1]] if speeds else None,
                    command_speed_distinct=len(speeds),
                    commands_all_acknowledged=all(c.get('acknowledged') for c in cmds),
                    command_elapsed_range_s=[cmds[0]['elapsed_s'], cmds[-1]['elapsed_s']] if cmds else None,
                    min_center_distance_to_ego_m=None,
                    contact=dict(raw_messages=len(raw), unique_message_times=len(times), pair_rows=len(events),
                                 collision_pairs=sorted({' / '.join(sorted(e['pair'])) for e in events}),
                                 episodes=episodes(times)))
        dist = [(p[0], math.hypot(p[1]-q[1], p[2]-q[2])) for p in poses if t0 <= p[0] <= end for q in [nearest(ego, p[0])]]
        if dist:
            tmin, dmin = min(dist, key=lambda d: d[1])
            info['min_center_distance_to_ego_m'] = dict(t=tmin, distance_m=round(dmin, 4))
        if times:
            first, last = times[0], times[-1]
            before = [d for d in dist if d[0] <= first-.5]
            info['min_center_distance_before_first_contact_minus_0_5s_m'] = round(min(d[1] for d in before), 4) if before else None
            info['at_last_contact'] = dict(t=last, actor_in_ego_frame=in_ego_frame(nearest(ego, last), nearest(poses, last)))
            info['at_first_contact'] = dict(t=first, actor_in_ego_frame=in_ego_frame(nearest(ego, first), nearest(poses, first)),
                actor_speed_prev_0_5s_mps=round(displacement(poses, first-.5, first)/.5, 4),
                ego_speed_prev_0_5s_mps=round(displacement(ego, first-.5, first)/.5, 6))
            info['ego_displacement_during_contact_m'] = round(displacement(ego, first, last), 6)
            info['actor_displacement_during_contact_m'] = round(displacement(poses, first, last), 4)
            if best[0] is not None:
                info['order'] = dict(stall_start_s=best[0], first_contact_s=first, stall_precedes_contact_s=round(first-best[0], 3),
                    ego_displacement_stall_to_first_contact_m=round(displacement(ego, best[0], first), 6))
        actors[name] = info
    out['additional_actors'] = actors
    return out


def interpolation_difference(reference, other):
    keys = sorted(reference)
    worst = 0.
    for e, (x, y) in other.items():
        i = bisect.bisect_left(keys, e)
        if i == 0 or i == len(keys):
            continue
        a, b = keys[i-1], keys[i]
        w = (e-a)/(b-a)
        ix = reference[a][0]+w*(reference[b][0]-reference[a][0]); iy = reference[a][1]+w*(reference[b][1]-reference[a][1])
        worst = max(worst, math.hypot(ix-x, iy-y))
    return worst


def main():
    overlay = read(PAIR/'actor_overlay.json')
    result = dict(scope='read-only audit of completed C1 A/B raw evidence; original verdicts unchanged',
                  episode_gap_s=EPISODE_GAP_S, overlay=dict(sha256=sha(PAIR/'actor_overlay.json'),
                  actors=overlay['additional_actors']), runs={})
    for key, folder in FOLDERS.items():
        run = PAIR/folder/'run1'
        report = read(run/'report.json')
        result['runs'][key] = dict(
            inputs_sha256={n: sha(run/n) for n in ('report.json', 'acquisition.json', 'scans_received.jsonl', 'follow_decisions.jsonl',
                                                   'additional_actors.json', 'ego_poses.jsonl', 'actor_poses.jsonl', 'trace.json')},
            raw_pass=report.get('passed'), overlay_route=report['actor_overlay']['routes'],
            recording_boundary=recording_boundary(run, report), contact_and_stall=contact_and_stall(run, report))
    # 각본 성격: 같은 명령 경과시각에서 A/B 추가 actor 명령 위치가 같으면 자차 거동과 무관한 사전 일정이다.
    commands = {k: {round(c['elapsed_s'], 3): (c['x_m'], c['y_m']) for c in read(PAIR/f/'run1'/'additional_actors.json')['commands']['eval_second_lead']}
                for k, f in FOLDERS.items()}
    common = sorted(set(commands['A']) & set(commands['B']))
    result['actor_script_ab_command_match'] = dict(common_elapsed_samples=len(common),
        only_A=len(set(commands['A'])-set(commands['B'])), only_B=len(set(commands['B'])-set(commands['A'])),
        max_position_difference_m=max((math.hypot(commands['A'][e][0]-commands['B'][e][0], commands['A'][e][1]-commands['B'][e][1]) for e in common), default=None),
        common_elapsed_span_s=[common[0], common[-1]] if common else None,
        b_vs_a_linear_interpolation_max_m=interpolation_difference(commands['A'], commands['B']),
        scope='A/B command positions at equal elapsed_s and B against linearly interpolated A; agreement means a pre-scheduled kinematic path independent of ego motion')
    (HERE/'pair_evidence_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False)[:200])


if __name__ == '__main__':
    main()
