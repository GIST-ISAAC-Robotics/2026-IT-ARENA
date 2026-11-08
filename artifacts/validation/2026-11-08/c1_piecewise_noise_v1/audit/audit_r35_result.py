"""r35 C1 주입 B 결과의 읽기 전용 감사.

완료된 원본(report/assessment/integrity/trace/status/decision/acquisition)만 읽는다.
시뮬레이션·재생을 하지 않고 원본과 판정선을 바꾸지 않는다.
출력: audit/r35_result_audit.json
"""
import collections
import hashlib
import json
import math
import pathlib
import re
import statistics

HERE = pathlib.Path(__file__).resolve().parent
R35 = HERE.parent
R34 = R35.parent / 'g1g2_lidar_stress_pair_v1'
RUNS = {
    'r34_A': R34 / 'A_no_injection',
    'r34_B': R34 / 'B_c1_injection',
    'r35_B': R35 / 'B_c1_injection',
}
LOW, START = .05, .10  # assess.py r27_gates 와 같은 규칙
USED = {}


def path_used(p):
    p = pathlib.Path(p)
    if p not in USED:
        USED[p] = hashlib.sha256(p.read_bytes()).hexdigest()
    return p


def read_json(p):
    return json.loads(path_used(p).read_text(encoding='utf-8'))


def read_jsonl(p):
    with open(path_used(p), encoding='utf-8') as f:
        return [json.loads(line) for line in f if line.strip()]


def pct(values, q):
    v = sorted(values)
    if not v:
        return None
    k = (len(v) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return v[lo] + (v[hi] - v[lo]) * (k - lo)


def dist_summary(values):
    if not values:
        return None
    return dict(n=len(values), min=min(values), p50=pct(values, .5), p95=pct(values, .95),
                max=max(values), mean=statistics.fmean(values))


def ego_frame(ego, ax, ay):
    dx, dy = ax - ego['x'], ay - ego['y']
    c, s = math.cos(ego['yaw']), math.sin(ego['yaw'])
    return [round(c * dx + s * dy, 3), round(-s * dx + c * dy, 3)]


def nearest_pose(poses, t):
    # poses: [[t, x, y, yaw], ...] 정렬됨
    lo, hi = 0, len(poses) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if poses[mid][0] < t:
            lo = mid + 1
        else:
            hi = mid
    cand = [poses[i] for i in (lo - 1, lo) if 0 <= i < len(poses)]
    return min(cand, key=lambda p: abs(p[0] - t))


def low_speed_segments(tr, t0, end):
    tr = [row for row in tr if t0 <= row['t'] <= end]
    start = next((row['t'] for row in tr if row['ground_speed_mps'] > START), None)
    segs, cur = [], None
    for a, b in zip(tr, tr[1:]):
        if start is None or a['t'] < start:
            continue
        if a['ground_speed_mps'] < LOW:
            cur = [a['t'], b['t']] if cur is None else [cur[0], b['t']]
        elif cur:
            segs.append(cur)
            cur = None
    if cur:
        segs.append(cur)
    return start, sorted(segs, key=lambda s: s[1] - s[0], reverse=True)


def counts(values):
    return dict(collections.Counter(values).most_common())


def status_runs(st):
    """연속한 같은 (behavior, normal.reason, selector_reason) 행을 [첫 t, 끝 t, 행 수, ...]로 묶는다."""
    out = []
    for r in st:
        key = [r.get('behavior'), (r.get('normal') or {}).get('reason'), (r.get('normal') or {}).get('selector_reason')]
        t = round(r.get('sim_time_s'), 3)
        if out and out[-1][3:] == key:
            out[-1][1], out[-1][2] = t, out[-1][2] + 1
        else:
            out.append([t, t, 1] + key)
    return out


def describe_segment(run, seg, trace, status, second, window_end, stop_s):
    a, b = seg
    rows = [r for r in trace if a <= r['t'] <= b]
    idx = trace.index(rows[0])
    hold_start = a
    for r in reversed(trace[:idx + 1]):
        if (r['autonomy'] or {}).get('behavior') != 'HOLD':
            break
        hold_start = r['t']
    # 감속을 만든 직전 판단도 보도록 HOLD 시작(없으면 구간 시작) 0.5 s 전부터 본다.
    st = [r for r in status if hold_start - 0.5 <= (r.get('sim_time_s') or -1) <= b]
    lead = [r for r in trace if hold_start - 0.5 <= r['t'] < a]
    after = [r for r in trace if r['t'] > b]
    recovered = next((r for r in after if r['ground_speed_mps'] > START), None)
    ego0, ego1 = rows[0], rows[-1]
    out = dict(
        start_s=a, end_s=b, duration_s=b - a,
        end_kind='case_window_end' if abs(b - max(r['t'] for r in trace if r['t'] <= window_end)) < 1e-9 else 'speed_recovered',
        hold_episode_start_s=hold_start,
        progress_m=[ego0['progress_m'], ego1['progress_m'], ego1['progress_m'] - ego0['progress_m']],
        course_s_m=ego0['s'],
        ground_speed_mps=dict(mean=statistics.fmean(r['ground_speed_mps'] for r in rows), max=max(r['ground_speed_mps'] for r in rows)),
        drive_speed_mps_max=max((r['drive_speed_mps'] or 0) for r in rows),
        safe_speed_mps_max=max((r['safe_speed_mps'] or 0) for r in rows),
        controller_speed_command_mps_max=max(((r['autonomy'] or {}).get('speed_command_mps') or 0) for r in rows),
        trace_behavior_reason=counts(f"{(r['autonomy'] or {}).get('behavior')}/{(r['autonomy'] or {}).get('reason')}" for r in rows),
        safety_reason=counts(r['safety_reason'] for r in rows),
        safety_clearance_m=[min(r['safety_clearance_m'] for r in rows), max(r['safety_clearance_m'] for r in rows)],
        lead_in_0p5s=dict(
            trace_rows=len(lead),
            ground_speed_mps=[r['ground_speed_mps'] for r in lead],
            controller_speed_command_mps=[(r['autonomy'] or {}).get('speed_command_mps') for r in lead],
            safe_speed_mps=[r['safe_speed_mps'] for r in lead],
            behavior=[(r['autonomy'] or {}).get('behavior') for r in lead]),
        status_window_s=[hold_start - 0.5, b],
        status_rows=len(st),
        status_runs=status_runs(st),
        status_normal_reason=counts((r.get('normal') or {}).get('reason') for r in st),
        status_selector=counts('/'.join(str((r.get('normal') or {}).get(k)) for k in ('selector_mode', 'selector_choice', 'selector_reason')) for r in st),
        status_persistence_reason=counts((r.get('persistence') or {}).get('reason') for r in st),
        status_follow_entry_space=counts((r.get('follow_entry_space') or {}).get('reason') for r in st),
        status_preparation=counts((r.get('preparation') or {}).get('reason') for r in st),
        primary_actor_ego_xy_m=dict(start=ego_frame(ego0, ego0['actor_x'], ego0['actor_y']), end=ego_frame(ego1, ego1['actor_x'], ego1['actor_y'])),
        primary_sampled_gap_m=[ego0['sampled_gap_m'], ego1['sampled_gap_m']],
        recovery=dict(recovered=recovered is not None,
                      first_speed_over_0p10_s=recovered['t'] if recovered else None,
                      delay_after_segment_end_s=(recovered['t'] - b) if recovered else None),
    )
    if second:
        p0, p1 = nearest_pose(second, a), nearest_pose(second, b)
        out['second_actor_ego_xy_m'] = dict(start=ego_frame(ego0, p0[1], p0[2]), end=ego_frame(ego1, p1[1], p1[2]))
    patches = [(r.get('persistence') or {}).get('front_median_xy') for r in st]
    patches = [p for p in patches if p]
    if patches:
        out['persistence_front_median_xy'] = dict(
            n=len(patches), x=[min(p[0] for p in patches), statistics.median(p[0] for p in patches), max(p[0] for p in patches)],
            y=[min(p[1] for p in patches), statistics.median(p[1] for p in patches), max(p[1] for p in patches)])
    if out['end_kind'] == 'case_window_end':
        out['after_window_to_stop'] = dict(stop_request_s=stop_s, note='trace ends at the case window; see ego_after_window')
    return out


SCAN_RE = re.compile(r'"scan": \{"sec": (\d+), "nanosec": (\d+)|"scan":\{"sec":(\d+),"nanosec":(\d+)')
REF_RE = re.compile(r'"reference_s": ?([-0-9.eE]+)')
OUT_RE = re.compile(r'"output": ?\[([-0-9.eE]+),')


def decision_stream(p, t0, end, windows):
    """판단 기록을 한 줄씩 정규식으로 읽어 스캔 나이와 구간별 출력 속도를 모은다."""
    ages_all, ages_case, outs = [], [], {i: [] for i in range(len(windows))}
    calls = 0
    with open(path_used(p), encoding='utf-8') as f:
        for line in f:
            if '"kind": "call"' not in line and '"kind":"call"' not in line:
                continue
            calls += 1
            m, r = SCAN_RE.search(line), REF_RE.search(line)
            if not (m and r):
                continue
            g = [x for x in m.groups() if x is not None]
            stamp = int(g[0]) + int(g[1]) * 1e-9
            ref = float(r.group(1))
            age = ref - stamp
            ages_all.append(age)
            if t0 <= ref <= end:
                ages_case.append(age)
            o = OUT_RE.search(line)
            for i, (a, b) in enumerate(windows):
                if a <= ref <= b and o:
                    outs[i].append(float(o.group(1)))
    return calls, ages_all, ages_case, outs


def scan_cluster(p, t_target, patch_xy, lidar_x):
    """구간 중간 판단 입력 스캔에서 지속 patch 가장 가까운 광선의 연결 묶음 길이."""
    with open(path_used(p), encoding='utf-8') as f:
        best = None
        for line in f:
            if '"call"' not in line:
                continue
            r = REF_RE.search(line)
            if r and float(r.group(1)) >= t_target:
                best = json.loads(line)
                break
    if best is None:
        return None
    sc = best['data']['input']['scan']
    pts = []
    for i, v in enumerate(sc['ranges']):
        if isinstance(v, (int, float)) and math.isfinite(v):
            a = sc['angle_min'] + i * sc['angle_increment']
            pts.append((i, v * math.cos(a) + lidar_x, v * math.sin(a)))
        else:
            pts.append((i, None, None))
    valid = [q for q in pts if q[1] is not None]
    near = min(valid, key=lambda q: math.hypot(q[1] - patch_xy[0], q[2] - patch_xy[1]))
    n = len(pts)

    def grow(step):
        out, k, prev = [], near[0], near
        while True:
            k = (k + step) % n
            q = pts[k]
            if q[1] is None or math.hypot(q[1] - prev[1], q[2] - prev[2]) > .05 or len(out) > n:
                return out
            out.append(q)
            prev = q
    group = list(reversed(grow(-1))) + [near] + grow(1)
    ends = group[0], group[-1]

    def beyond(end, step):
        q = pts[(end[0] + step) % n]
        if q[1] is None:
            return dict(ray=q[0], kind='nonfinite')
        return dict(ray=q[0], kind='jump', xy=[round(q[1], 3), round(q[2], 3)],
                    step_m=round(math.hypot(q[1] - end[1], q[2] - end[2]), 3))
    return dict(reference_s=best['data']['input']['reference_s'], frame='lidar ray xy + lidar_x_m(0.06) offset; assumed vehicle frame, not verified',
                beyond_ends=[beyond(ends[0], -1), beyond(ends[1], 1)],
                group_xy=[[round(q[1], 3), round(q[2], 3)] for q in group],
                nearest_ray_index=near[0], nearest_ray_xy=[round(near[1], 3), round(near[2], 3)],
                nearest_ray_to_patch_m=math.hypot(near[1] - patch_xy[0], near[2] - patch_xy[1]),
                connected_rays=len(group), connected_span_m=math.hypot(ends[0][1] - ends[1][1], ends[0][2] - ends[1][2]),
                connected_end_xy=[[round(e[1], 3), round(e[2], 3)] for e in ends],
                rule='adjacent finite rays with point step <= 0.05 m')


def wall_footprint_distance(obj_path, ego, local_pts):
    """평가 전용: 월드 벽 시각 메시의 XY 투영 삼각형과 차량 좌표 점의 거리. 제어 입력이 아니다."""
    verts, tris = [], []
    for line in path_used(obj_path).read_text(encoding='utf-8').splitlines():
        if line.startswith('v '):
            verts.append(tuple(map(float, line.split()[1:3])))
        elif line.startswith('f '):
            tris.append([int(x.split('/')[0]) - 1 for x in line.split()[1:4]])

    def seg_d(p, a, b):
        ax, ay, bx, by = *a, *b
        L = (bx - ax) ** 2 + (by - ay) ** 2
        u = 0 if L == 0 else max(0, min(1, ((p[0] - ax) * (bx - ax) + (p[1] - ay) * (by - ay)) / L))
        return math.hypot(p[0] - ax - u * (bx - ax), p[1] - ay - u * (by - ay))

    def inside(p, a, b, c):
        def s(p1, p2, p3):
            return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])
        d1, d2, d3 = s(p, a, b), s(p, b, c), s(p, c, a)
        return not ((d1 < 0 or d2 < 0 or d3 < 0) and (d1 > 0 or d2 > 0 or d3 > 0))

    c, s = math.cos(ego[3]), math.sin(ego[3])
    out = []
    for lx, ly in local_pts:
        p = (ego[1] + c * lx - s * ly, ego[2] + s * lx + c * ly)
        d = min(0 if inside(p, *(verts[i] for i in t)) else
                min(seg_d(p, verts[t[0]], verts[t[1]]), seg_d(p, verts[t[1]], verts[t[2]]), seg_d(p, verts[t[2]], verts[t[0]]))
                for t in tris)
        out.append(dict(vehicle_xy=[round(lx, 3), round(ly, 3)], world_xy=[round(p[0], 3), round(p[1], 3)], wall_footprint_distance_m=round(d, 4)))
    return out


def injection_raw(run):
    acq = read_json(run / 'run1' / 'acquisition.json')
    with open(path_used(run / 'run1' / 'scans_received.jsonl'), encoding='utf-8') as f:
        received = sum(1 for line in f if line.strip())
    delays = [r['injected_delay_s'] for r in acq if 'injected_delay_s' in r]
    errs = [r['impairment']['max_abs_error_m'] for r in acq if 'impairment' in r]
    return dict(acquisition_records=len(acq), received_scans=received, injected_records=len(delays),
                injected_delay_s=dist_summary(delays) if delays else None,
                max_abs_error_m=max(errs) if errs else None)


def run_summary(key, run):
    r = read_json(run / 'run1' / 'report.json')
    s = read_json(run / 'assessment_run1.json')
    integ = read_json(run / 'postflight_integrity_run1.json')
    exit_line = path_used(run / 'run1.exit').read_text(encoding='utf-8').strip()
    post = path_used(run / 'postflight_processes_run1.txt').read_text(encoding='utf-8').strip()
    t0, end = r['case_window']['green_sim_s'], r['case_window']['end_sim_s']
    trace = read_json(run / 'run1' / 'trace.json')
    status = read_jsonl(run / 'run1' / 'autonomy_status.jsonl')
    start, segs = low_speed_segments(trace, t0, end)
    second = read_json(run / 'run1' / 'additional_actors.json')['poses'].get('eval_second_lead')
    keep = segs[:3] if key == 'r35_B' else segs[:1]
    segments = [describe_segment(run, sg, trace, status, second, end, r.get('stop_request_sim_s')) for sg in keep]
    proc = [((x.get('motion') or {}).get('processing_ms')) for x in status]
    proc_case = [p for x, p in zip(status, proc) if p is not None and t0 <= (x.get('sim_time_s') or -1) <= end]
    proc = [p for p in proc if p is not None]
    calls, ages_all, ages_case, outs = decision_stream(run / 'run1' / 'follow_decisions.jsonl', t0, end, keep)
    for sg, (i, vals) in zip(segments, outs.items()):
        sg['decision_calls'] = dict(n=len(vals), output_speed_positive=sum(v > 0 for v in vals),
                                    output_speed_max=max(vals) if vals else None)
    eg = r['evaluation']
    out = dict(
        exit_line=exit_line, report_passed=r.get('passed'),
        raw_failed_gates=[k for k, v in r.get('gates', {}).items() if v is False],
        supplementary_passed=s.get('passed'),
        supplementary_failed_gates=[k for k, v in s['gates'].items() if v is False],
        progress_m=s['course']['progress_m'], longest_low_speed_s=s['longest_low_speed_s'],
        first_motion_s=start, peak_mps=s['peak_mps'], min_gap_m=s['min_gap'], min_swept_gap_m=s['min_swept_gap'],
        sampled_gap_at_window_end_m=eg['progress'].get('sampled_gap_at_window_end_m'),
        actor_contact_messages_with_ego=eg['gap'].get('actor_contact_messages_with_ego'),
        additional_actor_contact_rows=len(r.get('additional_actors', {}).get('combined_contacts', {}).get('events') or []),
        behavior_counts=eg.get('controller_mode', {}).get('behavior_counts'),
        integrity=dict(inputs=integ.get('inputs'), changed=integ.get('changed'), snapshot_unchanged=integ.get('snapshot_unchanged'),
                       postflight_processes_empty=not post, source_unchanged=r.get('source_unchanged'),
                       sensor_only_inputs=r.get('integrity', {}).get('sensor_only_controller_inputs')),
        stop=dict(request_s=r.get('stop_request_sim_s'), stable=(r.get('stop') or {}).get('stable'),
                  checks=r.get('stop_checks'), mcu_state=((r.get('actuation') or {}).get('final_mcu') or {}).get('state'),
                  mcu_reason=((r.get('actuation') or {}).get('final_mcu') or {}).get('reason'),
                  shutdown_clean=r.get('shutdown_clean'), remaining_test_pids=r.get('remaining_test_pids')),
        lidar_stress_report=dict(verified=(r.get('lidar_stress_evidence') or {}).get('verified'),
                                 minimum_added_delay_s=(r.get('lidar_stress_evidence') or {}).get('minimum_added_delay_s'),
                                 maximum_added_delay_s=(r.get('lidar_stress_evidence') or {}).get('maximum_added_delay_s')),
        injection_raw=injection_raw(run),
        scan_age_assessment=s.get('scan_age_s'),
        scan_age_recomputed=dict(decision_calls=calls, all_calls=dist_summary(ages_all), case_window_calls=dist_summary(ages_case),
                                 rule='decision input.reference_s minus input.scan stamp (first-ray)'),
        processing_ms=dict(field='autonomy_status.jsonl motion.processing_ms', status_rows=len(status),
                           all=dist_summary(proc), case_window=dist_summary(proc_case),
                           scope='sampled status rows only, not every decision call; per-scan motion compensation time as recorded'),
        low_speed_segments_total=len(segs),
        low_speed_segments=segments,
    )
    return out


def ego_after_window(run):
    r = read_json(run / 'run1' / 'report.json')
    end, stop = r['case_window']['end_sim_s'], r.get('stop_request_sim_s')
    poses = []
    with open(path_used(run / 'run1' / 'ego_poses.jsonl'), encoding='utf-8') as f:
        for line in f:
            p = json.loads(line)
            if end - 1 <= p[0] <= stop:
                poses.append(p)
    a = next(p for p in poses if p[0] >= end - .042)
    b = poses[-1]
    return dict(from_s=a[0], to_s=b[0], displacement_m=math.hypot(b[1] - a[1], b[2] - a[2]))


def same_place(run, course_s, progress):
    """참고용: 다른 run(r34 A 무주입)이 같은 진행 위치 ±1 m를 지날 때의 기록. 같은 입력의 새 주행이 아니다."""
    trace = read_json(run / 'run1' / 'trace.json')
    status = read_jsonl(run / 'run1' / 'autonomy_status.jsonl')
    rows = [r for r in trace if progress - 1 <= r['progress_m'] <= progress + 1]
    t0, t1 = rows[0]['t'], rows[-1]['t']
    st = [r for r in status if t0 <= r['sim_time_s'] <= t1]
    patches = [(r['sim_time_s'], (r.get('persistence') or {}).get('front_median_xy')) for r in st]
    return dict(run=str(run.relative_to(R35.parent)).replace('\\', '/'), progress_window_m=[progress - 1, progress + 1],
                course_s_at_r35_stall=course_s, time_s=[t0, t1], min_ground_speed_mps=min(r['ground_speed_mps'] for r in rows),
                trace_behavior=counts((r['autonomy'] or {}).get('behavior') for r in rows),
                status_runs=status_runs(st),
                status_selector=counts('/'.join(str((r.get('normal') or {}).get(k)) for k in ('selector_mode', 'selector_choice', 'selector_reason')) for r in st),
                front_patches=[[round(t, 2), p and [round(v, 3) for v in p]] for t, p in patches],
                note='r34 A is a different (no-injection) run; actor-relative timing differs; context only')


def main():
    out = dict(scope='read-only audit of completed r35 B and r34 A/B; no simulation, no replay, gates unchanged',
               rule=dict(low_speed='trace ground_speed_mps < 0.05 after first > 0.10, within case window (assess.py r27_gates)'),
               runs={k: run_summary(k, v) for k, v in RUNS.items()})
    out['runs']['r35_B']['ego_after_window_to_stop'] = ego_after_window(RUNS['r35_B'])
    long = out['runs']['r35_B']['low_speed_segments'][0]
    if 'persistence_front_median_xy' in long:
        pm = long['persistence_front_median_xy']
        mid = (long['start_s'] + long['end_s']) / 2
        cl = scan_cluster(RUNS['r35_B'] / 'run1' / 'follow_decisions.jsonl', mid, (pm['x'][1], pm['y'][1]), .06)
        long['scan_cluster_at_patch'] = cl
        with open(path_used(RUNS['r35_B'] / 'run1' / 'ego_poses.jsonl'), encoding='utf-8') as f:
            ego = min((json.loads(line) for line in f if line.strip()), key=lambda p: abs(p[0] - cl['reference_s']))
        cl['evaluation_only_wall_check'] = dict(
            ego_pose=ego, scope='ground-truth wall visual mesh (batched_walls_0.obj) XY footprint, evaluation only; '
            'ego pose reference point vs vehicle frame origin not verified (few cm uncertainty)',
            points=wall_footprint_distance(RUNS['r35_B'] / 'run1' / 'batched_walls_0.obj', ego,
                                           [(pm['x'][1], pm['y'][1])] + [tuple(e) for e in cl['connected_end_xy']]))
    out['same_place_reference'] = same_place(RUNS['r34_A'], long['course_s_m'], long['progress_m'][0])
    out['inputs_sha256'] = {str(p.relative_to(R35.parent)).replace('\\', '/'): h for p, h in sorted(USED.items())}
    (HERE / 'r35_result_audit.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
    compact = []
    for s in out['runs']['r35_B']['low_speed_segments']:
        compact.append(dict(
            start_s=s['start_s'], end_s=s['end_s'], duration_s=round(s['duration_s'], 3), end_kind=s['end_kind'],
            hold_episode_start_s=s['hold_episode_start_s'], course_s_m=s['course_s_m'],
            progress_delta_m=round(s['progress_m'][2], 3), ground_speed_mean_max_mps=[round(v, 3) for v in s['ground_speed_mps'].values()],
            command_max_mps=dict(controller=s['controller_speed_command_mps_max'], drive=round(s['drive_speed_mps_max'], 3), safe=round(s['safe_speed_mps_max'], 3)),
            decision_calls=s['decision_calls'], dominant_normal_reason=next(iter(s['status_normal_reason']), None),
            selector=s['status_selector'], safety_reason=s['safety_reason'],
            primary_actor_ego_xy_m=s['primary_actor_ego_xy_m'], second_actor_ego_xy_m=s.get('second_actor_ego_xy_m'),
            front_patch_median_xy=s.get('persistence_front_median_xy') and [round(s['persistence_front_median_xy'][a][1], 3) for a in ('x', 'y')],
            recovery=s['recovery']))
    (HERE / 'r35_low_speed_compact.json').write_text(json.dumps(dict(
        source='r35_result_audit.json', rule=out['rule']['low_speed'], segments=compact,
        ego_after_window_to_stop=out['runs']['r35_B']['ego_after_window_to_stop']), ensure_ascii=False, indent=1),
        encoding='utf-8', newline='\n')
    for k, v in out['runs'].items():
        print(k, v['exit_line'], 'progress', round(v['progress_m'], 2), 'longest', round(v['longest_low_speed_s'], 3),
              'proc_p50', v['processing_ms']['all'] and round(v['processing_ms']['all']['p50'], 3))


if __name__ == '__main__':
    main()
