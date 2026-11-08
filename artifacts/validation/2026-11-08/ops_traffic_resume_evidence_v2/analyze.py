"""ops_traffic_resume_v1 run1 사후 평가 보정. 평가 전용, 원시 자료만 읽는다.

v1 측정기 결함 두 가지를 원시 자료로 바로잡는다.
1. actor 시각: Gazebo Pose_V JSON에는 최상위 header가 없고 각 pose['header']['stamp']에 있다.
   proto3 JSON은 0인 sec/nsec를 생략하므로 생략값은 0으로 읽는다.
2. 평가 창: final_stop_hold가 없을 때 첫 STOP hold에서 잘린 창을 최초 출발~failure_stop_hold 끝으로 넓힌다.
원 판정(1.2 m 전진 실패)은 번복하지 않는다. 실행: python3 -B analyze.py (shapely 필요, WSL /usr/bin/python3)
"""
from pathlib import Path
import bisect, collections, hashlib, json, math, sys, time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
V1 = ROOT/'artifacts/validation/2026-11-08/ops_traffic_resume_v1'
RUN = V1/'run1'
sys.path.insert(0, str(ROOT/'scripts'))
from surface_contact_eval import SurfaceMap, evaluate_pose, rectangle  # noqa: E402

LOW_SPEED = .05          # m/s, r27 assess와 같은 정체 기준
SPEED_WINDOW = .05       # s, 보조 속도: 0.05 s 이상 떨어진 직전 표본까지의 변위/실제 시간
ACTOR_MAX_AGE = .05      # s, v1과 같은 actor 표본 최대 나이
STEERS = (0., -.45, .45) # v1과 같은 이산 조향 표본(rad), 실제 관절 기록 없음
EGO_L, EGO_W = .20, .15  # scripts/g1_traffic_eval.py EGO_LENGTH_M/EGO_WIDTH_M
read = lambda p: json.loads(Path(p).read_bytes())
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()


def steps_by_name(report):
    out = collections.defaultdict(list)
    for s in report['steps']:
        out[s['step']].append(s)
    return out


def actor_stream(path, model):
    rows, audit = [], dict(lines=0, parse_errors=0, top_level_header_lines=0, pose_entries=0,
                           names=collections.Counter(), frame_ids=collections.Counter(),
                           child_frame_ids=collections.Counter(), stamp_missing=0, non_increasing=0)
    for line in path.read_bytes().splitlines():
        if not line.strip():
            continue
        audit['lines'] += 1
        try:
            d = json.loads(line)
        except ValueError:
            audit['parse_errors'] += 1
            continue
        audit['top_level_header_lines'] += 'header' in d
        for p in d.get('pose', []):
            audit['pose_entries'] += 1
            audit['names'][p.get('name')] += 1
            header = p.get('header')
            if not isinstance(header, dict) or not isinstance(header.get('stamp'), dict):
                audit['stamp_missing'] += 1
                continue
            meta = {e.get('key'): (e.get('value') or [None])[0] for e in header.get('data', [])}
            audit['frame_ids'][meta.get('frame_id')] += 1
            audit['child_frame_ids'][meta.get('child_frame_id')] += 1
            if p.get('name') != model:
                continue
            st = header['stamp']
            t = int(st.get('sec', 0)) + int(st.get('nsec', 0))*1e-9
            q, pos = p['orientation'], p['position']
            yaw = math.atan2(2*(q.get('w', 1)*q.get('z', 0)+q.get('x', 0)*q.get('y', 0)),
                             1-2*(q.get('y', 0)**2+q.get('z', 0)**2))
            if rows and t <= rows[-1][0]:
                audit['non_increasing'] += 1
                continue
            rows.append((t, pos.get('x', 0.), pos.get('y', 0.), yaw))
    for k in ('names', 'frame_ids', 'child_frame_ids'):
        audit[k] = dict(audit[k])
    return rows, audit


def interval_stats(times):
    dt = [b-a for a, b in zip(times, times[1:])]
    c = collections.Counter(round(x*1000) for x in dt)
    s = sorted(dt)
    return dict(samples=len(times), first_s=times[0], last_s=times[-1], min_dt_s=s[0], median_dt_s=s[len(s)//2],
                max_dt_s=s[-1], non_positive=sum(x <= 0 for x in dt),
                dt_ms_histogram={str(k): v for k, v in sorted(c.items())})


def pair_speeds(rows):
    return [(a[0], b[0], math.dist(a[1:3], b[1:3])/(b[0]-a[0])) for a, b in zip(rows, rows[1:])]


def window_speeds(rows):
    """각 표본 b에서 SPEED_WINDOW 이상 앞선 가장 가까운 표본 a까지의 평균 속도. 평활 필터 없음."""
    times = [r[0] for r in rows]
    out = []
    for j, b in enumerate(rows):
        i = bisect.bisect_right(times, b[0]-SPEED_WINDOW)-1
        if i < 0:
            continue
        a = rows[i]
        out.append((a[0], b[0], math.dist(a[1:3], b[1:3])/(b[0]-a[0])))
    return out


def longest_low(speeds, t0, t1):
    """속도를 끝 표본 시각 b에 둔다. t0<b<=t1에서 연속 b가 LOW_SPEED 미만이면 b 간격을 누적(r27 assess 방식)."""
    best = run = 0.
    start = best_from = best_to = None
    prev = None
    for _, b, v in speeds:
        if b <= t0 or b > t1:
            continue
        if v < LOW_SPEED and prev is not None:
            if run == 0.:
                start = prev
            run += b-prev
            if run > best:
                best, best_from, best_to = run, start, b
        else:
            run = 0.
        prev = b
    return dict(longest_s=best, from_sim_s=best_from, to_sim_s=best_to)


def hold_stats(rows, t0, t1):
    w = [r for r in rows if t0 <= r[0] <= t1]
    dt = [b[0]-a[0] for a, b in zip(w, w[1:])]
    v = [math.dist(a[1:3], b[1:3])/d for a, b, d in zip(w, w[1:], dt)]
    return dict(from_sim_s=t0, to_sim_s=t1, samples=len(w), duration_s=w[-1][0]-w[0][0], peak_mps=max(v),
                max_gap_s=max(dt), drift_m=max(math.dist(w[0][1:3], r[1:3]) for r in w))


def main():
    started = time.monotonic()
    report, reg, ar = read(RUN/'report.json'), read(V1/'registered.json'), read(RUN/'actor_registered.json')
    st = steps_by_name(report)
    model = ar['actor']['model_name']
    t = lambda s: float(s['truth'][0])
    visual = t(st['visual_started'][0])
    first_press, first_stopped = t(st['press_stop'][0]), t(st['stopped'][0])
    first_hold = st['first_stop_hold'][0]
    before = st['before_resume'][0]['truth']
    resume_press = t(st['press_resume_start'][-1])
    closed = st['resume_closed'][0]
    permit = t(closed)
    fail_press, fail_stopped = t(st['press_stop'][1]), t(st['stopped'][1])
    fail_hold = st['failure_stop_hold'][0]
    end = fail_hold['to_sim_s']

    ego = [tuple(map(float, r[:4])) for r in (json.loads(l) for l in (RUN/'ego_poses.jsonl').read_bytes().splitlines() if l.strip())]
    actors, actor_audit = actor_stream(RUN/'actor_poses.jsonl', model)
    ego_all = interval_stats([r[0] for r in ego])
    origin = ego[0]
    depart = next(r[0] for r in ego if r[0] >= visual and math.dist(r[1:3], origin[1:3]) > .01)
    window = [r for r in ego if visual <= r[0] <= end]

    # actor 대조
    a_times = [a[0] for a in actors]
    a_stats = interval_stats(a_times)
    a_stats.update(x_range=[min(a[1] for a in actors), max(a[1] for a in actors)],
                   y_range=[min(a[2] for a in actors), max(a[2] for a in actors)],
                   registered_pose=ar['pose'],
                   max_offset_from_registered_m=max(math.dist(a[1:3], ar['pose'][:2]) for a in actors),
                   covers_ego_window=a_times[0] <= visual and a_times[-1] >= end - ACTOR_MAX_AGE)

    # 간격: 각 ego 표본에서 그 시각 이전 최신 actor 표본(나이 ≤ 0.05 s)
    alen, awid = ar['actor']['length_m'], ar['actor']['width_m']
    gaps, uncovered, max_age, overlap = [], 0, 0., 0
    for r in window:
        k = bisect.bisect_right(a_times, r[0])-1
        if k < 0 or r[0]-a_times[k] > ACTOR_MAX_AGE+1e-9:
            uncovered += 1
            continue
        a = actors[k]
        max_age = max(max_age, r[0]-a[0])
        e_poly, a_poly = rectangle(*r[1:4], EGO_L, EGO_W), rectangle(*a[1:4], alen, awid)
        d = e_poly.distance(a_poly)
        overlap += e_poly.intersects(a_poly)
        gaps.append((d, r[0]))
    step_travel = max(math.dist(a[1:3], b[1:3]) for a, b in zip(window, window[1:]))
    gmin = min(gaps)

    # 접촉: 원시 파일과 그래프 확인
    contact_bytes = (RUN/'contacts.jsonl').stat().st_size
    probe = st['contact_source'][0]

    # 노면: v1 assess와 같은 식(세 조향 표본, any 해석 최대, 벽 바퀴, 차체 밖). 동일 자세는 결과를 재사용(동등).
    surface = SurfaceMap.from_world(RUN/'runtime_world.sdf')
    cache, evaluated = {}, 0
    max_off = dict(any=0, center=0, full=0)
    wall_samples = body_out = 0
    off_intervals, current = [], None
    for r in window:
        key = r[1:4]
        if key not in cache:
            cache[key] = [evaluate_pose(surface, *key, s) for s in STEERS]
            evaluated += 1
        values = cache[key]
        for k in max_off:
            max_off[k] = max(max_off[k], *(v['off_counts'][k] for v in values))
        off_any = max(v['off_counts']['any'] for v in values)
        wall_samples += any(v['wall_wheels'] for v in values)
        body_out += values[0]['body_outside_road_area_m2'] > 1e-6
        if off_any:
            if current and r[0]-current['to_sim_s'] <= .01:
                current['to_sim_s'] = r[0]; current['max_any'] = max(current['max_any'], off_any)
            else:
                current = dict(from_sim_s=r[0], to_sim_s=r[0], max_any=off_any); off_intervals.append(current)

    # 종방향 전진: run.py와 같은 기준(before_resume truth 자세의 초기 방향 투영)
    ref = before
    proj = lambda r: (r[1]-ref[1])*math.cos(ref[3])+(r[2]-ref[2])*math.sin(ref[3])
    lat = lambda r: -(r[1]-ref[1])*math.sin(ref[3])+(r[2]-ref[2])*math.cos(ref[3])
    post = [r for r in ego if permit <= r[0] <= fail_press]
    best = max(post, key=proj)
    at_fail_press = [r for r in ego if r[0] <= fail_press][-1]
    last = [r for r in ego if r[0] <= end][-1]
    path_len = sum(math.dist(a[1:3], b[1:3]) for a, b in zip(post, post[1:]))

    raw_v, win_v = pair_speeds(ego), window_speeds(ego)
    stall = {}
    for name, sp in (('raw_pair_unsmoothed', raw_v), ('displacement_over_0p05s', win_v)):
        stall[name] = dict(
            after_permit_to_failure_stop=longest_low(sp, permit, fail_press),
            departure_to_first_stop=longest_low(sp, depart, first_press),
            departure_to_failure_stop_including_operator_stop=longest_low(sp, depart, fail_press))

    # 보조: 운영 표본(평가 전용 기록)의 허가 뒤 상태 분포, 시뮬 시간 가중
    states = collections.Counter()
    samples = [json.loads(l) for l in (RUN/'ops_log.jsonl').read_bytes().splitlines() if l.strip()]
    samples = [s for s in samples if s.get('row') == 'sample' and isinstance(s.get('sim_s'), (int, float))]
    for a, b in zip(samples, samples[1:]):
        if permit <= a['sim_s'] < fail_press and b['sim_s'] > a['sim_s']:
            states[f"{a.get('state')}|{a.get('reason')}|{a.get('safety_reason')}"] += min(b['sim_s'], fail_press)-a['sim_s']

    out = dict(
        scope='post-run evaluation of preserved ops_traffic_resume_v1 run1 raw data; no rerun, official 1.2 m failure preserved',
        inputs={p: sha(RUN/p) for p in ('report.json', 'ego_poses.jsonl', 'actor_poses.jsonl', 'contacts.jsonl',
                                         'runtime_world.sdf', 'actor_registered.json', 'ops_log.jsonl')}
               | {'../assessment.json': sha(V1/'assessment.json'), '../assess.py': sha(V1/'assess.py')},
        official=dict(report_passed=report['passed'], error=report['error'], v1_assessment_passed=read(V1/'assessment.json')['passed'],
                      progress_gate_m=1.2, retained='FAIL'),
        v1_measurement_defects=dict(
            actor_time='v1 read top-level header (absent in all lines) -> every stamp 0 -> 1 actor sample kept',
            window=f'no final_stop_hold -> v1 window ended at first_stop_hold {first_hold["to_sim_s"]} s (1603 ego samples); corrected window ends at failure_stop_hold {end} s'),
        timeline_sim_s=dict(visual_start=visual, first_departure_gt_0p01m=depart, first_stop_press=first_press,
            first_stopped=first_stopped, first_stop_hold=[first_hold['from_sim_s'], first_hold['to_sim_s']],
            resume_press=resume_press, resume_permitted=permit, resume_boundary_s=closed['last']['boundary_s'],
            resume_latency_sim_s=closed['sim_seconds'], failure_stop_press=fail_press, failure_stopped=fail_stopped,
            failure_stop_hold=[fail_hold['from_sim_s'], end], ego_last_sample=ego[-1][0]),
        ego_samples=dict(all=ego_all, evaluation_window=dict(from_sim_s=visual, to_sim_s=end, samples=len(window)),
                         pose_tuple='[t_sim_s, x, y, yaw, 5th field unused]'),
        actor=dict(model=model, audit=actor_audit, stream=a_stats),
        gap=dict(ego_rect_m=[EGO_L, EGO_W], actor_rect_m=[alen, awid], samples=len(gaps), uncovered_samples=uncovered,
                 actor_max_age_s=max_age, min_sample_gap_m=gmin[0], min_gap_at_sim_s=gmin[1], overlap_samples=overlap,
                 max_ego_travel_between_samples_m=step_travel,
                 conservative_lower_bound_m=gmin[0]-step_travel,
                 method='discrete samples at ego rate; actor static; lower bound subtracts max ego travel between samples'),
        contact=dict(contacts_jsonl_bytes=contact_bytes, graph_probe_returncode=probe['returncode'],
                     publisher_present='gz.msgs.Contacts' in probe['output'], reader_alive=probe['reader_alive_at_graph_audit'],
                     note='topic emits only on touch; empty file plus live publisher/reader means no contact message recorded'),
        surface=dict(method='v1 formula: evaluate_pose at every window ego sample for steering 0,-0.45,+0.45 rad; identical poses reuse result (equivalent)',
                     window_samples=len(window), unique_poses_evaluated=evaluated, max_off_wheels=max_off,
                     wall_wheel_samples=wall_samples, body_outside_samples=body_out, any_off_intervals=off_intervals[:50],
                     steering_scope='discrete 3-angle samples, no measured steering joint trace'),
        progress=dict(reference='before_resume truth pose (same as run.py at)', reference_pose=ref[:4],
                      evaluated_from_permit_to_failure_stop_press=[permit, fail_press],
                      max_longitudinal_m=proj(best), max_at_sim_s=best[0],
                      at_failure_stop_press_m=proj(at_fail_press), at_failure_hold_end_m=proj(last),
                      lateral_at_failure_hold_end_m=lat(last), planar_path_length_m=path_len, gate_m=1.2,
                      gate_met=proj(best) >= 1.2),
        stall=dict(threshold_mps=LOW_SPEED, methods=dict(
            raw_pair_unsmoothed='|p(k+1)-p(k)|/(t(k+1)-t(k)) for consecutive ego samples (1-3 ms), no filtering',
            displacement_over_0p05s='|p(k)-p(j)|/(t(k)-t(j)), j = latest sample at least 0.05 s earlier; no other filtering'),
            results=stall),
        stop_holds=dict(first=hold_stats(ego, first_hold['from_sim_s'], first_hold['to_sim_s']) | dict(from_stop_request_s=first_hold['from_stop_request_s']),
                        failure_recovery=hold_stats(ego, fail_hold['from_sim_s'], end) | dict(from_stop_request_s=fail_hold['from_stop_request_s']),
                        note='failure stop is the recovery STOP after the progress timeout, not the planned final_stop_hold'),
        ops_state_after_permit_sim_s=dict(sorted(states.items(), key=lambda kv: -kv[1])[:12]),
        runtime_s=time.monotonic()-started)
    (HERE/'evidence.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(progress=out['progress'], stall=stall, gap_min=gmin, max_off=max_off,
                          actor=a_stats['samples'], runtime=out['runtime_s']), ensure_ascii=False))


if __name__ == '__main__':
    main()
