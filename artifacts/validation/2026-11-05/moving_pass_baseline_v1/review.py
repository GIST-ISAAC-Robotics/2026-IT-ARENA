"""run1 사후 평가(평가 전용). 정답 자세는 여기서만 쓰며 제어기 입력이 아니다.

사용: python -I -B review.py [run 디렉터리]  → review.json
표준 라이브러리만 쓴다. 등록 파일·원시 기록은 읽기만 한다.
"""
import bisect
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
RUN = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else HERE / 'run1'
BODY_L, BODY_W = 0.20, 0.15          # 자차·상대 외곽(등록 actor 상자와 같은 20×15 cm)
HIT_TOL_M = 0.05                     # 광선 끝점-예상 교점 허용 오차
RAY_ORDER = 'stamp + i*time_increment (sequential: stamp = first ray, rotating_lidar.py)'


def jl(name):
    with (RUN / name).open(encoding='utf-8') as f:
        return [json.loads(l) for l in f if l.strip()]


class Track:
    """[t, x, y, yaw] 시계열 선형 보간(yaw는 짧은 쪽)."""
    def __init__(self, rows):
        self.r = rows
        self.t = [r[0] for r in rows]

    def at(self, t):
        i = bisect.bisect_left(self.t, t)
        if i <= 0:
            return self.r[0][1:]
        if i >= len(self.r):
            return self.r[-1][1:]
        a, b = self.r[i - 1], self.r[i]
        w = (t - a[0]) / (b[0] - a[0]) if b[0] > a[0] else 0.
        dy = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        return a[1] + w * (b[1] - a[1]), a[2] + w * (b[2] - a[2]), a[3] + w * dy


def to_frame(px, py, ox, oy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - ox, py - oy
    return c * dx + s * dy, -s * dx + c * dy


def ray_box(ox, oy, dx, dy, hx, hy):
    """원점·방향(상자 좌표계)의 광선과 축정렬 상자 [-hx,hx]×[-hy,hy]의 첫 교점 거리."""
    tmin, tmax = -math.inf, math.inf
    for o, d, h in ((ox, dx, hx), (oy, dy, hy)):
        if abs(d) < 1e-12:
            if abs(o) > h:
                return None
            continue
        a, b = (-h - o) / d, (h - o) / d
        tmin, tmax = max(tmin, min(a, b)), min(tmax, max(a, b))
    return tmin if tmax >= tmin > 0 else None


def in_sectors(deg, sectors):
    return any(abs((deg - c + 180) % 360 - 180) <= hw for c, hw in sectors)


def stats(xs):
    xs = sorted(x for x in xs if x is not None)
    if not xs:
        return None
    return {'n': len(xs), 'min': xs[0], 'median': xs[len(xs) // 2], 'max': xs[-1], 'mean': sum(xs) / len(xs)}


def main():
    report = json.loads((RUN / 'report.json').read_text(encoding='utf-8'))
    course = json.loads((HERE / 'course.json').read_text(encoding='utf-8'))
    case = course['cases'][0]
    tg = report['case_window']['green_sim_s']
    te = report['case_window']['end_sim_s']
    stop_s = report['stop_request_sim_s']
    sectors = json.loads((RUN / 'lidar_blind_sectors.json').read_text(encoding='utf-8'))
    fd = jl('follow_decisions.jsonl')
    header = next(r for r in fd if r['kind'] == 'header')['data']
    mc = header['motion_config']
    lx, ly, lyaw = mc['lidar_x_m'], mc['lidar_y_m'], mc['lidar_yaw_rad']
    calls = [r['data'] for r in fd if r['kind'] == 'call']
    ego = Track(jl('ego_poses.jsonl'))
    act = Track(jl('actor_poses.jsonl'))
    traj = jl('trajectory.jsonl')

    # 1. 각본 추적(정답): 녹색 전 대기, 창 안 등속, 창 뒤 정지.
    def along(t0, t1, tr=act):
        a, b = tr.at(t0), tr.at(t1)
        return (b[0] - a[0]) * math.cos(a[2]) + (b[1] - a[1]) * math.sin(a[2])
    windows = [(tg + k, min(tg + k + 1., te)) for k in range(int(te - tg))]
    per_s = [along(a, b) / (b - a) for a, b in windows]
    pre = [r for r in act.r if r[0] < tg - 0.02]
    actor = {
        'schedule': case['schedule'], 'expected_travel_m': sum((s['to_s'] - s['from_s']) * s['speed_mps'] for s in case['schedule']),
        'travel_in_window_m': along(tg, te), 'mean_speed_mps': along(tg, te) / (te - tg),
        'per_second_speed_mps': stats(per_s),
        'pre_green_max_displacement_m': max(math.hypot(r[1] - pre[0][1], r[2] - pre[0][2]) for r in pre),
        'pre_green_static_s': pre[-1][0] - pre[0][0],
        'post_window_displacement_m': along(te + 0.05, act.r[-1][0]),
        'harness_max_moving_error_m': report['evaluation']['harness']['max_moving_error_m'],
        'interpretation': '녹색 전 약 7.4 s 정지 대기 → 녹색부터 14 s 등속 → 정지. 창 안에서는 계속 움직인 앞차다.',
    }

    # 2. 자차-상대 순서·간격·속도(정답, 상대 진행축 기준).
    def rel(t):
        e, a = ego.at(t), act.at(t)
        lon, lat = to_frame(e[0], e[1], a[0], a[1], a[2])
        return lon, lat
    samples, t = [], tg
    while t <= te + 1e-9:
        lon, lat = rel(t)
        e0, e1 = ego.at(t - 0.05), ego.at(t + 0.05)
        v = math.hypot(e1[0] - e0[0], e1[1] - e0[1]) / 0.1
        samples.append({'t': round(t, 3), 'center_lon_m': lon, 'lateral_m': lat,
                        'body_gap_m': -lon - BODY_L, 'ego_speed_mps': v})
        t += 0.05
    steady = [s for s in samples if s['t'] >= tg + 3.]
    ego_travel = along(tg, te, ego)
    vs = [s['ego_speed_mps'] for s in steady]
    dips, inside = [], False                 # 0.10 m/s 미만으로 떨어진 구간(정지 판정과 별개)
    for s in steady:
        if s['ego_speed_mps'] < 0.10 and not inside:
            dips.append({'from_s': s['t'], 'min_mps': s['ego_speed_mps'], 'samples': 0})
        if s['ego_speed_mps'] < 0.10:
            dips[-1]['min_mps'] = min(dips[-1]['min_mps'], s['ego_speed_mps'])
            dips[-1]['samples'] += 1
        inside = s['ego_speed_mps'] < 0.10
    order = {
        'ego_always_behind': all(s['center_lon_m'] < -BODY_L for s in samples),
        'max_center_lon_m': max(s['center_lon_m'] for s in samples),
        'body_gap_m': stats([s['body_gap_m'] for s in samples]),
        'body_gap_at_green_m': samples[0]['body_gap_m'], 'body_gap_at_window_end_m': samples[-1]['body_gap_m'],
        'lateral_offset_m': stats([abs(s['lateral_m']) for s in samples]),
        'ego_travel_in_window_m': ego_travel, 'ego_mean_speed_mps': ego_travel / (te - tg),
        'ego_speed_after_3s_mps': stats(vs),
        'ego_speed_oscillation_after_3s_mps': (max(vs) - min(vs)) if vs else None,
        'ego_speed_dips_below_0p10_after_3s': {'count': len(dips), 'samples_0p05s': sum(d['samples'] for d in dips), 'episodes': dips},
        'peak_ego_speed_mps': max(s['ego_speed_mps'] for s in samples),
        'peak_ego_speed_t': max(samples, key=lambda s: s['ego_speed_mps'])['t'],
        'per_second': [s for s in samples if abs((s['t'] - tg) - round(s['t'] - tg)) < 1e-6],
    }

    # 3. 명령·보호층(트레이스).
    w = [r for r in traj if tg <= r['t'] <= te]
    reasons = {}
    for r in w:
        reasons[r['safety_reason']] = reasons.get(r['safety_reason'], 0) + 1
    clipped = [r for r in w if r['safe_speed_mps'] < r['drive_speed_mps'] - 0.03]
    command = {
        'trajectory_rows_in_window': len(w), 'safety_reason_counts': reasons,
        'safe_below_drive_by_0p03_rows': len(clipped),
        'drive_speed_mps': stats([r['drive_speed_mps'] for r in w]),
        'safe_speed_mps': stats([r['safe_speed_mps'] for r in w]),
        'safety_clearance_m': stats([r['safety_clearance_m'] for r in w]),
        'mcu_states': sorted({r.get('mcu_state') for r in w if r.get('mcu_state')}),
        'note': 'trajectory는 약 0.1 s 간격 표본; safe<drive 차이에는 모터/명령 시차가 섞일 수 있다.',
    }

    # 4. 제어기 판단(follow_decisions, 센서 입력만으로 만든 값).
    seg, prev = [], None
    for c in calls:
        x = c['output'][2]
        key = (x['behavior'], x['normal']['behavior'], x['follow_entry_space'].get('reason'), x['preparation']['reason'], x['persistence']['reason'])
        if key != prev:
            seg.append({'from_s': c['input']['reference_s'], 'behavior': key[0], 'normal_behavior': key[1],
                        'follow_entry_space': key[2], 'preparation': key[3], 'persistence': key[4], 'calls': 0})
        seg[-1]['calls'] += 1
        prev = key
    win_calls = [c for c in calls if tg <= c['input']['reference_s'] <= te]
    counts = {}
    for c in win_calls:
        k = c['output'][2]['follow_entry_space'].get('reason')
        counts[k] = counts.get(k, 0) + 1
    no_patch = [c['input']['reference_s'] for c in win_calls if c['output'][2]['persistence']['reason'] == 'no_front_patch']
    rates = [c['output'][2]['preparation'].get('motion_rate_mps') for c in win_calls]
    patch_err, patch_lat = [], []
    for c in win_calls:
        fm = c['output'][2]['persistence'].get('front_median_xy')
        if not fm:
            continue
        ts = c['output'][2]['normal'].get('source_last_s') or c['input']['reference_s']
        e, a = ego.at(ts), act.at(ts)
        # 상대 뒷면 중심(정답)을 자차 기준(base, 모델 원점)으로.
        rx, ry = a[0] - BODY_L / 2 * math.cos(a[2]), a[1] - BODY_L / 2 * math.sin(a[2])
        bx, by = to_frame(rx, ry, e[0], e[1], e[2])
        patch_err.append(fm[0] - bx)
        patch_lat.append(fm[1] - by)
    first_patch = next((c['input']['reference_s'] for c in win_calls if c['output'][2]['persistence']['reason'] != 'no_front_patch'
                        and c['input']['reference_s'] > (no_patch[0] if no_patch else 0)), None)
    controller = {
        'calls_total': len(calls), 'calls_in_window': len(win_calls),
        'behavior_counts_in_window': {b: sum(c['output'][2]['behavior'] == b for c in win_calls) for b in ('FOLLOW', 'HOLD', 'CRUISE')},
        'follow_entry_space_reason_counts': counts,
        'no_front_patch_span_s': [no_patch[0], no_patch[-1]] if no_patch else None, 'no_front_patch_calls': len(no_patch),
        'first_patch_after_gap_s': first_patch,
        'speed_command_during_no_patch_mps': stats([c['output'][0] for c in win_calls if c['input']['reference_s'] in no_patch]),
        'normal_during_no_patch': [{'s': c['input']['reference_s'], 'v': c['output'][0], 'behavior': c['output'][2]['normal']['behavior'],
                                    'swept_clearance_m': c['output'][2]['normal'].get('swept_clearance_m'),
                                    'observed_path_ahead_m': c['output'][2]['normal'].get('observed_path_ahead_m'),
                                    'target_before_hold_mps': c['output'][2]['normal'].get('target_speed_before_hold_mps')}
                                   for c in win_calls if c['input']['reference_s'] in no_patch][::5],
        'motion_rate_mps': stats(rates), 'motion_rate_samples': sum(r is not None for r in rates),
        'patch_x_minus_truth_rear_face_m': stats(patch_err), 'patch_y_minus_truth_rear_face_m': stats(patch_lat),
        'patch_frame_basis': 'rear_axle_to_patch_m - front_median_x = 0.0675 (rear_axle_x_m) → base/model 원점 좌표로 해석',
        'identity_or_stationarity_proven_any': any(c['output'][2]['follow_entry_space'].get('identity_or_stationarity_proven') for c in win_calls),
        'moving_object_identity_proven_any': any(c['output'][2]['relative_yield']['current_observer'].get('moving_object_identity_proven') for c in win_calls),
        'persistence_ready_any_in_window': any(c['output'][2]['persistence']['ready'] for c in win_calls),
        'passing_enabled_any_in_window': any(c['output'][2]['normal'].get('passing_enabled') for c in win_calls),
        'planner_input_sectors_equal_runtime_file': all(c['input']['rear_half_angle_deg']['sectors_deg'] == sectors for c in calls),
        'planner_input_rear_half_angle_deg': sorted({c['input']['rear_half_angle_deg']['rear_half_angle_deg'] for c in calls}),
        'planner_known_structure_policy': sorted({c['output'][2]['normal']['partial_observation']['known_structure']['policy'] for c in calls}),
        'selector_side_seen_any':any(c['output'][2]['normal']['selector_sides'][s]['seen'] for c in win_calls for s in ('left', 'right')),
        'bypass_rows_total': sum('bypass' in c['output'][2] for c in calls),
        'bypass_first_reference_s': next((c['input']['reference_s'] for c in calls if 'bypass' in c['output'][2]), None),
        'speed_command_mps': stats([c['output'][0] for c in win_calls]),
        'segments': seg,
    }

    # 5. 실제 스캔에서 상대가 보였는가(정답 교점과 측정 거리 대조).
    scans = jl('scans_received.jsonl')
    per_scan = []
    for sc in scans:
        t0 = sc['stamp_s']
        if t0 < tg - 1. or t0 > stop_s:
            continue
        expected = hits = blocked = 0
        ranges_hit, bearings, base_xy = [], [], []
        for i, r in enumerate(sc['ranges']):
            ti = t0 + i * sc['time_increment']
            ang = sc['angle_min'] + i * sc['angle_increment']
            e, a = ego.at(ti), act.at(ti)
            ox = e[0] + lx * math.cos(e[2]) - ly * math.sin(e[2])
            oy = e[1] + lx * math.sin(e[2]) + ly * math.cos(e[2])
            th = e[2] + lyaw + ang
            bx, by = to_frame(ox, oy, a[0], a[1], a[2])
            dx, dy = math.cos(th - a[2]), math.sin(th - a[2])
            d = ray_box(bx, by, dx, dy, BODY_L / 2, BODY_W / 2)
            if d is None:
                continue
            deg = math.degrees(math.atan2(math.sin(ang), math.cos(ang)))
            if in_sectors(deg, sectors):
                blocked += 1
                continue
            expected += 1
            bearings.append(deg)
            if isinstance(r, (int, float)) and math.isfinite(r) and abs(r - d) <= HIT_TOL_M:
                hits += 1
                ranges_hit.append(r)
                # 보정 전 원시 거리의 base(모델 원점) 좌표 근사. front_patch 창 0.20≤x≤1.60, |y|≤0.24, 한 점 |y|<0.15.
                base_xy.append((lx + r * math.cos(ang + lyaw), ly + r * math.sin(ang + lyaw)))
        elig = [p for p in base_xy if 0.20 <= p[0] <= 1.60 and abs(p[1]) <= 0.24]
        per_scan.append({'t': t0, 'expected_rays': expected, 'hit_rays': hits, 'structure_blocked_rays': blocked,
                         'bearing_deg': [min(bearings), max(bearings)] if bearings else None,
                         'min_hit_range_m': min(ranges_hit) if ranges_hit else None,
                         'hit_base_x_m': [min(p[0] for p in base_xy), max(p[0] for p in base_xy)] if base_xy else None,
                         'hit_base_y_m': [min(p[1] for p in base_xy), max(p[1] for p in base_xy)] if base_xy else None,
                         'front_patch_window_hits': len(elig),
                         'front_patch_window_has_abs_y_lt_0p15': any(abs(p[1]) < 0.15 for p in elig)})
    inwin = [s for s in per_scan if tg <= s['t'] <= te]
    gap_scans = [s for s in per_scan if controller['no_front_patch_span_s'] and
                 controller['no_front_patch_span_s'][0] - 0.2 <= s['t'] <= controller['no_front_patch_span_s'][1]]
    visibility = {
        'method': f'광선별 시각 {RAY_ORDER}; 정답 자세 보간으로 20×15 cm 상대 상자와 교점, 측정 거리 차 ≤ {HIT_TOL_M} m를 적중으로 센다. '
                  '구조 각도(lidar_blind_sectors.json, (중심°, 반폭°))에 든 광선은 차폐로 분리.',
        'scans_in_window': len(inwin), 'scans_with_expected_rays': sum(s['expected_rays'] > 0 for s in inwin),
        'scans_with_hits': sum(s['hit_rays'] > 0 for s in inwin),
        'scans_with_fewer_than_3_hits': sum(s['hit_rays'] < 3 for s in inwin),
        'hit_rays_per_scan': stats([s['hit_rays'] for s in inwin]),
        'hit_ratio': (sum(s['hit_rays'] for s in inwin) / max(1, sum(s['expected_rays'] for s in inwin))),
        'structure_blocked_rays_total': sum(s['structure_blocked_rays'] for s in inwin),
        'bearing_span_deg': [min(s['bearing_deg'][0] for s in inwin if s['bearing_deg']), max(s['bearing_deg'][1] for s in inwin if s['bearing_deg'])],
        'scans_during_no_front_patch': gap_scans,
        'scans_in_window_failing_front_patch_window_approx': sum(s['front_patch_window_hits'] < 3 or not s['front_patch_window_has_abs_y_lt_0p15'] for s in inwin),
        'front_patch_rule_source': 'src/arena_autonomy/arena_autonomy/bypass_handoff.py front_patch(contiguous_three_v0): 0.20<=x<=1.60, |y|<=0.24, 연속 3광선, |y|<0.15 한 점',
    }

    # 6. 기하 참고(이번 주행 증거 아님): 같은 구조 각도에서 옆·뒤 상대가 차지하는 광선과 가림.
    inc = 2 * math.pi / 500
    geo = []
    for lat in (0.0, 0.27):
        for dxc in [1.4, 1.0, 0.6, 0.3, 0.0, -0.3, -0.6, -0.8, -1.0, -1.4]:
            if lat == 0.0 and abs(dxc) < BODY_L + 0.01:
                continue
            tot = vis = 0
            for i in range(500):
                ang = -math.pi + i * inc
                d = ray_box(lx - dxc, ly - lat, math.cos(ang), math.sin(ang), BODY_L / 2, BODY_W / 2)
                if d is None:
                    continue
                tot += 1
                vis += not in_sectors(math.degrees(ang), sectors)
            geo.append({'lateral_m': lat, 'actor_center_x_m': dxc, 'rays_on_box': tot, 'rays_outside_structure': vis})
    rear_open = 180 - max(abs(c) + hw for c, hw in sectors if abs(c) > 90)
    geometry = {
        'scope': '정적 기하 계산. 자차 base 좌표의 같은 방향 20×15 cm 상대 중심 위치별 광선 수. 다른 차폐·거리 한계·운동 미반영. 실행 증거 아님.',
        'blind_sectors_deg_center_halfwidth': sectors,
        'rear_open_full_angle_deg': 2 * rear_open,
        'table': geo,
    }

    out = {'scope': '평가 전용 사후 검토. 정답 자세(actor/ego_poses)는 평가에만 쓰며 제어기 입력이 아니다. 판정선 완화·재판정 없음.',
           'run': RUN.relative_to(HERE.parents[3]).as_posix(), 't_green': tg, 'window_end': te, 'stop_request_s': stop_s,
           'validator': {'passed': report['passed'], 'run_valid': report['run_valid'], 'gates': report['gates'],
                         'case_metrics': report['evaluation']['case_metrics'], 'progress': report['evaluation']['progress'],
                         'min_sampled_gap_m': report['evaluation']['gap']['min_sampled_gap_m'],
                         'min_swept_lower_bound_m': report['evaluation']['gap']['min_swept_lower_bound_m'],
                         'actor_contact_messages_with_ego': report['evaluation']['gap']['actor_contact_messages_with_ego'],
                         'stop_checks': report['stop_checks'], 'shutdown_clean': report['shutdown_clean']},
           'actor_script': actor, 'order_gap_speed': order, 'command_and_safety': command,
           'controller_observation': controller, 'scan_visibility_truth_check': visibility, 'geometry_reference': geometry}
    (HERE / 'review.json').write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: out[k] for k in ('validator',)}, ensure_ascii=False)[:400])


if __name__ == '__main__':
    main()
