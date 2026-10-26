"""run17 M3 순수 결합의 촉발·후보·발행각·재개 구간 분해. 개루프 진단이며 주행/제어 변경이 아니다.

- 제어 계산은 run17과 같은 동결 소스를 그대로 호출하며 `TruthBlocked` 안에서만 수행한다.
- 정답 자세(ego/actor)는 계산 루프가 끝난 뒤 사후 분류에만 연다. 제어 입력으로 되돌리지 않는다.
- 저장 궤적·직전 조향(`previous_steering_rad`)은 원래 G1 기록의 값이다. M3가 실제로 따라갈
  궤적/센서/직전 발행각이 아니므로 아래의 대조는 원인 분리용이며 폐루프 성공·안전 판정이 아니다.
- 거절 원인 분류(`unsupported_cause`)는 판정용 `depth_support` 결과를 바꾸지 않는 근사 설명이다.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/arena_autonomy').is_dir())
LEGACY = ROOT/'artifacts/validation/2026-10-24/m3_surface_lateral_v1'
sys.path.insert(0, str(LEGACY))
import screen_lateral as screen  # noqa: E402  (src 경로도 추가한다)
from arena_autonomy.m3_observed_follow import M3ObservedFollow, command_arc  # noqa: E402
from arena_autonomy.observed_lateral_trial import path_observation_queries  # noqa: E402
from arena_autonomy.observation_support import depth_support  # noqa: E402
from arena_autonomy.local_follow import pursue_path  # noqa: E402
from arena_autonomy.lidar_motion import MotionUnavailable  # noqa: E402
from arena_autonomy.follow_trace import freeze  # noqa: E402
import arena_autonomy.lateral_intent as li  # noqa: E402

RECORDS = ('e3_onroad_g1', 'e3_onroad_r0plus', 'e3_stop_restart_g1')
RUN17 = LEGACY/'run17_follow_geometry/replay.json'
EPS = li.GEOMETRY_EPS_M


def stop_distances(speed):
    """참고선. 다음 fresh 판단(스캔 0.10+제어 0.02 s)+반응 0.10 s+2 m/s² 제동, 그리고 제동만."""
    v = max(.35, abs(speed))
    return {'next_decision_stop_m': v*(.10+.02+.10)+v*v/4., 'reaction_brake_m': v*.10+v*v/4.}


def pad_for(scene, cfg):
    return np.array([.025+cfg.budget.range_bias_m+.5*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])


def pose_queries(scene, pose, steering, cfg):
    """observed_candidate/command_observation과 같은 질의(차체 외곽+unknown 교차+바퀴 지름 외곽)."""
    q, _ = path_observation_queries(scene, pose.reshape(1, 3), cfg)
    from dataclasses import replace
    wheel = li.wheel_contact_corners(pose.reshape(1, 3), np.array([steering]),
                                     replace(cfg, contact_patch_length_m=2*cfg.wheel_radius_m)).reshape(-1, 2)
    wheel = wheel[(abs(wheel[:, 0]) > cfg.length_m/2+EPS) | (abs(wheel[:, 1]) > cfg.width_m/2+EPS)]
    return np.unique(np.vstack((q, wheel)), axis=0) if len(wheel) else q


def unsupported_cause(scene, q, pad):
    """거절 질의의 근사 원인과 원인 광선. 판정 자체는 depth_support가 결정한다."""
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    status = [r.status for r in scene.rays]
    depth = np.array([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return' else -1. for r in scene.rays])
    out = []
    for point in np.asarray(q).reshape(-1, 2):
        d = point-origins
        ang = np.arctan2(directions[:, 0]*d[:, 1]-directions[:, 1]*d[:, 0], directions[:, 0]*d[:, 0]+directions[:, 1]*d[:, 1])
        rad = np.hypot(d[:, 0], d[:, 1])
        pos = np.where(ang >= -1e-10, ang, np.inf)
        neg = np.where(ang <= 1e-10, -ang, np.inf)
        lo, hi = int(np.argmin(pos)), int(np.argmin(neg))
        if not (np.isfinite(pos[lo]) and np.isfinite(neg[hi])) or pos[lo]+neg[hi] > .025 or abs(lo-hi) > 1:
            out.append(('bracket_gap', None))
            continue
        bad = [i for i in (lo, hi) if status[i] != 'return']
        if bad:
            out.append(('rear_mask' if status[bad[0]] == 'masked' else 'no_return', bad[0]))
            continue
        margin = {i: depth[i]-rad[i] for i in (lo, hi)}
        worst = min(margin, key=margin.get)
        out.append(('behind_return' if margin[worst] <= 0 else 'within_pad', worst))
    return out


def first_unsupported(scene, poses, steering, progress, cfg, pad):
    """자세 순서대로 지지 실패가 처음 나타나는 진행 거리와 그 자세의 거절 질의."""
    for index, pose in enumerate(poses):
        q = pose_queries(scene, pose, steering[index], cfg)
        if not len(q):
            continue
        ok = depth_support(scene, q, pad)
        if not np.all(ok):
            return float(progress[index]), q[~ok]
    return None, np.zeros((0, 2))


def summarize_queries(scene, q, pad, cfg):
    causes = unsupported_cause(scene, q, pad)
    lx, wy = cfg.length_m/2, cfg.width_m/2
    rows = []
    for (cause, ray), point in zip(causes, q):
        where = 'front' if point[0] > lx else ('rear' if point[0] < -lx else 'beside')
        hit = None
        if ray is not None and scene.rays[ray].status == 'return':
            hit = list(scene.rays[ray].hit_xy_m)
        rows.append({'xy': [float(point[0]), float(point[1])], 'cause': cause, 'where': where,
                     'side': 'left' if point[1] > 0 else 'right', 'hit_xy': hit})
    return rows


def threat_points(scene, path, cfg):
    """side_threats와 같은 창·벽·compact 조건의 측방 반환(base_link)."""
    frame = li.PathFrame(path)
    state = li.ego_state(frame, cfg)
    rays = [r for r in scene.rays if r.status == 'return']
    pts = np.asarray([r.hit_xy_m for r in rays]).reshape(-1, 2)
    compact, _ = li.compact_return_mask(scene, cfg)
    s, l = frame.project(pts)
    ds = s-state['s0']
    window = (ds >= -cfg.band_behind_m) & (ds <= cfg.band_ahead_m)
    wall = np.zeros(len(pts), dtype=bool)
    for _, sign in li.SIDES:
        wall |= np.abs(l-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m
    interior = np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m
    out = {}
    for name, sign in li.SIDES:
        side = window & interior & ~wall & (sign*(l-state['d0']) >= state['half_span_m']) & compact
        out[name] = pts[side].tolist()
    return out, state


def analyze_record(name, stored_rows):
    motion, config, events = screen.load_inputs(screen.RECORDS[name])
    from dataclasses import replace
    config = replace(config, occupancy_timing_mode='per_ray_candidate', recent_observation_enabled=False)
    m3 = M3ObservedFollow(config, allow_simulation_trial=True)
    cfg = m3.lateral_config
    # 소스를 바꾸지 않고 이번 호출의 기본 G1 반환값만 가로챈다(반환값은 그대로 전달).
    captured, original = {}, m3.base.command

    def capture(*a, **k):
        captured['base'] = original(*a, **k)
        return captured['base']
    m3.base.command = capture
    calls, mismatches, index = [], 0, 0
    last_source = None
    with screen.TruthBlocked():
        for event in events:
            if event['kind'] == 'reset':
                m3.reset()
                last_source = None
                continue
            data, scan, history = screen.restore_call(screen.thaw(event['data']['input']), motion)
            scene = screen.motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            speed, prev, dt = data['speed_mps'], data['previous_steering_rad'], data['dt_s']
            fresh = last_source is None or scene.source_first_s > last_source
            last_source = scene.source_first_s
            captured.clear()
            out = m3.command(scene, history, speed, data['side'], prev, dt)
            frozen = freeze(out)
            stored = stored_rows[index]['output']
            same = frozen[0] == stored[0] and frozen[1] == stored[1] and frozen[2]['reason'] == stored[2]['reason']
            mismatches += int(not same)
            index += 1
            detail = out[2]
            lat = detail.get('m3_lateral') or {}
            row = {'i': index-1, 't': scene.t_ref_s, 'age_s': scene.t_ref_s-scene.source_first_s, 'fresh': fresh,
                   'speed': speed, 'previous_steering': prev, 'recorded_output': list(event['data']['output'][:2]),
                   'base_reason': detail.get('baseline_reason'), 'base_output': detail.get('baseline_output'),
                   'reason': detail['reason'], 'output': [float(out[0]), float(out[1])], 'mode': lat.get('mode'),
                   'held_after': m3.held, 'release_count_after': m3.release_count, 'active_after': m3.lateral.active,
                   'stop_ref': stop_distances(speed)}
            if lat.get('threats'):
                row['threats'] = {k: {'present': v['present'], 'points': v['points'],
                                      'extended_surface_points': v.get('extended_surface_points'),
                                      'nearest': v['nearest']} for k, v in lat['threats'].items()}
                row['state'] = lat.get('state')
            if lat.get('mode') not in (None, 'BASELINE', 'OFF', 'HOLD_INPUT', 'HOLD_PATH'):
                # 이번 호출의 동결 LocalFollow 경계 경로(기본 경로)에서 후보를 다시 만든다.
                pad = pad_for(scene, cfg)
                base_detail = captured['base'][2]
                bp = np.asarray(base_detail['path_points']) if base_detail.get('path_points') is not None else None
                row['checks'] = {}
                for key, check in (lat.get('checks') or {}).items():
                    row['checks'][key] = {'reasons': check['reasons'], 'target_m': check['target_m'],
                        'required_progress_m': check.get('required_progress_m'),
                        'unsupported_queries': check.get('unsupported_queries'),
                        'lateral_min_margin_m': (check.get('lateral_clearance') or {}).get('min_margin_m'),
                        'lateral_worst': (check.get('lateral_clearance') or {}).get('worst'),
                        'surface': check.get('surface')}
                row['_scene'] = scene
                row['_pad'] = pad
                if bp is not None:
                    row['_base_path'] = bp
            if detail.get('m3_selected'):
                sel = np.asarray(detail['path_points'])
                age = scene.t_ref_s-scene.source_first_s
                try:
                    _, desired, _ = pursue_path(sel, speed, config, max(age, detail.get('boundary_age_s') or 0.))
                except MotionUnavailable:
                    desired = None
                envelope = max(speed, m3.trial_speed_cap_mps)
                distance = li.committed_progress_m(envelope, age, cfg)
                pad = pad_for(scene, cfg)
                # 이번 발행각은 clip(desired, prev±3·dt)로 다시 계산한다(동결 식과 같음).
                issued = float(np.clip(desired, prev-3*dt, prev+3*dt)) if desired is not None else None
                arcs = {}
                for label, angle in (('issued', issued), ('previous', prev)):
                    if angle is None:
                        continue
                    poses = command_arc(angle, distance, cfg)
                    progress = np.linspace(0., distance, len(poses))
                    first, bad = first_unsupported(scene, poses, np.full(len(poses), angle), progress, cfg, pad)
                    arcs[label] = {'angle': angle, 'first_unsupported_progress_m': first,
                                   'unsupported_examples': summarize_queries(scene, bad[:12], pad, cfg)}
                geometry = detail.get('m3_command_geometry') or {}
                row['selected'] = {'desired_pp_rad': desired, 'issued_rad': issued, 'previous_rad': prev,
                    'rate_limited': desired is not None and abs(desired-issued) > 1e-9,
                    'commit_distance_m': distance, 'arcs': arcs,
                    'geometry_checks': [{'steering_rad': c['steering_rad'], 'reasons': c['reasons'],
                                         'lateral_min_margin_m': c['lateral_clearance'].get('min_margin_m'),
                                         'lateral_worst': c['lateral_clearance'].get('worst'),
                                         'surface_reason': c['surface'].get('reason')}
                                        for c in geometry.get('checks', [])],
                    'swept_clearance_m': detail.get('swept_clearance_m'),
                    'target_speed_before_hold_mps': detail.get('target_speed_before_hold_mps'),
                    'observation': {k: v for k, v in (detail.get('m3_command_observation') or {}).items()}}
                row['selected_path'] = sel
            calls.append(row)
    # 후보별 최초 지지 실패 진행 거리(선택 실패 판단만). 같은 동결 함수로 후보를 다시 만든다.
    for row in calls:
        if '_scene' not in row or '_base_path' not in row:
            continue
        scene, pad, bp = row.pop('_scene'), row.pop('_pad'), row.pop('_base_path')
        frame = li.PathFrame(bp)
        state = li.ego_state(frame, cfg)
        pts, _ = threat_points(scene, bp, cfg)
        row['threat_xy'] = pts
        for key, check in row['checks'].items():
            cand = li.offset_candidate(frame, state, check['target_m'], cfg, key)
            dist = check['required_progress_m'] or 0.
            keep = cand['progress'] <= dist+EPS
            first, bad = first_unsupported(scene, cand['poses'][keep], cand['steering_rad'][keep],
                                           cand['progress'][keep], cfg, pad)
            check['first_unsupported_progress_m'] = first
            check['unsupported_examples'] = summarize_queries(scene, bad[:16], pad, cfg)
            check['delta_m'] = cand['delta_m']
    for row in calls:
        row.pop('_scene', None)
        row.pop('_pad', None)
        row.pop('_base_path', None)
    return calls, mismatches


# ------------------------------------------------------------------ 사후 정답 분류(계산 뒤)
def load_truth(name):
    base = screen.ROOT/screen.RECORDS[name]
    def series(rows):
        a = np.asarray(rows, dtype=float)
        return a[np.argsort(a[:, 0])]
    ego = series([json.loads(x) for x in (base/'ego_poses.jsonl').read_text().splitlines() if x.strip()])
    actors = {}
    lead = base/'actor_poses.jsonl'
    if lead.exists():
        rows = [json.loads(x) for x in lead.read_text().splitlines() if x.strip()]
        if rows:
            actors['lead'] = series(rows)
    extra = json.loads((base/'additional_actors.json').read_text(encoding='utf-8'))
    for key, rows in extra.get('poses', {}).items():
        actors['side' if 'side' in key else key] = series(rows)
    return ego, actors


def at(series, t):
    i = int(np.clip(np.searchsorted(series[:, 0], t), 1, len(series)-1))
    a, b = series[i-1], series[i]
    w = 0. if b[0] == a[0] else float(np.clip((t-a[0])/(b[0]-a[0]), 0., 1.))
    yaw = a[3]+w*math.remainder(b[3]-a[3], math.tau)
    return a[1]+w*(b[1]-a[1]), a[2]+w*(b[2]-a[2]), yaw


def to_world(xy, pose):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    p = np.asarray(xy, dtype=float).reshape(-1, 2)
    return np.column_stack((pose[0]+c*p[:, 0]-s*p[:, 1], pose[1]+s*p[:, 0]+c*p[:, 1]))


def rect_distance(points, pose, length=.20, width=.15):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    d = np.asarray(points)-[pose[0], pose[1]]
    lx, ly = c*d[:, 0]+s*d[:, 1], -s*d[:, 0]+c*d[:, 1]
    ex, ey = np.maximum(0., abs(lx)-length/2), np.maximum(0., abs(ly)-width/2)
    return np.hypot(ex, ey)


def label_points(points, t, ego, actors, tolerance=.06):
    if not len(points):
        return {}
    w = to_world(points, at(ego, t))
    labels = np.array(['other']*len(w), dtype=object)
    best = np.full(len(w), np.inf)
    for key, series in actors.items():
        d = rect_distance(w, at(series, t))
        take = (d <= tolerance) & (d < best)
        labels[take] = key
        best = np.minimum(best, d)
    out = {}
    for lab in labels:
        out[lab] = out.get(lab, 0)+1
    return out


def relative_actor(t, ego, actors):
    pose = at(ego, t)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    out = {}
    for key, series in actors.items():
        a = at(series, t)
        dx, dy = a[0]-pose[0], a[1]-pose[1]
        out[key] = {'x_m': c*dx+s*dy, 'y_m': -s*dx+c*dy, 'yaw_rel_rad': math.remainder(a[2]-pose[2], math.tau)}
    return out


def posthoc(name, calls):
    ego, actors = load_truth(name)
    for row in calls:
        if 'threat_xy' in row:
            row['truth_threat_labels'] = {k: label_points(v, row['t'], ego, actors) for k, v in row['threat_xy'].items()}
            row['truth_relative_actor'] = relative_actor(row['t'], ego, actors)
            row.pop('threat_xy')
        for check in (row.get('checks') or {}).values():
            hits = [q['hit_xy'] for q in check.get('unsupported_examples', []) if q['hit_xy'] is not None]
            check['truth_blocking_hit_labels'] = label_points(np.asarray(hits).reshape(-1, 2), row['t'], ego, actors) if hits else {}
        sel = row.get('selected')
        if sel:
            for arc in sel['arcs'].values():
                hits = [q['hit_xy'] for q in arc['unsupported_examples'] if q['hit_xy'] is not None]
                arc['truth_blocking_hit_labels'] = label_points(np.asarray(hits).reshape(-1, 2), row['t'], ego, actors) if hits else {}
            row['truth_relative_actor'] = relative_actor(row['t'], ego, actors)
        row.pop('selected_path', None)


def count(values):
    out = {}
    for v in values:
        out[v] = out.get(v, 0)+1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def summarize(name, calls):
    trig = [r for r in calls if r['mode'] not in (None, 'BASELINE', 'OFF')]
    first = trig[0] if trig else None
    stops = [r for r in calls if r['mode'] == 'HOLD_STOP_TRIAL']
    selected = [r for r in calls if r.get('selected')]
    cand_reasons = {}
    cand_first = {}
    cand_causes = {}
    blocking_labels = {}
    for r in stops:
        for key, check in r['checks'].items():
            for reason in check['reasons']:
                cand_reasons.setdefault(key, {}).setdefault(reason, 0)
                cand_reasons[key][reason] += 1
            p = check.get('first_unsupported_progress_m')
            if p is not None:
                cand_first.setdefault(key, []).append(p)
            for q in check.get('unsupported_examples', []):
                k = f"{q['cause']}:{q['where']}"
                cand_causes.setdefault(key, {}).setdefault(k, 0)
                cand_causes[key][k] += 1
            for lab, n in check.get('truth_blocking_hit_labels', {}).items():
                blocking_labels.setdefault(key, {}).setdefault(lab, 0)
                blocking_labels[key][lab] += n
    def dist(values):
        if not values:
            return None
        a = np.asarray(values)
        return {'n': len(a), 'min': float(a.min()), 'p50': float(np.median(a)), 'max': float(a.max()),
                'below_reaction_brake_0.066m': int(np.sum(a < .0656)), 'below_next_decision_0.108m': int(np.sum(a < .1076))}
    sel_rows = []
    for r in selected:
        s = r['selected']
        sel_rows.append({'i': r['i'], 't': round(r['t'], 3), 'fresh': r['fresh'], 'mode': r['mode'], 'reason': r['reason'],
            'speed': round(r['speed'], 3), 'desired': None if s['desired_pp_rad'] is None else round(s['desired_pp_rad'], 3),
            'issued': None if s['issued_rad'] is None else round(s['issued_rad'], 3), 'previous': round(s['previous_rad'], 3),
            'issued_first_unsupported_m': s['arcs'].get('issued', {}).get('first_unsupported_progress_m'),
            'previous_first_unsupported_m': s['arcs'].get('previous', {}).get('first_unsupported_progress_m'),
            'geometry': [(round(c['steering_rad'], 3), c['reasons']) for c in s['geometry_checks']],
            'clearance': s['swept_clearance_m'], 'held_after': r['held_after'], 'release_after': r['release_count_after']})
    # 측방 선택 뒤 HOLD 재개 연쇄: 선택(wait 포함) 판단 사이에 끼어든 정지 원인.
    chains, current = [], None
    for r in calls:
        if r['reason'] == 'm3_release_wait':
            if current is None:
                current = {'start_i': r['i'], 'start_t': r['t'], 'fresh_in_wait': 0, 'max_release': 0, 'end': None}
            current['fresh_in_wait'] += int(r['fresh'])
            current['max_release'] = max(current['max_release'], r['release_count_after'])
        elif current is not None:
            current['end'] = {'i': r['i'], 't': r['t'], 'reason': r['reason'], 'mode': r['mode'], 'fresh': r['fresh']}
            chains.append(current)
            current = None
    if current is not None:
        chains.append(current)
    return {
        'calls': len(calls),
        'first_trigger': None if first is None else {k: first.get(k) for k in (
            'i', 't', 'mode', 'reason', 'speed', 'base_reason', 'base_output', 'threats', 'state',
            'truth_threat_labels', 'truth_relative_actor', 'checks')},
        'trigger_truth_labels': count('|'.join(sorted(f'{side}:{lab}' for side, labs in r.get('truth_threat_labels', {}).items()
                                                     for lab in labs)) or 'none' for r in trig if 'truth_threat_labels' in r),
        'modes_after_trigger': count(r['mode'] for r in trig),
        'reasons_after_trigger': count(r['reason'] for r in trig),
        'base_reason_when_trial_stopped': count(r['base_reason'] for r in stops),
        'candidate_reasons_in_hold_stop': cand_reasons,
        'candidate_first_unsupported_progress_m': {k: dist(v) for k, v in cand_first.items()},
        'candidate_unsupported_causes': cand_causes,
        'candidate_blocking_hit_truth_labels': blocking_labels,
        'selected': sel_rows,
        'release_chains': chains,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('out_dir')
    args = parser.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    stored = json.loads(RUN17.read_text(encoding='utf-8'))
    report = {'scope': 'open_loop_run17_breakdown_not_driving', 'truth_used_for_control': False,
              'records': {}}
    for name in RECORDS:
        started = time.perf_counter()
        calls, mismatches = analyze_record(name, stored['records'][name]['rows'])
        posthoc(name, calls)
        with (out/f'calls_{name}.jsonl').open('w', encoding='utf-8', newline='\n') as stream:
            for row in calls:
                stream.write(json.dumps(row, ensure_ascii=False)+'\n')
        summary = summarize(name, calls)
        summary['run17_output_mismatches'] = mismatches
        summary['elapsed_s'] = time.perf_counter()-started
        report['records'][name] = summary
        print(name, 'calls', len(calls), 'mismatch', mismatches, 'elapsed', round(summary['elapsed_s'], 1), flush=True)
    (out/'breakdown.json').write_text(json.dumps(report, indent=1, ensure_ascii=False), encoding='utf-8', newline='\n')


if __name__ == '__main__':
    main()
