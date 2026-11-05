"""직각 코너 시험 월드 단독 주행의 사후 집계(평가 전용). 정답 자세·벽 기하는 집계에만 쓰며 제어 입력이 아니다.

사용: analyze.py <run 디렉터리> <course_world 디렉터리> <출력 json>
원 report.json 판정을 바꾸지 않는다. 코너 구간은 시험 중심선 s(꼭짓점 ±1.0 m)로 나눈다.
"""
from collections import Counter
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import Polygon, Point
from shapely.ops import unary_union

ZONE_HALF = 1.0
STOP_V, STOP_MIN = .05, .1


def jsonl(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def rect(x, y, yaw, length, width):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = [(dx, dy) for dx, dy in ((length/2, width/2), (-length/2, width/2), (-length/2, -width/2), (length/2, -width/2))]
    return Polygon([(x+c*dx-s*dy, y+s*dx+c*dy) for dx, dy in pts])


def link_union(world, name):
    model = ET.parse(world).find("./world/model[@name='it_arena_track_static']")
    link = model.find(f"link[@name='{name}']")
    boxes = []
    for col in link.findall('collision'):
        p = list(map(float, col.findtext('pose').split()))
        sx, sy, _ = map(float, col.findtext('geometry/box/size').split())
        boxes.append(rect(p[0], p[1], p[5], sx, sy))
    return unary_union(boxes)


def spans(t, flag, min_s=0.):
    out, cur = [], None
    for a, b, f in zip(t, t[1:], flag):
        if f:
            cur = [a, b] if cur is None else [cur[0], b]
        elif cur is not None:
            out.append(cur); cur = None
    if cur:
        out.append(cur)
    return [c for c in out if c[1]-c[0] >= min_s]


def weighted(t, labels, mask):
    out = Counter()
    for a, b, la, m in zip(t, t[1:], labels, mask):
        if m:
            out[str(la)] += b-a
    return {k: round(x, 3) for k, x in out.most_common()}


def main(run, course, output):
    run, course, output = Path(run), Path(course), Path(output)
    result = dict(scope='평가 전용 사후 집계. 정답 자세·벽 기하는 제어 입력이 아니다. 1회 결과.')
    rpath = run/'report.json'
    if not rpath.exists():
        result['error'] = 'report.json missing'
        output.write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
        return 1
    r = json.loads(rpath.read_bytes())
    keys = ('passed', 'error', 'route_complete', 'completed_laps', 'progress_m', 'target_progress_m', 'lap_length_m',
            'collision_samples', 'stop_collision_samples', 'max_centerline_error_m', 'false_start', 'signal_states',
            'marker_ids', 'wall_sides', 'stop_stable', 'stop_wait_sim_s', 'stop_displacement_m', 'dense_pose_stop_evidence',
            'actuation_passed', 'shutdown_clean', 'remaining_test_pids', 'process_exit_codes', 'process_errors',
            'source_unchanged', 'sensor_only_inputs', 'safety_sensor_only_inputs', 'lidar_rate_passed', 'speed_profile',
            'drivetrain_stall', 'failure_cleanup_stop', 'start_time_sim_s')
    result['report'] = {k: r.get(k) for k in keys}
    act = r.get('actuation') or {}
    result['report']['final_mcu'] = {k: (act.get('final_mcu') or {}).get(k) for k in ('state', 'reason', 'feedback_mode')}
    result['report']['stop_mode'] = act.get('stop_mode')
    scene = json.loads((course/'scene.json').read_text(encoding='utf-8'))
    checks = json.loads((course/'geometry_checks.json').read_text(encoding='utf-8'))
    lap = float(scene['track']['lap_length_m'])
    corners = checks['corners']
    world = run/'runtime_world.sdf'
    walls = link_union(world, 'walls')
    road = link_union(world, 'track_surface')

    rows = jsonl(run/'trajectory.jsonl')
    green_i = next((i for i, x in enumerate(rows) if x['light'] == 'green'), None)
    t = np.array([x['time'] for x in rows])
    v = np.array([math.hypot(x['dynamics']['truth_longitudinal_mps'], x['dynamics']['truth_lateral_mps']) for x in rows])
    prog = np.array([x['progress_m'] for x in rows])
    s0 = rows[0]['s']
    s = (s0 + prog) % lap
    cmd = np.array([x['autonomy'].get('speed_command_mps') or 0. for x in rows])
    safe = np.array([x.get('safe_drive_speed_mps') or 0. for x in rows])
    err = np.array([x['centerline_error_m'] for x in rows])
    move = next((float(a) for a, b in zip(t, v) if b >= .1), None)
    clear = []
    off_body = []
    for x in rows:
        fp = rect(x['x'], x['y'], x['yaw'], .20, .15)
        clear.append(fp.distance(walls))
        off_body.append(fp.difference(road).area)
    clear, off_body = np.array(clear), np.array(off_body)
    after = (t >= (move if move is not None else np.inf))

    def zone_mask(sk):
        d = (s - sk + lap/2) % lap - lap/2
        return (np.abs(d) <= ZONE_HALF) & after

    # 계획기 0명령(50 Hz 판단 기록)
    dec = []
    fpath = run/'follow_decisions.jsonl'
    if fpath.exists():
        with fpath.open(encoding='utf-8') as stream:
            next(stream)
            for line in stream:
                e = json.loads(line)
                if e['kind'] != 'call':
                    continue
                vv, st, d = e['data']['output']
                dec.append((e['data']['input']['reference_s'], vv, st, d.get('reason'), d.get('behavior'),
                            d.get('preferred_wall'), d.get('path_wall'), d.get('turn_preview'),
                            (d.get('partial_observation') or {}).get('reason') if isinstance(d.get('partial_observation'), dict) else None))
    dt_ = np.array([q[0] for q in dec]) if dec else np.array([])
    ds_ = np.interp(dt_, t, s) if dec else np.array([])

    def zero_in(mask_fn=None, sk=None):
        out, n = Counter(), 0
        for (tt, vv, st, reason, beh, pw, pathw, tp, po), ss in zip(dec, ds_):
            if move is None or tt < move:
                continue
            if sk is not None and abs((ss - sk + lap/2) % lap - lap/2) > ZONE_HALF:
                continue
            n += 1
            if vv <= .001:
                out[str(reason)] += .02
        return n, {k: round(x, 2) for k, x in out.most_common()}

    zones = []
    for c in corners:
        m = zone_mask(c['s_m'])
        if not m.any():
            zones.append(dict(name=c['name'], role=c['role'], turn_deg=c['turn_deg'], reached=False))
            continue
        idx = np.nonzero(m)[0]
        tz = t[idx]
        stops = [dict(start_sim_s=round(a, 3), seconds=round(b-a, 3)) for a, b in
                 spans(list(t), list((v < STOP_V) & m), STOP_MIN)]
        n_calls, zero = zero_in(sk=c['s_m'])
        entry = idx[0]
        zones.append(dict(
            name=c['name'], role=c['role'], turn_deg=c['turn_deg'], reached=True,
            enter_sim_s=round(float(tz[0]), 3), exit_sim_s=round(float(tz[-1]), 3), seconds=round(float(tz[-1]-tz[0]), 3),
            completed_zone=bool(((s[idx[-1]] - c['s_m'] + lap/2) % lap - lap/2) >= ZONE_HALF - .1),
            entry_ground_mps=round(float(v[entry]), 3), min_ground_mps=round(float(v[idx].min()), 3),
            max_ground_mps=round(float(v[idx].max()), 3), mean_ground_mps=round(float(v[idx].mean()), 3),
            min_wall_clearance_m=round(float(clear[idx].min()), 4), max_centerline_error_m=round(float(err[idx].max()), 3),
            max_body_off_road_m2=round(float(off_body[idx].max()), 5), collision_samples=int(sum(rows[i]['collision'] for i in idx)),
            ground_stops=stops, decision_calls=n_calls, zero_command_s_by_reason=zero,
            safety_reason_s=weighted(list(t), [x['safety']['reason'] for x in rows], list(m)),
            state_s=weighted(list(t), [x['autonomy']['state'] for x in rows], list(m)),
            safe_below_cmd_samples=int(np.sum(safe[idx] < cmd[idx] - .05))))
    result['corner_zones'] = zones
    result['zone_definition'] = f'시험 중심선 s 기준 꼭짓점 ±{ZONE_HALF} m, 첫 0.1 m/s 이동 뒤 표본. 진행 s=(시작 s+progress) mod 랩.'
    gs = [dict(start_sim_s=round(a, 3), seconds=round(b-a, 3), s_m=round(float(np.interp(a, t, s)), 2))
          for a, b in spans(list(t), list((v < STOP_V) & after), STOP_MIN)]
    n_calls, zero = zero_in()
    crossing = None
    for a, b, ta, tb in zip(prog, prog[1:], t, t[1:]):
        if a < lap <= b:
            crossing = ta + (tb-ta)*(lap-a)/(b-a)
            break
    first_green = float(t[green_i]) if green_i is not None else None
    result['overall'] = dict(
        first_green_sample_s=first_green, first_ground_0p1_mps_s=move,
        lap_s_from_first_green=None if crossing is None or first_green is None else round(crossing-first_green, 3),
        final_progress_m=round(float(prog[-1]), 3), final_s_m=round(float(s[-1]), 3),
        peak_ground_mps=round(float(v[after].max()), 3) if after.any() else None,
        min_wall_clearance_m=round(float(clear[after].min()), 4) if after.any() else None,
        max_body_off_road_m2=round(float(off_body[after].max()), 5) if after.any() else None,
        ground_stops=gs, decision_calls_after_move=n_calls, zero_command_s_by_reason=zero,
        zero_command_s=round(sum(zero.values()), 2))
    # 마지막 3초(실패 원인 확인용)
    tail = [i for i in range(len(rows)) if t[i] >= t[-1]-3.]
    result['last_3s'] = [dict(t=round(float(t[i]), 3), x=round(rows[i]['x'], 3), y=round(rows[i]['y'], 3),
                              yaw=round(rows[i]['yaw'], 3), s=round(float(s[i]), 3), v=round(float(v[i]), 3),
                              cmd=round(float(cmd[i]), 3), safe=round(float(safe[i]), 3),
                              steer=rows[i]['autonomy'].get('steering_command_rad'), state=rows[i]['autonomy']['state'],
                              safety=rows[i]['safety']['reason'], err=round(float(err[i]), 3),
                              wall_clear=round(float(clear[i]), 4), collision=rows[i]['collision'])
                         for i in tail[::max(1, len(tail)//30)]]
    if dec:
        last = [q for q in dec if q[0] >= t[-1]-3.]
        result['last_3s_decisions'] = [dict(t=round(q[0], 3), v=round(q[1], 3), steer=round(q[2], 3), reason=q[3], behavior=q[4],
                                             preferred_wall=q[5], path_wall=q[6], turn_preview=q[7], partial=q[8])
                                       for q in last[::max(1, len(last)//30)]]
    for name in ('metrics', 'surface', 'motor_audit'):
        p = output.parent/(output.name.replace('_analysis.json', f'_{name}.json'))
        try:
            result[name] = json.loads(p.read_bytes()) if p.exists() and p.stat().st_size else None
        except ValueError:
            result[name] = 'unreadable'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=1, default=str), encoding='utf-8', newline='\n')
    print(json.dumps(dict(passed=r.get('passed'), error=(r.get('error') or '')[:200], overall=result['overall'],
                          zones=[(z['name'], z.get('reached'), z.get('min_ground_mps'), z.get('min_wall_clearance_m'),
                                  z.get('zero_command_s_by_reason')) for z in zones]), ensure_ascii=False, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main(*sys.argv[1:4]))
