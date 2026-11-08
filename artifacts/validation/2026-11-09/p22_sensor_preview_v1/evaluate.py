"""2단계(평가 전용): 1단계 관측을 고정한 뒤에만 궤적·월드·마커·상대 자세와 대조한다."""
import bisect
import hashlib
import json
import math
import re
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SCENE = ROOT / 'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/scene.json'
RUNS = {
    'r11': 'artifacts/validation/2026-11-07/junction_handoff_v1/run1',
    'r27': 'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/run1',
}
MATCH_M = .15
ACTOR_M = .25
ENTRY = {'E1': (5.635, 4.80), 'E2': (5.635, 4.20)}
EXIT = {'E1': (4.815, 4.20), 'E2': (4.815, 4.80)}  # 북행 기준 상류/하류
C4_INNER = (6.645, 5.495)
BRANCH_CENTER = (6.14, 4.50)
WINDOWS = [('C4_approach', 16.55, 17.545), ('C4_turn', 17.545, 17.85),
           ('P22_approach', 17.85, 18.68), ('P22_beside', 18.68, 19.36)]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_jsonl(path):
    with path.open(encoding='utf-8') as f:
        return [json.loads(l) for l in f if l.strip()]


def wall_corners(world):
    text = world.read_text(encoding='utf-8')
    corners, boxes = [], []
    for m in re.finditer(r'<collision name="([^"]*)">(.*?)</collision>', text, re.S):
        name, body = m.groups()
        pose, size = re.search(r'<pose>([^<]*)</pose>', body), re.search(r'<size>([^<]*)</size>', body)
        if not pose or not size or not name.startswith('walls'):
            continue
        p = list(map(float, pose.group(1).split()))
        sx, sy, sz = map(float, size.group(1).split())
        c, s = math.cos(p[5]), math.sin(p[5])
        boxes.append((name, p[0], p[1], p[5], sx, sy))
        for ax, ay in ((sx/2, sy/2), (sx/2, -sy/2), (-sx/2, sy/2), (-sx/2, -sy/2)):
            corners.append((p[0] + c*ax - s*ay, p[1] + s*ax + c*ay))
    return np.array(corners), boxes


def centerline_corners(scene):
    pts = np.array(scene['track']['centerline_polyline'], float)
    seg = np.diff(pts, axis=0)
    length = np.hypot(seg[:, 0], seg[:, 1])
    s = np.r_[0, np.cumsum(length)]
    heading = np.unwrap(np.arctan2(seg[:, 1], seg[:, 0]))
    out, i = [], 0
    while i < len(heading):
        j = i
        while j + 1 < len(heading) and abs(heading[j+1] - heading[j]) > 1e-3:
            j += 1
        turn = heading[j] - heading[i-1] if i > 0 else 0.
        if j > i and abs(turn) > math.radians(60):
            mid = (s[i] + s[j+1]) / 2
            out.append(dict(s=float(mid), turn_deg=float(math.degrees(turn)),
                            xy=pts[int(np.argmin(abs(s - mid)))].tolist()))
        i = j + 1
    for k, c in enumerate(out, 1):
        c['name'] = f'C{k}'
        c['direction'] = 'left' if c['turn_deg'] > 0 else 'right'
    return out, pts, s


class Poses:
    def __init__(self, rows):
        a = np.array([r for r in rows], float)
        order = np.argsort(a[:, 0], kind='stable')
        self.a = a[order]
        self.t = self.a[:, 0]
        self.yaw = np.unwrap(self.a[:, 3])

    def at(self, t):
        x = float(np.interp(t, self.t, self.a[:, 1]))
        y = float(np.interp(t, self.t, self.a[:, 2]))
        yaw = float(np.interp(t, self.t, self.yaw))
        return x, y, yaw

    def speed(self, t, h=.05):
        x0, y0, _ = self.at(t - h)
        x1, y1, _ = self.at(t + h)
        return math.hypot(x1 - x0, y1 - y0) / (2*h)

    def gap(self, t):
        i = bisect.bisect_left(self.t, t)
        return float(min(abs(self.t[max(0, i-1)] - t), abs(self.t[min(len(self.t)-1, i)] - t)))


def to_world(pose, p):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c*p[0] - s*p[1], y + s*p[0] + c*p[1])


def to_body(pose, q):
    x, y, yaw = pose
    dx, dy = q[0] - x, q[1] - y
    c, s = math.cos(yaw), math.sin(yaw)
    return (c*dx + s*dy, -s*dx + c*dy)


def main():
    scene_doc = json.loads(SCENE.read_bytes())
    corners_cl, cl_pts, cl_s = centerline_corners(scene_doc)

    def project_s(q):
        d = np.hypot(cl_pts[:, 0] - q[0], cl_pts[:, 1] - q[1])
        return float(cl_s[int(np.argmin(d))])

    def place(q):
        """세계 점을 사람이 읽는 장소 이름으로. 평가 전용."""
        if math.dist(q, ENTRY['E1']) <= MATCH_M or math.dist(q, ENTRY['E2']) <= MATCH_M:
            return 'P22_entry_edge'
        if math.dist(q, EXIT['E1']) <= MATCH_M or math.dist(q, EXIT['E2']) <= MATCH_M:
            return 'P22_exit_edge'
        sq = project_s(q)
        near = min(corners_cl, key=lambda c: abs(c['s'] - sq))
        if abs(near['s'] - sq) <= 1.0:
            return f"{near['name']}({near['direction']})"
        return f'straight_s{sq:.1f}'

    report = dict(scope='saved C1 observations from two completed main-route laps; evaluator-only matching; '
                        'no new simulation, no steering, no branch driving',
                  centerline_corners=corners_cl, runs={})
    for name, rel in RUNS.items():
        D = ROOT / rel
        obs = json.loads((HERE / f'observations_{name}.json').read_bytes())
        sensor = D / 'follow_decisions.jsonl'
        world = D / 'runtime_world.sdf'
        inputs = {p.relative_to(ROOT).as_posix(): digest(p) for p in (
            sensor, D / 'trace.json', D / 'ego_poses.jsonl', world, SCENE)}
        assert inputs[sensor.relative_to(ROOT).as_posix()] == obs['sensor_sha256']
        wc, boxes = wall_corners(world)
        # 실제 월드 확인: 입구/출구 간극 꼭짓점이 실제 벽 꼭짓점과 일치하는지
        world_check = {k: float(np.min(np.hypot(*(wc - np.array(v)).T)))
                       for k, v in {'entry_E1': ENTRY['E1'], 'entry_E2': ENTRY['E2'],
                                    'exit_E1': EXIT['E1'], 'exit_E2': EXIT['E2'], 'C4_inner': C4_INNER}.items()}
        # 간극 내부(직선 위)에 벽 상자가 없는지: 입구 y 4.21–4.79, x=5.635 선 위 상자 중심 탐색
        gap_walls = [b[0] for b in boxes if abs(b[1] - 5.61) < .03 and 4.21 < b[2] < 4.79]
        poses = Poses(load_jsonl(D / 'ego_poses.jsonl'))
        trace = json.loads((D / 'trace.json').read_bytes())
        tkey = 'time' if 'time' in trace[0] else 't'
        tt = [v[tkey] for v in trace]

        def trace_at(t):
            i = bisect.bisect_left(tt, t)
            return min(trace[max(0, i-1):min(len(trace), i+1)], key=lambda v: abs(v[tkey] - t))

        actors = {}
        if (D / 'actor_poses.jsonl').exists():
            actors['actor'] = Poses(load_jsonl(D / 'actor_poses.jsonl'))
        if (D / 'additional_actors.json').exists():
            extra = json.loads((D / 'additional_actors.json').read_bytes())
            for k, v in extra.get('poses', {}).items():
                actors[k] = Poses(v)

        def near_actor(t, q):
            best = None
            for k, p in actors.items():
                if not p.t[0] <= t <= p.t[-1]:
                    continue
                ax, ay, _ = p.at(t)
                d = math.hypot(q[0] - ax, q[1] - ay)
                if d <= ACTOR_M and (best is None or d < best[1]):
                    best = (k, d)
            return best

        scans, cands = [], []
        for idx, r in enumerate(obs['rows']):
            pose = poses.at(r['t'])
            tr = trace_at(r['t'])
            s = tr['s']
            window = next((w for w, a, b in WINDOWS if a <= s < b), None)
            e1b = to_body(pose, ENTRY['E1'])
            ctr = to_body(pose, BRANCH_CENTER)
            c4b = to_body(pose, C4_INNER)
            speed = poses.speed(r['t'])
            row = dict(i=idx, t=r['t'], s=s, window=window, speed_mps=speed, pose_gap_s=poses.gap(r['t']),
                       entry_E1_forward_m=e1b[0], entry_E1_lateral_m=e1b[1], entry_center_forward_m=ctr[0],
                       C4_inner_forward_m=c4b[0], returns=r['returns'], unknown=r['unknown'], features=[])
            for side in ('right', 'left'):
                for kind in ('notches', 'terminations'):
                    for c in r[side][kind]:
                        w1 = to_world(pose, c['e1'])
                        item = dict(side=side, kind='notch' if kind == 'notches' else 'termination',
                                    e1_world=w1, e1_place=place(w1), e1_body=c['e1'],
                                    depth_rays=c['depth_rays'], a_span=c['a_span'])
                        a1 = near_actor(r['t'], w1)
                        if kind == 'notches':
                            w2 = to_world(pose, c['e2'])
                            item.update(e2_world=w2, e2_place=place(w2), gap_m=c['gap_m'],
                                        gap_depth_rays=c['gap_depth_rays'], gap_unknown_rays=c['gap_unknown_rays'],
                                        c_span=c['c_span'], back_faces=len(c['back_faces']))
                            a1 = a1 or near_actor(r['t'], w2)
                            if math.dist(w1, ENTRY['E1']) <= MATCH_M and math.dist(w2, ENTRY['E2']) <= MATCH_M:
                                item['match'] = 'P22_entry'
                                item['err_m'] = [math.dist(w1, ENTRY['E1']), math.dist(w2, ENTRY['E2'])]
                            elif math.dist(w1, EXIT['E1']) <= MATCH_M and math.dist(w2, EXIT['E2']) <= MATCH_M:
                                item['match'] = 'P22_exit'
                            else:
                                d1 = float(np.min(np.hypot(*(wc - np.array(w1)).T)))
                                d2 = float(np.min(np.hypot(*(wc - np.array(w2)).T)))
                                item['match'] = 'other_real_wall_ends' if max(d1, d2) <= MATCH_M else 'not_a_real_gap'
                                item['wall_corner_dist_m'] = [d1, d2]
                        else:
                            item['match'] = item['e1_place']
                        if a1:
                            item['actor_near'] = [a1[0], a1[1]]
                        row['features'].append(item)
                        cands.append(dict(item, t=r['t'], s=s, window=window, i=idx))
            scans.append(row)

        def first_consecutive(pred):
            best, run = None, []
            for row in scans:
                if pred(row):
                    run.append(row)
                    if best is None or len(run) > len(best):
                        best = list(run)
                else:
                    run = []
            return best or []

        entry_hit = lambda row: any(f['kind'] == 'notch' and f['side'] == 'right' and f.get('match') == 'P22_entry'
                                    and row['entry_E1_forward_m'] > 0 for f in row['features'])
        entry_scans = [row for row in scans if entry_hit(row)]
        rn = [c for c in cands if c['kind'] == 'notch' and c['side'] == 'right']
        false_rn = [c for c in rn if c['match'] in ('not_a_real_gap', 'other_real_wall_ends')]
        c4_rn = [c for c in rn if c['window'] in ('C4_approach', 'C4_turn')]
        consec = first_consecutive(entry_hit)
        # 접근 창(C4 접근~P22 옆) 각 스캔의 관측/비관측 표
        window_rows = []
        for row in scans:
            if row['window'] is None:
                continue
            feats = [f"{f['side'][0].upper()}{'N' if f['kind'] == 'notch' else 'T'}:{f['match']}"
                     + (f"@{f['actor_near'][0]}" if 'actor_near' in f else '') for f in row['features']]
            window_rows.append(dict(t=round(row['t'], 3), s=row['s'], window=row['window'],
                                    speed=round(row['speed_mps'], 3), E1_fwd=round(row['entry_E1_forward_m'], 3),
                                    E1_lat=round(row['entry_E1_lateral_m'], 3),
                                    C4_fwd=round(row['C4_inner_forward_m'], 3), unknown=row['unknown'],
                                    features=sorted(set(feats))))
        # ID20
        frames, status_times = [], []
        for v in trace:
            if 20 in v['autonomy'].get('marker_ids', []):
                status_times.append(v[tkey])
                lr = (v['autonomy'].get('vision') or {}).get('last_result') or {}
                if 'stamp' in lr:
                    frames.append(lr['stamp'])
        status_file = D / 'autonomy_status.jsonl'
        if status_file.exists():
            for e in load_jsonl(status_file):
                if 20 in e.get('marker_ids', []):
                    status_times.append(e['sim_time_s'])
        status_times = sorted(set(round(x, 4) for x in status_times))
        frames = sorted(set(frames))

        def where(t):
            pose = poses.at(t)
            return dict(t=t, s=trace_at(t)['s'], speed_mps=poses.speed(t),
                        entry_E1_forward_m=to_body(pose, ENTRY['E1'])[0],
                        C4_inner_forward_m=to_body(pose, C4_INNER)[0])

        id20 = dict(status_times_s=status_times, distinct_camera_stamps_s=frames,
                    camera_stamps_available=bool(frames),
                    first_status=where(status_times[0]) if status_times else None,
                    first_camera=where(frames[0]) if frames else None,
                    camera_frames=[where(t) for t in frames])

        def summarize(row):
            f = next(f for f in row['features'] if f['kind'] == 'notch' and f.get('match') == 'P22_entry')
            return dict(t=row['t'], s=row['s'], window=row['window'], speed_mps=row['speed_mps'],
                        E1_forward_m=row['entry_E1_forward_m'], center_forward_m=row['entry_center_forward_m'],
                        E1_time_left_s=row['entry_E1_forward_m'] / max(row['speed_mps'], 1e-6),
                        center_time_left_s=row['entry_center_forward_m'] / max(row['speed_mps'], 1e-6),
                        gap_m=f['gap_m'], err_m=f['err_m'], gap_depth_rays=f['gap_depth_rays'],
                        gap_unknown_rays=f['gap_unknown_rays'], back_faces=f['back_faces'],
                        pose_gap_s=row['pose_gap_s'])

        approach = [row for row in scans if row['window'] in ('P22_approach',) or
                    (row['window'] in ('C4_turn', 'C4_approach') and row['entry_E1_forward_m'] > 0)]
        c4_left_T = [c for c in cands if c['side'] == 'left' and c['kind'] == 'termination'
                     and math.dist(c['e1_world'], C4_INNER) <= MATCH_M]
        rt = [c for c in cands if c['side'] == 'right' and c['kind'] == 'termination']
        by_place = {}
        for c in rt:
            by_place.setdefault(c['match'], []).append(c['i'])
        rn_by_match = {}
        for c in rn:
            rn_by_match.setdefault(c['match'], []).append(c['i'])
        before_e1 = [row for row in scans if row['window'] and row['entry_E1_forward_m'] > 0]
        if consec and len(consec) >= 2 and not false_rn and not c4_rn:
            verdict = '판별 가능'
        elif entry_scans:
            verdict = '부분'
        else:
            verdict = '판별 불가'
        report['runs'][name] = dict(
            inputs=inputs, observations_sha256=digest(HERE / f'observations_{name}.json'),
            world_check_nearest_corner_m=world_check, wall_boxes_inside_entry_gap=gap_walls,
            scans=len(scans), window_scans={w: sum(row['window'] == w for row in scans) for w, _, _ in WINDOWS},
            scans_before_E1_in_windows=len(before_e1),
            verdict=verdict,
            right_notch_scans_total=len({c['i'] for c in rn}),
            right_notch_by_match={k: len(set(v)) for k, v in rn_by_match.items()},
            right_notch_false=[{k: c[k] for k in ('t', 's', 'match', 'e1_world', 'e2_world', 'gap_m',
                                                  'wall_corner_dist_m') if k in c}
                               | ({'actor_near': c['actor_near']} if 'actor_near' in c else {}) for c in false_rn],
                               right_notch_in_C4_windows=len(c4_rn),
            right_notch_exit=[{k: c[k] for k in ('t', 's', 'gap_m', 'e1_world')} for c in rn if c['match'] == 'P22_exit'],
            entry_notch_scans=[summarize(row) for row in entry_scans],
            entry_notch_longest_consecutive=len(consec),
            entry_notch_first=summarize(entry_scans[0]) if entry_scans else None,
            right_termination_scans_by_place={k: len(set(v)) for k, v in sorted(by_place.items())},
            right_termination_entry_first=next(({k: c[k] for k in ('t', 's', 'e1_world')}
                                                | {'E1_forward_m': next(r for r in scans if r['i'] == c['i'])['entry_E1_forward_m']}
                                                for c in rt if c['match'] == 'P22_entry_edge'), None),
            C4_left_termination_scans=len({c['i'] for c in c4_left_T}),
            C4_left_termination_first=({k: c4_left_T[0][k] for k in ('t', 's', 'e1_world')}
                                       | {'C4_inner_forward_m': next(r for r in scans if r['i'] == c4_left_T[0]['i'])['C4_inner_forward_m']}
                                       if c4_left_T else None),
            id20=id20,
            approach_rows=window_rows,
        )
    (HERE / 'evaluation.json').write_bytes((json.dumps(report, ensure_ascii=False, indent=1) + '\n').encode('utf-8'))
    brief = {n: {k: v for k, v in r.items() if k in ('verdict', 'scans', 'window_scans', 'right_notch_scans_total',
                                                      'right_notch_by_match', 'right_notch_in_C4_windows',
                                                      'entry_notch_longest_consecutive', 'entry_notch_first',
                                                      'right_termination_scans_by_place', 'C4_left_termination_scans',
                                                      'world_check_nearest_corner_m', 'wall_boxes_inside_entry_gap')}
             for n, r in report['runs'].items()}
    print(json.dumps(dict(corners=[(c['name'], round(c['s'], 2), c['direction']) for c in corners_cl], runs=brief),
                     ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
