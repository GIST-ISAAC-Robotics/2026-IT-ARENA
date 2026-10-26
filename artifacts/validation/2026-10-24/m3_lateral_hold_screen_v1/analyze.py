"""M3 측후방 '중앙 복귀 보류' 후보의 오프라인 선별. 주행·운영 코드 미연결.

후보 판단(candidate_state)은 저장된 원시 스캔·운동 이력으로 재구성한 장면과
당시 제어기가 출력한 경로점만 받는다. 정답 자세·actor 일정·지도는 그 함수에
들어가지 않으며, 아래 evaluation_* 함수에서 사후 평가에만 쓴다. 이 계산은
폐루프 주행·통과·안전 판정이 아니다. 저장 궤적은 원래 정책의 것이다.
"""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from shapely.geometry import Polygon, Point

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call, thaw  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402
import g1_traffic_eval as ev  # noqa: E402  (평가 전용 중심선)

# 기존 운영값을 그대로 읽어 쓴다. 새 판정선이 아니다.
WALL_OFFSET = .425          # FollowConfig.wall_offset_m
AGREEMENT = .08             # FollowConfig.boundary_agreement_m
INTERIOR = WALL_OFFSET-AGREEMENT
ROAD_HALF = .225            # 공식 본선 0.45 m (트랙 사양, 센서 관측 아님)
LENGTH, WIDTH = .20, .15
FIXED_PAD, MOTION_BOUND = .025+.03, .5   # per_ray_padding(.025+measurement, 0.5 m/s)
# 이 선별에서만 쓰는 후보 매개변수(사전 고정, 결과를 보고 바꾸지 않음).
BAND_BEHIND, BAND_AHEAD = .60, .35       # 자차 중심 기준 경로 호길이 창
EXTENSION = .60                          # 경로 양 끝의 분류용 선형 연장(자유 공간 아님)
MIN_POINTS = 3                           # 단일 잡음 반환으로 보류하지 않기 위한 최소 개수
MIN_OFFSET = .02                         # 이보다 작은 중앙 편차는 보류해도 의미가 없음

TRACES = {
    'e3_onroad_g1': 'artifacts/validation/2026-10-24/e3_onroad_v1/g1',
    'e3_onroad_r0plus': 'artifacts/validation/2026-10-24/e3_onroad_v1/r0plus',
    'solo_g1_20261016': 'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run',
}
MODULES = ['src/arena_autonomy/arena_autonomy/follow_trace.py', 'src/arena_autonomy/arena_autonomy/scene_motion.py',
           'src/arena_autonomy/arena_autonomy/local_scene.py', 'src/arena_autonomy/arena_autonomy/lidar_motion.py',
           'src/arena_autonomy/arena_autonomy/lidar_observation.py', 'scripts/g1_traffic_eval.py',
           'src/arena_gazebo/worlds/it_arena_official/centerline.csv',
           'src/arena_gazebo/worlds/it_arena_official/scene.json']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


# ---------------------------------------------------------------- 후보(센서·당시 경로만)
def path_frame(path):
    p = np.asarray(path, dtype=float).reshape(-1, 2)
    keep = np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-9]
    p = p[keep]
    if len(p) < 3:
        return None
    t0 = (p[1]-p[0])/np.linalg.norm(p[1]-p[0])
    t1 = (p[-1]-p[-2])/np.linalg.norm(p[-1]-p[-2])
    q = np.vstack([p[0]-EXTENSION*t0, p, p[-1]+EXTENSION*t1])
    seg = np.diff(q, axis=0)
    length = np.linalg.norm(seg, axis=1)
    start = np.r_[0., np.cumsum(length)][:-1]-EXTENSION
    return q, seg, length, start


def project(points, frame):
    q, seg, length, start = frame
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    rel = points[:, None, :]-q[None, :-1, :]
    frac = np.clip(np.sum(rel*seg[None], axis=2)/(length**2)[None], 0., 1.)
    foot = q[None, :-1, :]+frac[:, :, None]*seg[None]
    index = np.argmin(np.linalg.norm(points[:, None, :]-foot, axis=2), axis=1)
    rows = np.arange(len(points))
    tangent = seg[index]/length[index][:, None]
    offset = points-foot[rows, index]
    lateral = tangent[:, 0]*offset[:, 1]-tangent[:, 1]*offset[:, 0]   # +: 경로 진행 방향 왼쪽
    along = start[index]+frac[rows, index]*length[index]
    extended = (index == 0) | (index == len(seg)-1)
    return along, lateral, extended, tangent


def candidate_state(scene, path):
    """측후방의 '경계로 설명되지 않는 반환'이 중앙 복귀 방향에 있는지만 본다.

    입력: 순차 시각으로 운동 보정한 LocalScene, 당시 제어기 경로(base_link).
    정체(차/벽)·속도를 추정하지 않으며 미관측을 비어 있다고 보지 않는다.
    """
    frame = path_frame(path) if path is not None else None
    if frame is None:
        return {'status': 'no_path'}
    rays = [r for r in scene.rays if r.status == 'return']
    points = np.asarray([r.hit_xy_m for r in rays], dtype=float).reshape(-1, 2)
    along, lateral, extended, _ = project(points, frame)
    ego_s, ego_l, _, ego_t = project(np.zeros((1, 2)), frame)
    ego_s, ego_l = float(ego_s[0]), float(ego_l[0])
    psi = -math.atan2(ego_t[0, 1], ego_t[0, 0])            # 경로 대비 자차 yaw
    half = (LENGTH*abs(math.sin(psi))+WIDTH*abs(math.cos(psi)))/2
    centering = 0 if abs(ego_l) < .005 else (1 if ego_l < 0 else -1)   # 경로 쪽으로 가는 횡방향 부호
    ds = along-ego_s
    window = (ds >= -BAND_BEHIND) & (ds <= BAND_AHEAD) & (np.abs(lateral) < INTERIOR)
    sides = {}
    for name, sign in (('left', 1), ('right', -1)):
        mask = window & (sign*(lateral-ego_l) > 0)
        gap = sign*(lateral[mask]-ego_l)-half
        sides[name] = {'points': int(np.sum(mask)), 'extended_points': int(np.sum(mask & extended)),
                       'min_lateral_gap_m': float(np.min(gap)) if len(gap) else None,
                       'ds_at_min_m': float(ds[mask][np.argmin(gap)]) if len(gap) else None}
    # 센서 방위 기준 측후방(90~150°) 무응답·마스크 개수. 비어 있다는 증거로 쓰지 않는다.
    unknown = {'left': 0, 'right': 0}
    masked = 0
    for ray in scene.rays:
        bearing = _sensor_bearing_deg(ray)
        if ray.status == 'masked':
            masked += 1
        elif ray.status == 'unknown' and 90 <= abs(bearing) <= 150:
            unknown['left' if bearing > 0 else 'right'] += 1
    toward = None if centering == 0 else ('left' if centering > 0 else 'right')
    active = bool(toward and abs(ego_l) >= MIN_OFFSET and sides[toward]['points'] >= MIN_POINTS)
    return {'status': 'path', 'ego_offset_from_path_m': ego_l, 'yaw_from_path_rad': psi,
            'ego_half_span_m': half, 'centering_toward': toward, 'sides': sides,
            'side_rear_unknown': unknown, 'masked_rays': masked, 'hold_active': active,
            'ego_inside_road_by_wall_offset': abs(ego_l)+half <= ROAD_HALF}


def _sensor_bearing_deg(ray):
    # 원시 센서 방위(운동 보정 회전 제외)로 섹터를 나눈다. 점 위치는 바꾸지 않는다.
    angle = SCAN_META['angle_min']+ray.source_index*SCAN_META['angle_increment']
    return math.degrees(math.remainder(angle, math.tau))


SCAN_META = {}


# ---------------------------------------------------------------- 평가 전용
def rectangle(x, y, yaw, length=LENGTH, width=WIDTH):
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [(+length/2, +width/2), (+length/2, -width/2), (-length/2, -width/2), (-length/2, +width/2)]
    return Polygon([(x+c*a-s*b, y+s*a+c*b) for a, b in corners])


def nearest_row(rows, times, t):
    from bisect import bisect_right
    i = bisect_right(times, t)-1
    return rows[i] if i >= 0 and t-times[i] <= .05 else None


def evaluation_actor(ego_row, actor_row, scene, state):
    """정답 자세로 측후방 반환이 실제 옆차에서 왔는지 사후 분류한다(후보 입력 아님)."""
    _, x, y, yaw = ego_row
    _, ax, ay, ayaw = actor_row
    c, s = math.cos(yaw), math.sin(yaw)
    rx, ry = c*(ax-x)+s*(ay-y), -s*(ax-x)+c*(ay-y)
    body = rectangle(rx, ry, ayaw-yaw)
    pts = [r.hit_xy_m for r in scene.rays if r.status == 'return']
    near = sum(body.distance(Point(p)) <= .08 for p in pts)
    in_band = (-BAND_BEHIND-.1 <= rx <= BAND_AHEAD+.1)
    return {'actor_rel_x_m': rx, 'actor_rel_y_m': ry, 'actor_in_band_eval': bool(in_band),
            'returns_within_8cm_of_actor_eval': int(near)}


def run_trace(name, base):
    base = ROOT/base
    decisions = base/'follow_decisions.jsonl'
    ego = [json.loads(l) for l in (base/'ego_poses.jsonl').read_text().splitlines()]
    ego_t = [r[0] for r in ego]
    actor, actor_t = None, None
    extra = base/'additional_actors.json'
    if extra.exists():
        actor = json.loads(extra.read_text())['poses']['eval_e3_side']
        actor_t = [r[0] for r in actor]
    line = None
    if name.startswith('solo'):
        world = ROOT/'src/arena_gazebo/worlds/it_arena_official'
        route = np.genfromtxt(world/'centerline.csv', delimiter=',', names=True)
        lap = json.loads((world/'scene.json').read_text(encoding='utf-8'))['track']['lap_length_m']
        line = ev.Centerline(route['x_m'], route['y_m'], route['s_m'], lap)
    motion = None
    rows, skipped = [], {'input_error': 0, 'no_reference': 0}
    with decisions.open() as stream:
        for raw in stream:
            event = json.loads(raw)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                continue
            if event['kind'] != 'call':
                continue
            call = thaw(event['data'])
            inp, out = call['input'], call['output']
            if inp['reference_s'] is None:
                skipped['no_reference'] += 1
                continue
            data, scan, history = restore_call(inp, motion)
            SCAN_META.update(angle_min=scan.angle_min, angle_increment=scan.angle_increment)
            try:
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            except (MotionUnavailable, ValueError):
                skipped['input_error'] += 1
                continue
            detail = out[2]
            state = candidate_state(scene, detail.get('path_points'))
            row = {'t': data['reference_s'], 'reason': detail.get('reason'), 'speed_cmd': out[0],
                   'steer_cmd': out[1], 'measured_speed': data['speed_mps'],
                   'scan_age_at_ref_s': data['reference_s']-scene.source_first_s, **state}
            e = nearest_row(ego, ego_t, data['reference_s'])
            if actor is not None and e is not None:
                a = nearest_row(actor, actor_t, data['reference_s'])
                if a is not None:
                    row.update(evaluation_actor(e, a, scene, state))
            if line is not None and e is not None:
                s_val, dist = line.nearest_s(e[1], e[2])
                row['eval_track_s_m'] = s_val
            rows.append(row)
    return rows, skipped, [str(p.relative_to(ROOT)) for p in (decisions, base/'ego_poses.jsonl')] + (
        [str(extra.relative_to(ROOT))] if extra.exists() else [])


# ---------------------------------------------------------------- 해석적 표
def analytic(scan_ages):
    out = {}
    # 1) 횡여유 예산(공식 도로 0.45 m, 명목 20x15 cm)
    def half_span(theta):
        return (LENGTH*abs(math.sin(theta))+WIDTH*abs(math.cos(theta)))/2
    actor_offset = .13
    out['lateral_budget'] = {
        'road_width_m': 2*ROAD_HALF, 'car_width_m': WIDTH,
        'two_cars_both_on_edge_gap_m': 2*ROAD_HALF-2*WIDTH,
        'registered_actor_offset_m': actor_offset,
        'max_gap_ego_on_road_edge_m': (actor_offset-WIDTH/2)-(-ROAD_HALF+WIDTH),
        'gap_ego_at_slot_minus_0p09_m': (actor_offset-WIDTH/2)-(-.09+WIDTH/2),
        'gap_ego_centered_m': (actor_offset-WIDTH/2)-WIDTH/2,
        'grid_columns_gap_m': .18-WIDTH,
        'yaw_added_half_span_m': {f'{d:g}deg': half_span(math.radians(d))-WIDTH/2 for d in (0, 2, 3.9, 5.1, 7.7, 10.5)},
    }
    # 2) 정지 출발 Pure Pursuit 조향(look-ahead 0.30 m, 후륜축 기준) vs 완만한 수렴의 최대 heading
    wheelbase = .135
    pp = {}
    for d in (.03, .06, .09):
        x = math.sqrt(.30**2-d**2)
        pp[f'{d:g}m'] = math.atan(wheelbase*2*d/(x*x+d*d))
    out['pure_pursuit_standstill_steering_rad'] = pp
    out['cubic_blend_peak_heading_deg'] = {f'{d:g}m_over_{L:g}m': math.degrees(math.atan(1.5*d/L))
                                          for d in (.09,) for L in (.3, .6, 1.0, 1.5, 2.0)}
    # 3) 측방 per-ray 팽창: 시뮬레이터 스캔 순서(−180°→+180°, 0.1 s)에서 방위별 나이
    q = np.percentile(scan_ages, [0, 50, 100])
    pad = {}
    for b in (-150, -120, -90, 90, 120, 150):
        acquisition = (math.radians(b)+math.pi)/(2*math.pi)*.1
        pad[f'{b}deg'] = {k: FIXED_PAD+MOTION_BOUND*(float(v)-acquisition)
                          for k, v in zip(('min_ref', 'median_ref', 'max_ref'), q)}
    out['per_ray_padding_m_by_bearing'] = pad
    out['scan_reference_offset_s'] = dict(zip(('min', 'median', 'max'), map(float, q)))
    # 4) 후방 ±30° 마스크에서 뒤따라오는 평행 차의 첫 관측(앞면 먼 모서리 기준)
    lidar_x, tan30 = .06, math.tan(math.radians(30))
    vis = {}
    for gap in (.03, .07, .13, .15):
        near = WIDTH/2+gap
        front_face_x = lidar_x-(near+WIDTH)/tan30   # 앞면의 먼 모서리가 150° 경계 안에 들어오는 위치
        side_face_x = lidar_x-near/tan30            # 옆면 가까운 쪽이 보이기 시작하는 위치
        lead = -LENGTH/2-front_face_x               # 자차 뒤끝(−0.10)까지 남은 거리
        vis[f'gap_{gap:g}m'] = {'first_visible_front_x_m': front_face_x, 'side_face_visible_x_m': side_face_x,
                                'distance_to_ego_rear_m': lead,
                                'warning_s': {f'dv_{dv:g}': lead/dv for dv in (.1, .45, 1.0)}}
    out['rear_mask_first_visibility'] = vis
    # 5) 반응·제동 중 상대 도달 범위(가정값 명시)
    tau, decel = .20, 2.0   # 가정: 최대 측정 나이+제어 주기+구동 지연, braking_speed의 임시 감속도
    reach = {}
    for v in (.3, .5, 1.0):
        stop_t = tau+v/decel
        reach[f'ego_{v:g}mps'] = {'stop_time_s': stop_t, 'stop_distance_m': v*tau+v*v/(2*decel),
                                  'object_reach_isotropic_0p5_m': .5*stop_t,
                                  'object_lateral_reach_m': {f'{lat:g}mps': lat*stop_t for lat in (.05, .1, .2)}}
    out['future_reach_assumption'] = {'tau_s': tau, 'decel_mps2': decel, 'status': 'assumption, not measured',
                                      'rows': reach}
    return out


def summarize(name, rows):
    with_path = [r for r in rows if r['status'] == 'path']
    active = [r for r in with_path if r['hold_active']]
    summary = {'decisions': len(rows), 'with_path': len(with_path), 'hold_active': len(active),
               'offset_ge_2cm': sum(abs(r['ego_offset_from_path_m']) >= MIN_OFFSET for r in with_path),
               'outside_road_by_wall_offset': sum(not r['ego_inside_road_by_wall_offset'] for r in with_path)}
    if name.startswith('e3'):
        inband = [r for r in with_path if r.get('actor_in_band_eval')]
        summary['eval_actor_in_band'] = len(inband)
        first_active = next((r for r in active), None)
        summary['first_hold_active'] = compact(first_active)
        # 정답 기준 옆차 쪽 반환이 실제로 있었는데 보류가 꺼진 이유(평가 전용 분류)
        summary['eval_inband_inactive_reasons'] = _counts(
            'centering_none' if r['centering_toward'] is None else
            ('offset_lt_2cm' if abs(r['ego_offset_from_path_m']) < MIN_OFFSET else
             ('object_not_on_centering_side' if r['sides'][r['centering_toward']]['points'] < MIN_POINTS else 'other'))
            for r in inband if not r['hold_active'])
        summary['eval_active_with_actor_returns'] = sum(r.get('returns_within_8cm_of_actor_eval', 0) > 0 for r in active)
        # 정답상 측후방 창 안인데 그 차 근처 반환이 0인 구간(후방 마스크·가림). 평가 전용.
        summary['eval_in_band_without_actor_returns'] = _intervals(
            rows, lambda r: r.get('actor_in_band_eval') and r.get('returns_within_8cm_of_actor_eval', 0) == 0)
        summary['eval_centered_with_actor_returns'] = _intervals(
            rows, lambda r: r['status'] == 'path' and r['centering_toward'] is None
            and r.get('returns_within_8cm_of_actor_eval', 0) > 0)
    else:
        summary['active_track_s_eval'] = _bins([r.get('eval_track_s_m') for r in active])
        summary['active_examples'] = [compact(r) for r in active[:: max(1, len(active)//8)][:8]]
        summary['active_min_gap_m'] = sorted(round(r['sides'][r['centering_toward']]['min_lateral_gap_m'], 4) for r in active)[:10]
        summary['active_extended_only'] = sum(r['sides'][r['centering_toward']]['extended_points'] ==
                                              r['sides'][r['centering_toward']]['points'] for r in active)
    return summary


def post_hoc(rows):
    """결과를 본 뒤 추가한 진단. 사전 등록 후보가 아니며 독립 자료 검증 전 채택 근거가 아니다."""
    with_path = [r for r in rows if r['status'] == 'path']

    def active(r, no_extension, heading):
        side = r['centering_toward']
        if not side or abs(r['ego_offset_from_path_m']) < MIN_OFFSET:
            return False
        if heading and abs(r['yaw_from_path_rad']) > math.radians(15):
            return False
        points = r['sides'][side]['points']-(r['sides'][side]['extended_points'] if no_extension else 0)
        return points >= MIN_POINTS
    def interior(r, no_extension=False):
        return max(r['sides'][s]['points']-(r['sides'][s]['extended_points'] if no_extension else 0)
                   for s in ('left', 'right')) >= MIN_POINTS
    straight = [r for r in with_path if abs(r['yaw_from_path_rad']) <= math.radians(15)]
    out = {'abs_yaw_from_path_gt_15deg': len(with_path)-len(straight),
           'any_side_interior_ge_min_points': sum(interior(r) for r in with_path),
           'any_side_interior_track_s_eval': _bins([r.get('eval_track_s_m') for r in with_path if interior(r)]),
           'any_side_interior_heading_15deg_no_extension': sum(interior(r, True) for r in straight),
           'any_side_interior_heading_15deg_no_extension_track_s_eval':
               _bins([r.get('eval_track_s_m') for r in straight if interior(r, True)])}
    for label, no_ext, heading in (('original', False, False), ('no_extension', True, False),
                                   ('heading_15deg', False, True), ('both', True, True)):
        hits = [r for r in with_path if active(r, no_ext, heading)]
        out[label] = {'active': len(hits), 'first_t': hits[0]['t'] if hits else None,
                      'active_track_s_eval': _bins([r.get('eval_track_s_m') for r in hits])}
    return out


def _intervals(rows, predicate, joint=.05):
    spans = []
    for r in rows:
        if not predicate(r):
            continue
        if spans and r['t']-spans[-1][1] <= joint:
            spans[-1][1], spans[-1][2] = r['t'], spans[-1][2]+1
        else:
            spans.append([r['t'], r['t'], 1])
    return [[round(a, 3), round(b, 3), n] for a, b, n in spans]


def _counts(values):
    out = {}
    for v in values:
        out[v] = out.get(v, 0)+1
    return out


def _bins(values, width=2.):
    out = {}
    for v in values:
        if v is None:
            continue
        key = f'{width*math.floor(v/width):.0f}-{width*math.floor(v/width)+width:.0f}'
        out[key] = out.get(key, 0)+1
    return out


def compact(r):
    if r is None:
        return None
    keep = ('t', 'reason', 'speed_cmd', 'steer_cmd', 'measured_speed', 'ego_offset_from_path_m', 'yaw_from_path_rad',
            'centering_toward', 'hold_active', 'side_rear_unknown', 'actor_rel_x_m', 'actor_rel_y_m',
            'actor_in_band_eval', 'returns_within_8cm_of_actor_eval', 'eval_track_s_m')
    out = {k: r.get(k) for k in keep if k in r}
    if r.get('sides'):
        out['sides'] = r['sides']
    return out


def main():
    result = {'scope': 'offline screening on stored inputs; candidate uses scene + recorded controller path only; '
                       'ground truth only in *_eval fields; not closed-loop, not a driving/safety verdict',
              'parameters': {'band_behind_m': BAND_BEHIND, 'band_ahead_m': BAND_AHEAD, 'extension_m': EXTENSION,
                             'min_points': MIN_POINTS, 'min_offset_m': MIN_OFFSET, 'interior_m': INTERIOR,
                             'road_half_m_track_spec': ROAD_HALF},
              'traces': {}, 'sources': {}}
    ages = []
    for name, base in TRACES.items():
        rows, skipped, files = run_trace(name, base)
        ages += [r['scan_age_at_ref_s'] for r in rows]
        result['traces'][name] = {'summary': summarize(name, rows), 'skipped': skipped,
                                  'post_hoc_diagnostic': post_hoc(rows)}
        if name.startswith('e3'):
            result['traces'][name]['timeline'] = [compact(r) for r in rows if 3.9 <= r['t'] <= 6.8 or 13.8 <= r['t'] <= 15.2]
        for f in files:
            result['sources'][f] = sha(ROOT/f)
    result['analytic'] = analytic(np.asarray(ages))
    for m in MODULES + [str(Path(__file__).resolve().relative_to(ROOT))]:
        result['sources'][m] = sha(ROOT/m)
    (OUT/'report.json').write_text(json.dumps(result, indent=1, ensure_ascii=False), encoding='utf-8', newline='\n')
    brief = {k: {key: v['summary'][key] for key in ('decisions', 'with_path', 'hold_active', 'offset_ge_2cm',
                                                    'outside_road_by_wall_offset')} | {'post_hoc': v['post_hoc_diagnostic']}
             for k, v in result['traces'].items()}
    print(json.dumps(brief, indent=1, ensure_ascii=False))


if __name__ == '__main__':
    main()
