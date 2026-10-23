"""관측 지지 실패의 기하 witness. 평가 전용이며 운영 판단·관측 요구를 바꾸지 않는다.

`sweep_support`와 같은 공개 함수(`sweep_queries`/`depth_support`/
`reachable_from_unknown`)로 질의별 원인을 다시 나누고, 몸체 진입 거리는
순간 회전 중심(ICR) 회전의 별도 표본으로 확인한다. 2 cm 자세 표본·두 고정
조향각의 기존 질의 범위를 그대로 따르며 연속 외곽·동적 침입·실제 정지를
증명하지 않는다.
"""
import math

import numpy as np

from arena_autonomy import observation_support as obs
from arena_autonomy.observation_lateral import outside_pieces, protected_queries, reachable_from_unknown
from arena_autonomy.linear_observation import clip_halfplane

HALF_CELL = math.pi/500
EARLY_REASONS = ('observation_invalid_state', 'current_forward_unobserved', 'observation_horizon_exceeded',
                 'observation_too_few_returns', 'current_occupancy_overrides_history')


def body_corners(cfg):
    lx, wy = cfg.length_m/2, cfg.width_m/2
    return np.array([[-lx, -wy], [lx, -wy], [lx, wy], [-lx, wy]])


def arc_body(steer, travel, cfg):
    """후륜축이 정곡률로 travel만큼 간 뒤의 몸체 꼭짓점(ICR 회전식)."""
    curvature = math.tan(steer)/cfg.wheelbase_m
    corners = body_corners(cfg)
    if curvature == 0.:
        return corners+[travel, 0.]
    icr = np.array([-cfg.wheelbase_m/2, 1/curvature])
    c, s = math.cos(curvature*travel), math.sin(curvature*travel)
    return (corners-icr)@np.array([[c, s], [-s, c]])+icr


def entry_travel(point, steer, distance, cfg, step=5e-4):
    """현재 몸체 밖 점이 이 원호 이동에서 처음 몸체 안에 드는 후륜축 거리.

    `sweep_queries`의 자세식과 독립적으로 점을 역회전해 원래 몸체와 비교한다.
    0.5 mm 표본과 기존 2 cm 표본 거리를 함께 쓰며 경계 허용치는 1e-9 m다.
    """
    curvature = math.tan(steer)/cfg.wheelbase_m
    travels = np.unique(np.r_[np.arange(0., distance, step), np.arange(.02, distance, .02), distance])
    q = np.asarray(point, dtype=float)
    if curvature == 0.:
        local = np.column_stack((q[0]-travels, np.full(len(travels), q[1])))
    else:
        icr = np.array([-cfg.wheelbase_m/2, 1/curvature])
        d, turn = q-icr, curvature*travels
        local = np.column_stack((np.cos(turn)*d[0]+np.sin(turn)*d[1],
                                 -np.sin(turn)*d[0]+np.cos(turn)*d[1]))+icr
    inside = ((abs(local[:, 0]) <= cfg.length_m/2+1e-9) & (abs(local[:, 1]) <= cfg.width_m/2+1e-9))
    hit = np.flatnonzero(inside)
    return float(travels[hit[0]]) if len(hit) else None


def distance_parts(speed, target, age, cfg):
    """기존 요구 거리의 물리 항 분해. 합계는 `sweep_support`와 같은 식이다."""
    v = max(speed, target)
    return dict(requirement_speed_mps=v, future_motion_m=v*.10+v*v/4, age_term_m=v*age,
                braking_margin_m=.06, standstill_gap_m=cfg.standstill_gap_m,
                total_m=cfg.standstill_gap_m+.06+v*(age+.10)+v*v/4)


def distance_class(travel, parts):
    """처음 진입한 거리가 어느 요구 항에서 생기는지. 누적 경계 1e-9 m."""
    if travel is None:
        return 'outside_independent_sweep'
    bound = parts['future_motion_m']
    for name, extra in (('future_motion', 0.), ('age_term', parts['age_term_m']),
                        ('braking_margin', parts['braking_margin_m']),
                        ('standstill_gap', parts['standstill_gap_m'])):
        bound += extra
        if travel <= bound+1e-9:
            return name
    return 'beyond_requirement'


def query_role(point, steer, cfg):
    """현재 몸체 기준 위치 역할. 회전 중에는 회전 중심 쪽을 inner로 부른다."""
    lx, wy = cfg.length_m/2, cfg.width_m/2
    x, y = point
    lon = 'front' if x > lx+1e-10 else ('rear' if x < -lx-1e-10 else 'side')
    if abs(y) <= wy+1e-10:
        lat = 'center'
    elif steer == 0.:
        lat = 'left' if y > 0 else 'right'
    else:
        lat = 'inner' if (y > 0) == (steer > 0) else 'outer'
    behind = lat != 'center' and x < -cfg.wheelbase_m/2
    return f'{lon}_{lat}'+('_behind_rear_axle' if behind else '')


def containing_rays(scene, point):
    """점을 포함하는 비반환(unknown/masked) 광선 쐐기(±π/500)."""
    q = np.asarray(point, dtype=float)
    rows = []
    for ray in scene.rays:
        if ray.status == 'return':
            continue
        v = q-np.asarray(ray.ray_origin_xy_m)
        d = ray.direction_xy
        angle = math.atan2(d[0]*v[1]-d[1]*v[0], d[0]*v[0]+d[1]*v[1])
        if abs(angle) <= HALF_CELL+1e-12:
            rows.append(ray)
    return rows


def bracket(scene, point):
    """`depth_support`와 같은 방식으로 고른 양쪽 광선의 상태·깊이 여유(패딩 전)."""
    q = np.asarray(point, dtype=float)
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    delta = q-origins
    angles = np.arctan2(directions[:, 0]*delta[:, 1]-directions[:, 1]*delta[:, 0],
                        directions[:, 0]*delta[:, 0]+directions[:, 1]*delta[:, 1])
    lower = int(np.argmin(np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf)))
    upper = int(np.argmin(np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf)))
    rows = []
    for index in (lower, upper):
        ray = scene.rays[index]
        radius = float(np.linalg.norm(delta[index]))
        depth = math.dist(ray.ray_origin_xy_m, ray.hit_xy_m) if ray.status == 'return' else None
        rows.append(dict(index=ray.source_index, status=ray.status, angle_rad=float(angles[index]),
                         query_radius_m=radius, return_depth_m=depth))
    return rows


def closest_on_polygon(point, polygon):
    q = np.asarray(point, dtype=float)
    p = np.asarray(polygon, dtype=float)
    start, step = p, np.roll(p, -1, axis=0)-p
    length2 = np.maximum(np.sum(step*step, axis=1), 1e-30)
    t = np.clip(np.sum((q-start)*step, axis=1)/length2, 0., 1.)
    nearest = start+t[:, None]*step
    distance = np.linalg.norm(nearest-q, axis=1)
    cross = step[:, 0]*(q[1]-start[:, 1])-step[:, 1]*(q[0]-start[:, 0])
    area = abs(np.sum(p[:, 0]*np.roll(p[:, 1], -1)-np.roll(p[:, 0], -1)*p[:, 1]))
    if area > 1e-14 and (np.all(cross >= -1e-12) or np.all(cross <= 1e-12)):
        return q.copy(), 0.
    index = int(np.argmin(distance))
    return nearest[index], float(distance[index])


def lateral_witness(scene, point, cfg):
    """`reachable_from_unknown`과 같은 출발 영역(현재 몸체 밖 미관측 쐐기)의 최근접 출발점."""
    q = np.asarray(point, dtype=float)
    best = None
    for ray in scene.rays:
        if ray.status == 'return':
            continue
        budget = cfg.object_motion_bound_mps*(scene.t_ref_s-ray.acquired_at_s)
        r = budget+.01
        box = [(q[0]-r, q[1]-r), (q[0]+r, q[1]-r), (q[0]+r, q[1]+r), (q[0]-r, q[1]+r)]
        dx, dy = ray.direction_xy
        c, s = math.cos(HALF_CELL), math.sin(HALF_CELL)
        cut = clip_halfplane(clip_halfplane(box, ray.ray_origin_xy_m, (c*dx+s*dy, -s*dx+c*dy), 1),
                             ray.ray_origin_xy_m, (c*dx-s*dy, s*dx+c*dy), -1)
        if not cut:
            continue
        for piece in outside_pieces(cut, cfg.length_m, cfg.width_m):
            start, separation = closest_on_polygon(q, piece)
            if separation <= budget+1e-12 and (best is None or separation < best['separation_m']):
                best = dict(ray_index=ray.source_index, ray_status=ray.status,
                            ray_origin_xy_m=list(ray.ray_origin_xy_m),
                            ray_bearing_deg=math.degrees(math.atan2(dy, dx)),
                            ray_acquired_at_s=ray.acquired_at_s, possible_start_xy_m=start.tolist(),
                            separation_m=separation, motion_budget_m=budget)
    return best


def ray_row(ray, scene):
    return dict(ray_index=ray.source_index, ray_status=ray.status, ray_origin_xy_m=list(ray.ray_origin_xy_m),
                ray_bearing_deg=math.degrees(math.atan2(ray.direction_xy[1], ray.direction_xy[0])),
                ray_acquired_at_s=ray.acquired_at_s, source_first_s=scene.source_first_s)


def failure_witnesses(sources, speed, target, steer, previous, cfg, protected_pose):
    """실패 질의마다 역할·처음 진입 거리·원인·광선 witness를 만든다.

    반환 `consistent`는 이 재계산의 지지 질의 수가 실제 `sweep_support`와
    같은지다. 다르면 witness를 근거로 쓰지 않는다.
    """
    result = obs.sweep_support(sources, speed, target, steer, previous, cfg,
                               protected_sensor_pose=protected_pose)
    if result['reason'] in EARLY_REASONS or result['supported']:
        return result, [], True
    current = sources[0]
    distance = result['required_distance_m']
    angles = tuple(dict.fromkeys((steer, previous)))
    queries = np.unique(np.vstack([obs.sweep_queries(current, a, distance, cfg)[0] for a in angles]), axis=0)
    protected = protected_queries(queries, protected_pose)
    supported = np.zeros(len(queries), dtype=bool)
    depth_any = np.zeros(len(queries), dtype=bool)
    valid = []
    for s in sources:
        if (s.source_epoch != current.source_epoch or s.t_ref_s != current.t_ref_s
                or s.source_first_s > current.source_first_s):
            continue
        pad = np.array([.025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*(s.t_ref_s-r.acquired_at_s)
                        for r in s.rays])
        depth = obs.depth_support(s, queries, pad)
        reach = np.zeros(len(queries), dtype=bool)
        check = np.flatnonzero(depth & protected)
        reach[check] = reachable_from_unknown(s, queries[check], cfg)
        supported |= depth & ~reach
        depth_any |= depth
        valid.append((s, depth, reach))
    consistent = int(supported.sum()) == result['supported_queries'] and len(queries) == result['queries']
    parts = distance_parts(speed, target, current.t_ref_s-current.source_first_s, cfg)
    rows = []
    for j in np.flatnonzero(~supported):
        q = queries[j]
        entries = {a: entry_travel(q, a, distance, cfg) for a in angles}
        found = [a for a in angles if entries[a] is not None]
        arc = min(found, key=lambda a: entries[a]) if found else angles[0]
        row = dict(query_xy_m=q.tolist(), protected=bool(protected[j]), arc_steer_rad=arc,
                   entry_travel_m=entries[arc], distance_class=distance_class(entries[arc], parts),
                   role=query_role(q, arc, cfg))
        inside = containing_rays(current, q)
        if not depth_any[j]:
            neighbours = bracket(current, q)
            if any(r.status == 'masked' for r in inside):
                cause, ray = 'masked_wedge', next(r for r in inside if r.status == 'masked')
            elif inside:
                cause, ray = 'unknown_wedge', inside[0]
            elif any(r['status'] != 'return' for r in neighbours):
                # 쐐기 ±반 셀 밖이어도 양쪽 인접 깊이 중 한쪽이 결측일 수 있다.
                # 해당 실제 광선을 기록하며 빈 공간으로 보간하지 않는다.
                missing = next(r for r in neighbours if r['status'] != 'return')
                ray = current.rays[missing['index']]
                cause = 'masked_bracket' if ray.status == 'masked' else 'unknown_bracket'
            else:
                cause, ray = 'bracket_unsupported', None
            row.update(cause=cause, bracket=neighbours)
            if ray is not None:
                row.update(ray_row(ray, current))
        else:
            s = next(s for s, depth, reach in valid if depth[j])
            witness = lateral_witness(s, q, cfg)
            row.update(cause='lateral_reach', source_first_s=s.source_first_s,
                       inside_current_unknown_wedge=bool(inside), **(witness or {}),
                       lateral_witness_found=witness is not None)
        rows.append(row)
    return {**result, 'distance_parts': parts}, rows, consistent
