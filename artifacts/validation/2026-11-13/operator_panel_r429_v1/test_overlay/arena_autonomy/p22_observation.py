"""P22형 측벽 간극 관측기. 저장 C1 순차·운동보정 장면만 사용하는 비주행 진단 전용.

지도·진행량·마커·정답 자세·시간·절대 위치를 쓰지 않는다. 무반환/가림 광선은
연속성을 끊을 뿐 간극(자유 공간) 근거로 세지 않는다. 기준은 PLAN.md §4.
"""
import math

import numpy as np

GAP_JUMP_MIN_M = .08
SPLIT_TOL_M = .02
SEG_MIN_POINTS = 5
SEG_MIN_SPAN_M = .15
SEG_MAX_RESIDUAL_M = .02
PARALLEL_MAX_DEG = 20.
LATERAL_RANGE_M = (.20, 1.20)
E1_X_RANGE_M = (-.10, 3.00)
DEPTH_MIN_M = .25
DEPTH_MIN_RAYS = 3
TERM_WINDOW_M = (.03, .80)
COLLINEAR_MAX_DEG = 8.
COLLINEAR_MAX_OFFSET_M = .05
GAP_RANGE_M = (.40, .85)
GAP_EDGE_MARGIN_M = .03


def _fit(points):
    p = np.asarray(points, float)
    m = p.mean(axis=0)
    _, _, vh = np.linalg.svd(p - m, full_matrices=False)
    v = vh[0]
    if v[0] < 0 or (v[0] == 0 and v[1] < 0):
        v = -v
    normal = np.array([-v[1], v[0]])
    along = (p - m) @ v
    return dict(m=m, v=v, n=normal, lo=float(along.min()), hi=float(along.max()),
                span=float(np.ptp(along)), residual=float(np.max(np.abs((p - m) @ normal))),
                heading_deg=math.degrees(math.atan2(v[1], v[0])), count=len(p))


def _split(points):
    """현 기준 재귀 분할. 반환: 인덱스 구간 리스트."""
    out = []
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        if b - a + 1 < SEG_MIN_POINTS:
            continue
        p0, p1 = points[a], points[b]
        d = p1 - p0
        length = float(np.hypot(*d))
        if length < 1e-9:
            continue
        rel = points[a:b + 1] - p0
        dev = np.abs(rel[:, 0] * d[1] - rel[:, 1] * d[0]) / length
        k = int(np.argmax(dev))
        if dev[k] > SPLIT_TOL_M and 0 < k < b - a:
            stack.append((a, a + k))
            stack.append((a + k, b))
        else:
            out.append((a, b))
    return sorted(out)


def segments(scene):
    runs = []
    run = []
    prev = None
    for ray in scene.rays:
        if ray.status != 'return':
            if len(run) >= SEG_MIN_POINTS:
                runs.append(run)
            run, prev = [], None
            continue
        p = np.asarray(ray.hit_xy_m, float)
        direction = np.asarray(ray.direction_xy, float)
        rng = float(np.hypot(*(p - np.asarray(ray.ray_origin_xy_m, float))))
        if prev is not None:
            step = math.acos(max(-1., min(1., float(direction @ prev[2]))))
            jump = max(GAP_JUMP_MIN_M, 3 * rng * step)
            if ray.source_index != prev[0] + 1 or np.hypot(*(p - prev[1])) > jump:
                if len(run) >= SEG_MIN_POINTS:
                    runs.append(run)
                run = []
        run.append(p)
        prev = (ray.source_index, p, direction)
    if len(run) >= SEG_MIN_POINTS:
        runs.append(run)
    segs = []
    for run in runs:
        pts = np.asarray(run)
        for a, b in _split(pts):
            s = _fit(pts[a:b + 1])
            if s['span'] >= SEG_MIN_SPAN_M and s['residual'] <= SEG_MAX_RESIDUAL_M:
                segs.append(s)
    return segs


def _depth_hits(rays, line, side, lo, hi):
    """A 직선을 along∈(lo,hi]에서 가로질러 너머 DEPTH_MIN_M 이상에 반환한 광선. 무반환은 세지 않음."""
    m, v, n = line['m'], line['v'], line['n']
    count = 0
    unknown = 0
    for origin, hit, direction, status in rays:
        dn = float(direction @ n)
        if abs(dn) < 1e-6:
            continue
        t = float((m - origin) @ n) / dn
        if t <= 0:
            continue
        along = float((origin + t * direction - m) @ v)
        if not lo < along <= hi:
            continue
        if status == 'return':
            beyond = side * float((hit - m) @ n)
            # side=+1(왼쪽)이면 법선 방향 +, 오른쪽이면 법선 -를 "너머"로 본다.
            if beyond >= DEPTH_MIN_M and t < float(np.hypot(*(hit - origin))):
                count += 1
        elif status == 'unknown':
            unknown += 1
    return count, unknown


def observe(scene):
    segs = segments(scene)
    rays = [(np.asarray(r.ray_origin_xy_m, float),
             None if r.hit_xy_m is None else np.asarray(r.hit_xy_m, float),
             np.asarray(r.direction_xy, float), r.status) for r in scene.rays]
    result = {'segments': len(segs), 'right': {'terminations': [], 'notches': []},
              'left': {'terminations': [], 'notches': []}}
    for side_name, side in (('right', -1.), ('left', 1.)):
        walls = []
        for s in segs:
            lateral = side * float(s['m'][1])
            if abs(s['heading_deg']) <= PARALLEL_MAX_DEG and LATERAL_RANGE_M[0] <= lateral <= LATERAL_RANGE_M[1]:
                walls.append(s)
        # 법선 n=[-v_y, v_x]는 진행방향 왼쪽. 오른쪽 벽 너머는 -n 쪽.
        for a in walls:
            e1 = a['m'] + a['hi'] * a['v']
            if not E1_X_RANGE_M[0] <= e1[0] <= E1_X_RANGE_M[1]:
                continue
            term_rays, term_unknown = _depth_hits(rays, a, side, a['hi'] + TERM_WINDOW_M[0], a['hi'] + TERM_WINDOW_M[1])
            if term_rays < DEPTH_MIN_RAYS:
                continue
            base = dict(e1=[float(e1[0]), float(e1[1])], a_heading_deg=a['heading_deg'], a_span=a['span'],
                        a_residual=a['residual'], a_lateral=float(side * a['m'][1]), depth_rays=term_rays,
                        unknown_rays=term_unknown)
            result[side_name]['terminations'].append(base)
            for c in walls:
                if c is a:
                    continue
                if abs(c['heading_deg'] - a['heading_deg']) > COLLINEAR_MAX_DEG:
                    continue
                c0 = c['m'] + c['lo'] * c['v']
                c1 = c['m'] + c['hi'] * c['v']
                off = max(abs(float((c0 - a['m']) @ a['n'])), abs(float((c1 - a['m']) @ a['n'])))
                if off > COLLINEAR_MAX_OFFSET_M:
                    continue
                e2_along = float((c0 - a['m']) @ a['v'])
                gap = e2_along - a['hi']
                if not GAP_RANGE_M[0] <= gap <= GAP_RANGE_M[1]:
                    continue
                gap_rays, gap_unknown = _depth_hits(rays, a, side, a['hi'] + GAP_EDGE_MARGIN_M, e2_along - GAP_EDGE_MARGIN_M)
                if gap_rays < DEPTH_MIN_RAYS:
                    continue
                back = []
                for b in segs:
                    if b is a or b is c:
                        continue
                    rel = abs(((b['heading_deg'] - a['heading_deg']) + 90) % 180 - 90)
                    if rel < 70:
                        continue
                    beyond = side * float((b['m'] - a['m']) @ a['n'])
                    along = float((b['m'] - a['m']) @ a['v'])
                    if .05 <= beyond <= 1.2 and a['hi'] - .05 <= along <= e2_along + .05:
                        back.append(dict(span=b['span'], beyond=beyond))
                result[side_name]['notches'].append(dict(
                    base, e2=[float(c0[0]), float(c0[1])], gap_m=gap, c_span=c['span'], c_offset_m=off,
                    gap_depth_rays=gap_rays, gap_unknown_rays=gap_unknown, back_faces=back))
    return result
