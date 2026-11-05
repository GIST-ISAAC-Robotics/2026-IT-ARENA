"""[격리 시험 전용] 직각 벽 꺾은선 경로 후보를 ws2 사본에만 적용한다. 공용 트리에는 적용하지 않는다.

변경 1 (local_path.py): 3차 적합 잔차가 커서 'wall_shape_uncertain'이 되는 관측 벽에 한해,
  같은 관측 표본을 꺾은선(Douglas-Peucker 2 cm)으로 나누고 각 직선 구간을 0.425 m 평행 이동한 뒤
  45–135° 모서리를 반경 0.45 m(최소 0.40 m) 원호로 잇는다. 관측 끝 밖으로 외삽하지 않으며,
  다음 구간 관측이 원호에 부족하면 평행선 교점에서, 마지막 구간은 관측 끝에서 offset 앞에서 경로를 끝낸다.
변경 2 (local_follow.py): 선호 벽 경로가 승인되지 않았고 반대 벽이 위 꺾은선 모델로 긴 경계 조건을
  만족하며 직전 승인 경로(기억)와 일치하면 그 반대 벽 경로를 쓴다(기존 boundary_bridge 직전).
  기억이 없으면 적용하지 않는다. 확인 횟수·점유·속도/제동·독립 보호층은 그대로다.
사용: python3 apply_trial.py <ws2 루트>
"""
import sys
from pathlib import Path

ws = Path(sys.argv[1])
lp = ws/'src/arena_autonomy/arena_autonomy/local_path.py'
lf = ws/'src/arena_autonomy/arena_autonomy/local_follow.py'

HELPER = '''

def _polyline_offset_path(samples, sign, offset, fillet=.45, min_radius=.40, tolerance=.02,
                          min_turn_deg=20., corner_range_deg=(45., 135.), min_segment=.10):
    """[격리 직각 코너 시험 전용] 꺾은선 관측 벽의 평행 경로. 실제 관측 표본만 쓰고 외삽하지 않는다."""
    from shapely.geometry import LineString
    pts = np.asarray(samples, dtype=float)
    if len(pts) < 12:
        return None
    simple = np.asarray(LineString(pts).simplify(tolerance, preserve_topology=False).coords)
    # 잡음으로 생긴 작은 꺾임은 이웃 구간과 합친다.
    changed = True
    while changed and len(simple) > 2:
        changed = False
        for i in range(1, len(simple)-1):
            a, b = simple[i]-simple[i-1], simple[i+1]-simple[i]
            turn = math.degrees(math.atan2(a[0]*b[1]-a[1]*b[0], a @ b))
            if abs(turn) < min_turn_deg:
                simple = np.delete(simple, i, axis=0)
                changed = True
                break
    if len(simple) < 3 or len(simple) > 5:
        return None
    seg = np.diff(simple, axis=0)
    lengths = np.linalg.norm(seg, axis=1)
    if np.any(lengths < min_segment):
        return None
    t = seg/lengths[:, None]
    n = sign*np.column_stack((t[:, 1], -t[:, 0]))
    turns = [math.degrees(math.atan2(t[i, 0]*t[i+1, 1]-t[i, 1]*t[i+1, 0], t[i] @ t[i+1])) for i in range(len(t)-1)]
    if any(not corner_range_deg[0] <= abs(x) <= corner_range_deg[1] for x in turns):
        return None
    # 평행선 교점(모서리)과 양 끝.
    q = [simple[0]+offset*n[0]]
    for i in range(len(t)-1):
        p0, p1 = simple[i+1]+offset*n[i], simple[i+1]+offset*n[i+1]
        m = np.column_stack((t[i], -t[i+1]))
        lam = np.linalg.solve(m, p1-p0)
        q.append(p0+lam[0]*t[i])
    q.append(simple[-1]+offset*n[-1])
    q = np.asarray(q)
    avail = np.linalg.norm(np.diff(q, axis=0), axis=1)
    if np.any(np.sum(np.diff(q, axis=0)*t, axis=1) <= .05):
        return None
    out = [q[0]]
    used_before = 0.
    for k in range(len(turns)):
        half = math.tan(math.radians(abs(turns[k]))/2)
        # 앞 구간에서 쓸 수 있는 길이와 뒤 구간 관측 길이(다음 모서리가 있으면 절반)를 넘지 않는다.
        room_in = avail[k]-used_before
        room_out = avail[k+1] if k == len(turns)-1 else avail[k+1]/2
        radius = min(fillet, room_in/half, room_out/half)
        if radius < min_radius:
            # 다음 구간 관측이 짧다. 두 평행선의 교점(관측 벽에서 offset 떨어진 마지막 점)에서 끝낸다.
            out.append(q[k+1])
            return _densify(out)
        tan_len = radius*half
        start, end = q[k+1]-t[k]*tan_len, q[k+1]+t[k+1]*tan_len
        out.append(start)
        side = 1. if turns[k] > 0 else -1.
        normal_in = side*np.array([-t[k, 1], t[k, 0]])
        center = start+radius*normal_in
        a0 = math.atan2(*(start-center)[::-1])
        sweep = math.radians(turns[k])
        for f in np.linspace(0., 1., 12)[1:-1]:
            ang = a0+f*sweep
            out.append(center+radius*np.array([math.cos(ang), math.sin(ang)]))
        out.append(end)
        used_before = tan_len
    # 마지막 관측 구간 뒤에 다른 벽이 이어질 수 있으므로 관측 끝에서 offset만큼 앞에서 끝낸다.
    # (다음 벽이 짧게 보일 때의 교점 끝과 같은 위치가 되어 스캔 사이 경로 끝이 흔들리지 않는다.)
    end = q[-1]-t[-1]*offset
    if (end-out[-1]) @ t[-1] > .02:
        out.append(end)
    return _densify(out)


def _densify(points, step=.02):
    pts = np.asarray(points, dtype=float)
    keep = np.r_[True, np.linalg.norm(np.diff(pts, axis=0), axis=1) > 1e-9]
    pts = pts[keep]
    if len(pts) < 2:
        return None
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    if s[-1] < .05:
        return None
    grid = np.linspace(0., s[-1], max(3, int(s[-1]/step)+1))
    return np.column_stack([np.interp(grid, s, pts[:, i]) for i in (0, 1)])


def local_path('''

OLD_FIT = '''    if np.percentile(residual, 80) > .06:
        return None, {"reason": "wall_shape_uncertain"}
'''
NEW_FIT = '''    if np.percentile(residual, 80) > .06:
        # [격리 직각 코너 시험 전용] 3차식이 표현하지 못하는 꺾인 실제 벽만 꺾은선으로 다시 표현한다.
        shifted = _polyline_offset_path(samples, sign, offset)
        if shifted is None:
            return None, {"reason": "wall_shape_uncertain"}
        path_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(shifted, axis=0), axis=1))]
        path = np.column_stack([np.interp(np.linspace(0., path_s[-1], 36), path_s, shifted[:, axis]) for axis in (0, 1)])
        return path, {"reason": "local_path", "shape_model": "polyline_trial", "boundary_samples": len(wall),
                      "boundary_observed_ahead_m": float(last-anchor_s),
                      "wall_distance_m": float(abs(wall[anchor, 1])), "path_points": path.tolist()}
'''

OLD_BRIDGE = '''                if (not eligible and cfg.boundary_bridge and carried is not None and preferred is not None
'''
NEW_BRIDGE = '''                if not eligible and carried is not None:
                    # [격리 직각 코너 시험 전용] 선호 벽이 직각 모서리에서 끊기면, 꺾은선으로 표현된 긴 반대 벽
                    # 경로가 직전 승인 경로와 일치할 때만 이어서 쓴다. 기억이 없으면 쓰지 않는다.
                    opposite, other = opposite_wall()
                    if (sufficient(opposite, other) and other.get('shape_model') == 'polyline_trial'
                            and path_agrees(opposite, carried, cfg.boundary_agreement_m)):
                        candidate, info, candidate_wall = opposite, other, other_side
                        eligible = True
                        detail['boundary_selection'] = 'polyline_opposite_history_trial'
''' + OLD_BRIDGE

for path, old, new in ((lp, 'def local_path(', HELPER.lstrip('\n').rsplit('def local_path(', 1)[0] + 'def local_path('),
                       (lp, OLD_FIT, NEW_FIT), (lf, OLD_BRIDGE, NEW_BRIDGE)):
    text = path.read_text(encoding='utf-8')
    if text.count(old) != 1:
        raise SystemExit(f'anchor not unique in {path.name}: {old[:40]!r}')
    path.write_bytes(text.replace(old, new).encode('utf-8'))
print('applied')
