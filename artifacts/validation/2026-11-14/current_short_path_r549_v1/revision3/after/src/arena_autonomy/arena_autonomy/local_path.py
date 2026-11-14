"""관측 벽의 지역 평행 경로 + 후륜축 Pure Pursuit. 지도/정답 위치 불사용."""
import copy
import math

import numpy as np


def mask_scan(scan, rear_half_angle_deg=30.):
    """후방 60도는 장착 미확정 사각으로 취급. 빈 공간으로 채우지 않는다."""
    from arena_autonomy.local_scene import blind_spec,range_bands
    from arena_autonomy.measurement_scan import MeasurementScan
    from arena_autonomy.scan_input import scan_angles
    from dataclasses import replace
    rear, sectors = blind_spec(rear_half_angle_deg)
    bands=range_bands(rear_half_angle_deg,len(sectors))
    result = copy.deepcopy(scan)
    angles = scan_angles(scan)
    if isinstance(scan,MeasurementScan):
        angles=(angles+math.pi)%math.tau-math.pi
    ranges = np.asarray(scan.ranges, dtype=float).copy()
    ranges[np.abs(angles) > math.radians(180 - rear)] = np.nan
    # 기존 각도 전용은 unknown, CAD 거리 범위가 있으면 자기 반환에만 한정한다.
    for index,(center, width) in enumerate(sectors):
        offset = np.remainder(angles - math.radians(center) + math.pi, math.tau) - math.pi
        selected=np.abs(offset) <= math.radians(width) + 1e-12
        if bands is not None:
            lo,hi=bands[index]
            selected &= ~np.isfinite(ranges) | ((ranges>=lo)&(ranges<=hi))
        ranges[selected] = np.nan
    if isinstance(scan,MeasurementScan):
        return replace(scan,ranges_m=tuple(ranges.tolist()))
    result.ranges = ranges.tolist()
    return result


def ray_continuous_gaps(points, gaps, limits, rays, horizon, tolerance=.03, near=6, span=.30, heading=20.):
    """전방으로 멀어지는 옆 벽에서 광선 간격 때문에 생긴 단절만 고른다.

    같은 회전에서 원시 광선 번호가 이어진 가까운 여섯 점의 직선에 다음 두 실제 점이
    수직 3 cm 안으로 놓일 때만 잇는다. 벽 방향이 차체 전방에서 벗어나거나 결측/다른 면이
    끼면 잇지 않는다. 점을 만들거나 옮기지 않으며 실제 벽의 연속을 인증하지 않는다.
    """
    joined = np.zeros(len(gaps), dtype=bool)
    ranges = np.linalg.norm(points, axis=1)
    for index in np.flatnonzero(gaps > limits):
        first, stop = index+1-near, index+3
        if first < 0 or stop > len(points) or ranges[index+1] > horizon:
            continue
        steps = np.diff(rays[first:stop])
        if abs(steps[0]) != 1 or np.any(steps != steps[0]):
            continue
        base = points[first:index+1]
        if np.linalg.norm(base[-1]-base[0]) < span:
            continue
        center = base.mean(axis=0)
        tangent = np.linalg.svd(base-center, full_matrices=False)[2][0]
        if tangent @ (base[-1]-base[0]) < 0:
            tangent = -tangent
        if tangent[0] <= 0 or abs(tangent[1]) > math.sin(math.radians(heading)):
            continue
        offsets = points[first:stop]-center
        joined[index] = (np.max(np.abs(offsets @ [-tangent[1], tangent[0]])) <= tolerance
                         and np.all(np.diff(offsets @ tangent) > 0))
    return joined


def _path_ahead(path):
    """원점 최근접 경로점부터 끝까지의 경로 호길이."""
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    return float(arc[-1]-arc[int(np.argmin(np.linalg.norm(path, axis=1)))])


def _covers(path, old, tolerance):
    """old의 모든 점이 path 선분에서 tolerance 안에 있는가."""
    a, d = path[:-1], np.diff(path, axis=0)
    norm2 = np.maximum(1e-12, np.sum(d*d, axis=1))
    t = np.clip(np.sum((old[:, None]-a[None])*d[None], axis=2)/norm2, 0., 1.)
    distance = np.linalg.norm(a[None]+t[:, :, None]*d[None]-old[:, None], axis=2)
    return bool(np.max(np.min(distance, axis=1)) <= tolerance)


def _origin_error(path):
    """차량 원점(0,0)에서 경로 선분까지의 최소 거리. 경로 점만 쓰며 외삽하지 않는다."""
    a, d = path[:-1], np.diff(path, axis=0)
    t = np.clip(-np.sum(a*d, axis=1)/np.maximum(1e-12, np.sum(d*d, axis=1)), 0., 1.)
    return float(np.min(np.linalg.norm(a+t[:, None]*d, axis=1)))


def narrow_occluders(points, broken, max_points=6, max_extent=.04, min_depth=.03):
    """방위 순서 벽 사슬에서 벽 바로 앞 좁은 가림체(신호 지주 등) 묶음을 찾는다.

    양옆 연결이 모두 끊긴 max_points 이하·지름 max_extent 이하 묶음이, 앞뒤 실제 벽점을 잇는
    선분보다 차량 쪽으로 min_depth 이상 나와 있고, 앞뒤 벽점 간격이 그 그림자로 설명될 만큼
    짧을 때만 해당한다. 반환은 (묶음 점 마스크, 앞뒤 벽점 인덱스 쌍). 점을 만들거나 옮기지 않는다.
    """
    breaks = np.flatnonzero(broken)
    mask, joins = np.zeros(len(points), dtype=bool), []
    for first, second in zip(breaks[:-1], breaks[1:]):
        cluster = points[first+1:second+1]
        if not 1 <= len(cluster) <= max_points or second+1 >= len(points):
            continue
        before, after = points[first], points[second+1]
        if 2*np.max(np.linalg.norm(cluster-cluster.mean(axis=0), axis=1)) > max_extent:
            continue
        chord = after-before
        length = float(np.linalg.norm(chord))
        if length < 1e-9 or length > 2*(.10+.035*min(np.linalg.norm(before), np.linalg.norm(after))):
            continue
        normal = np.array([-chord[1], chord[0]])/length
        if np.dot(-before, normal) < 0:
            normal = -normal  # 차량(원점) 쪽이 양수
        if np.min((cluster-before) @ normal) < min_depth:
            continue
        mask[first+1:second+1] = True
        joins.append((int(first), int(second+1)))
    return mask, joins


def _adjacent_cluster_candidates(idx, ranges, current_breaks, max_points, min_depth):
    """원래 edge→count→오른쪽/왼쪽 순서에서 불가능한 묶음만 먼저 제외한다.

    인접 원본 번호와 같은 방사 거리 차이만 묶어서 계산한다. 현 길이·폭·방사
    길이의 최종 검사는 호출자에 그대로 남긴다. 큰 임의 입력은 원래 열거로 돌아간다.
    """
    edges = np.flatnonzero(current_breaks)
    if not len(edges):
        return ()
    limit = min(max_points, len(idx)-2)
    counts = range(1, limit+1)
    candidates = ((lo, hi) for edge in edges for count in counts
                  for lo, hi in ((edge+1, edge+1+count), (edge-count+1, edge+1))
                  if lo >= 1 and hi < len(idx))
    if limit <= 0 or limit > 16 or len(idx)*limit > 65536:
        return candidates
    pairs = np.asarray(list(candidates), dtype=int).reshape(-1, 2)
    if not len(pairs):
        return ()
    lo, hi = pairs.T
    breaks = np.r_[0, np.cumsum(np.diff(idx) != 1)]
    keep = breaks[hi] == breaks[lo-1]
    cluster_max = np.empty(len(pairs))
    maxima = ranges
    widths = hi-lo
    for count in counts:
        if count > 1:
            maxima = np.maximum(maxima[:-1], ranges[count-1:])
        rows = widths == count
        cluster_max[rows] = maxima[lo[rows]]
    # <=의 부정을 사용해 원 함수의 비교 의미(비유한 연산 포함)를 보존한다.
    keep &= ~(np.minimum(ranges[lo-1], ranges[hi])-cluster_max <= min_depth)
    return pairs[keep]


def break_adjacent_occluders(points, broken, max_points=6, max_extent=.04,
                            min_depth=.06, max_radial_extent=.10):
    """단절 옆의 좁은 근거리 묶음을 선택형 벽 기준선에서만 분리한다.

    짧고 잡음이 큰 두 벽점 현의 법선 대신 원점으로부터의 거리 차이를 쓴다.
    한쪽 단절만 있어도 비교하지만 양쪽 실제 반환·기존 그림자 길이 한도는 필요하다.
    호출자가 이미 연결한 경계는 다시 끊지 않는다. 반환은 원본 인덱스이며 원시
    점유·보호 입력은 수정하지 않는다. 미관측 공간의 자유 판정이나 물체 식별이 아니다.
    """
    points = np.asarray(points, dtype=float)
    broken = np.asarray(broken, dtype=bool)
    if points.ndim != 2 or points.shape[1] != 2 or len(broken) != max(0, len(points)-1):
        raise ValueError('invalid wall chain shape')
    if not np.all(np.isfinite(points)):
        raise ValueError('nonfinite wall chain')
    alive = np.ones(len(points), dtype=bool)
    # 반복의 상한은 복잡한 장면에서 이 국소 분리가 전체 계산을 지배하지 않게 한다.
    for _ in range(12):
        idx = np.flatnonzero(alive)
        if len(idx) < 3:
            break
        p = points[idx]
        adjacent = np.diff(idx) == 1
        current_breaks = adjacent & broken[idx[:-1]]
        ranges = np.linalg.norm(p, axis=1)
        found = None
        for lo, hi in _adjacent_cluster_candidates(idx, ranges, current_breaks, max_points, min_depth):
            # 다른 제거 구간을 포함해 작은 물체 여러 개를 하나로 합치지 않는다.
            if not np.all(np.diff(idx[lo-1:hi+1]) == 1):
                continue
            before, after, cluster = p[lo-1], p[hi], p[lo:hi]
            if np.linalg.norm(after-before) > 2*(.10+.035*min(ranges[lo-1], ranges[hi])):
                continue
            if min(ranges[lo-1], ranges[hi])-np.max(ranges[lo:hi]) <= min_depth:
                continue
            center = cluster.mean(axis=0)
            direction = center/max(1e-12, np.linalg.norm(center))
            normal = np.array([-direction[1], direction[0]])
            if (np.ptp(cluster @ normal) > max_extent
                    or np.ptp(cluster @ direction) > max_radial_extent):
                continue
            found = (lo, hi)
            break
        if found is None:
            break
        lo, hi = found
        alive[idx[lo:hi]] = False
    idx = np.flatnonzero(alive)
    joins = [(int(a), int(b)) for a, b in zip(idx[:-1], idx[1:]) if b-a > 1]
    return ~alive, joins


def _sample_count(first, last, spacing=.025):
    """수치 반올림만으로 같은 호길이의 재표본 점 개수가 달라지지 않게 한다.

    끝점 차감·나눗셈의 기계 정밀도 범위에서만 정수 격자에 맞춘다.
    물리 간격·관측 끝·판정 허용량을 늘리지 않는다.
    """
    ratio = (last-first)/spacing
    nearest = round(ratio)
    roundoff = 16*math.ulp(1.)*max(1., abs(first/spacing), abs(last/spacing))
    if abs(ratio-nearest) <= roundoff:
        ratio = float(nearest)
    return max(12, math.floor(ratio)+1)


def local_path(points, side, offset=.425, horizon=1.8, rays=None, piecewise_corner=False, minimum_radius=0.,
               narrow_occluder_bridge=False, minimum_ahead_m=.28, bearings_rad=None):
    """연결된 관측 벽의 호길이 매개곡선. 수직 벽/급커브도 y=f(x)로 강제하지 않는다.

    rays는 points와 같은 순서의 원시 광선 번호다. 주지 않으면 거리 간격만으로 연결한다.
    bearings_rad는 같은 실제 반환 광선의 보정 후 방향이다. 주어지면 이 각도로 정렬한다.
    센서 원점과 차체 원점의 차이로 가까운 벽과 먼 배경이 교차 정렬되는 것을 막는다.
    narrow_occluder_bridge(선택형, 기본 꺼짐)는 벽 앞 좁은 가림체 묶음을 벽 사슬에서만 빼고
    그 앞뒤 실제 벽점을 잇는다. 가림체 점은 호출자의 점유·보호 검사에 그대로 남는다.
    minimum_ahead_m은 벽 앵커 앞 최소 관측 호길이다. 기본0.28m는 유지하며,
    저속 재획득 호출만 차체 길이 기준을 전달한다. 경로의 전방 길이 검사는 별도다.
    """
    sign = 1 if side == "left" else -1
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        return None, {"reason": "wall_lost"}
    bearings = None if bearings_rad is None else np.asarray(bearings_rad, dtype=float)
    if bearings is not None and (bearings.shape != (len(points),) or not np.all(np.isfinite(bearings))):
        return None, {"reason": "wall_bearings_invalid"}
    finite = np.all(np.isfinite(points), axis=1)
    points = points[finite]
    if len(points) < 12:
        return None, {"reason": "wall_lost"}
    # 왼쪽은 감소, 오른쪽은 증가 방향이 전진이다. 광선이 없으면 기존 원점 방위를 유지한다.
    angles = np.arctan2(points[:, 1], points[:, 0]) if bearings is None else bearings[finite]
    order = np.argsort(-sign * angles)
    points = points[order]
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    # 먼 점의 각도 간격은 넓지만, 떨어진 벽/차량 사이를 잇는 임의 선분은 만들지 않는다.
    limits = .10 + .035 * np.minimum(np.linalg.norm(points[:-1], axis=1), np.linalg.norm(points[1:], axis=1))
    broken = gaps > limits
    if rays is not None:
        broken &= ~ray_continuous_gaps(points, gaps, limits, np.asarray(rays)[finite][order], horizon)
    bridged = 0
    if narrow_occluder_bridge:
        occluder, joins = break_adjacent_occluders(points, broken)
        if joins:
            keep = ~occluder
            new_index = np.cumsum(keep)-1
            points = points[keep]
            gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
            limits = .10 + .035 * np.minimum(np.linalg.norm(points[:-1], axis=1), np.linalg.norm(points[1:], axis=1))
            broken = gaps > limits
            if rays is not None:
                kept_rays = np.asarray(rays)[finite][order][keep]
                broken &= ~ray_continuous_gaps(points, gaps, limits, kept_rays, horizon)
            for before, _ in joins:
                broken[new_index[before]] = False
            bridged = len(joins)
    breaks = np.flatnonzero(broken)
    eligible = []
    for wall in np.split(points, breaks + 1):
        if len(wall) < 12:
            continue
        candidates = np.flatnonzero((np.abs(wall[:, 0]) < .35) & (sign * wall[:, 1] > .10))
        if not len(candidates):
            continue
        distances = np.linalg.norm(wall[candidates] - [0., sign * offset], axis=1)
        anchor = int(candidates[np.argmin(distances)])
        score = float(np.min(distances))
        if score > .35:
            continue
        # 거리 잡음의 지그재그를 모두 더하면 직선의 호길이가 몇 배 부풀고
        # 1.8 m 관측 예산을 실제 0.5 m 정도에서 소진한다. 연결 구간 안에서만
        # 호길이 측정용 좌표를 평활화한다. 실제 적합/잔차와 보호층은 원시 점을 사용한다.
        # 깨끗한 벽의 실제 모서리까지 바꾸지 않는다. 각 중간점과 양옆 점의
        # 현 사이 수직 잔차의 중앙값으로 구간 전체에 퍼진 거칠기만 검출한다.
        # 소수의 모서리/끝면은 이 중앙값을 올리지 않는다.
        chord = wall[2:] - wall[:-2]
        middle = wall[1:-1] - wall[:-2]
        roughness = float(np.median(np.abs(chord[:, 0]*middle[:, 1] - chord[:, 1]*middle[:, 0]) /
                                    np.maximum(1e-9, np.linalg.norm(chord, axis=1))))
        metric = wall
        if roughness > .003:
            padded = np.pad(wall, ((3, 3), (0, 0)), mode='edge')
            metric = np.column_stack([np.convolve(padded[:, axis], np.ones(7)/7, mode='valid')
                                      for axis in (0, 1)])
        steps = np.maximum(1e-6, np.linalg.norm(np.diff(metric, axis=0), axis=1))
        arc = np.r_[0., np.cumsum(steps)]
        anchor_s = arc[anchor]
        first, last = max(0., anchor_s-.25), min(arc[-1], anchor_s+horizon)
        if last-anchor_s >= minimum_ahead_m:
            eligible.append((score, wall, arc, anchor, anchor_s, first, last))
    if not eligible:
        return None, {"reason": "wall_segment_short"}
    # 가까운 구간이 기둥에 끊겼다면 그 앞에서 실제 관측한 긴 벽 구간을 선택한다.
    # 구간 사이를 연결하거나 기둥 점을 보호층에서 제거하지 않는다.
    _, wall, arc, anchor, anchor_s, first, last = min(eligible, key=lambda item: item[0])
    # 같은 호길이 간격으로 재표본화하고 이웃 점으로 접선을 구한다. 관측 끝 밖으로 외삽하지 않는다.
    sample_s = np.linspace(first, last, _sample_count(first, last))
    samples = np.column_stack([np.interp(sample_s, arc, wall[:, axis]) for axis in (0, 1)])
    if piecewise_corner:
        from arena_autonomy.piecewise_wall import corner_path
        path = corner_path(samples, side, offset, minimum_radius)
        if path is not None:
            budget = None
            # 벽 탐색 길이는 벽 호길이다. 바깥 모서리의 오프셋 경로는 평행 이동·원호 때문에
            # 그보다 짧다. 같은 연결 구간에 실제 관측이 남을 때만 한 번 더 읽는다.
            # 관측 끝 밖 외삽·다른 구간 연결·경계 길이 기준 변경은 하지 않는다.
            ahead = _path_ahead(path)
            if ahead < horizon and arc[-1]-last > 1e-9:
                extended = min(arc[-1], last+max(0., horizon-ahead)+.05)  # .05는 재표본화 여유
                wider_s = np.linspace(first, extended, _sample_count(first, extended))
                wider = corner_path(np.column_stack([np.interp(wider_s, arc, wall[:, axis]) for axis in (0, 1)]),
                                    side, offset, minimum_radius)
                # 두 번째 모서리·다른 면이 들어오면 corner_path가 거부한다. 기존 경로를 벽 적합
                # 허용량 안에서 그대로 포함하고 더 길어질 때만 바꾼다.
                if wider is not None and _path_ahead(wider) >= ahead and _covers(wider, path, .03):
                    budget = {'wall_ahead_before_m': float(last-anchor_s), 'path_ahead_before_m': ahead,
                              'path_ahead_m': _path_ahead(wider), 'extended': True}
                    path, last = wider, extended
            info = {'reason': 'local_path', 'wall_model': 'piecewise_corner',
                    'boundary_samples': len(wall),
                    'boundary_observed_ahead_m': float(last-anchor_s),
                    'wall_distance_m': float(abs(wall[anchor, 1])), 'path_points': path.tolist(),
                    # 원호 위에서는 벽 y좌표가 오프셋과 달라도 경로 위에 있을 수 있다. 앵커 판정용.
                    'path_anchor_error_m': _origin_error(path)}
            if budget is not None:
                info['corner_path_budget'] = budget
            if bridged:
                info['narrow_occluders_bridged'] = bridged
            return path, info
    # x(s), y(s)를 함께 적합한다. y=f(x)와 달리 수직 벽을 표현하며,
    # 상자 벽의 작은 끝면/돌출부를 매 점의 법선으로 증폭하지 않는다.
    u = (sample_s-first) / max(1e-6, last-first)
    basis = np.column_stack((np.ones_like(u), u, u*u, u*u*u))
    weights = np.ones(len(samples))
    for _ in range(5):
        coefficients = np.linalg.lstsq(basis*weights[:, None], samples*weights[:, None], rcond=None)[0]
        residual = np.linalg.norm(basis@coefficients-samples, axis=1)
        weights = np.sqrt(np.minimum(1., .025/np.maximum(residual, 1e-6)))
    if np.percentile(residual, 80) > .06:
        return None, {"reason": "wall_shape_uncertain"}
    smoothed = basis @ coefficients
    tangent = np.column_stack((u*0, u*0+1, 2*u, 3*u*u)) @ coefficients
    norm = np.linalg.norm(tangent, axis=1)
    if np.any(norm < 1e-7):
        return None, {"reason": "wall_tangent_uncertain"}
    tangent /= norm[:, None]
    normal = sign * np.column_stack((tangent[:, 1], -tangent[:, 0]))
    shifted = smoothed + offset * normal
    # 판/벽 끝면의 작은 반경에 평행 곡선을 만들면 오프셋 곡선이 접힐 수 있다.
    # 관측 벽과 반대 방향으로 진행하는 접힌 가지를 경로에 포함하지 않는다.
    direction = np.sum(np.diff(shifted, axis=0) * (tangent[1:]+tangent[:-1]), axis=1)
    folds = np.flatnonzero(direction <= 0.)
    anchor_index = int(np.argmin(np.abs(sample_s-anchor_s)))
    before = folds[folds < anchor_index]
    after = folds[folds >= anchor_index]
    lo = int(before[-1]+1) if len(before) else 0
    hi = int(after[0]+1) if len(after) else len(shifted)
    shifted = shifted[lo:hi]
    if len(shifted) < 3:
        return None, {'reason': 'offset_path_folded'}
    # 원래 관측 벽과 그 법선만 사용한다. 36점은 경로 표현 해상도이며 새 센서점이 아니다.
    path_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(shifted, axis=0), axis=1))]
    path = np.column_stack([np.interp(np.linspace(0., path_s[-1], 36), path_s, shifted[:, axis]) for axis in (0, 1)])
    info = {"reason": "local_path", "boundary_samples": len(wall),
            "boundary_observed_ahead_m": float(last-anchor_s),
            "wall_distance_m": float(abs(wall[anchor, 1])), "path_points": path.tolist()}
    if piecewise_corner:
        # 같은 코너 경로가 관측에 따라 이 곡선 표현으로 바뀌어도 앵커 판정 지표를 유지한다. 진단 전용.
        info["path_anchor_error_m"] = _origin_error(path)
    if bridged:
        info["narrow_occluders_bridged"] = bridged
    return path, info


def pursuit_command(points, side, speed, wheelbase=.145, max_steering=.37,
                    max_speed=2.0, lateral_acceleration=3.5, offset=.425, scan_age=.2):
    failure = None
    for wall in (side, 'right' if side == 'left' else 'left'):
        target_speed, steering, details = _pursuit_for_wall(
            points, wall, speed, wheelbase, max_steering, max_speed, lateral_acceleration, offset, scan_age)
        details.update(preferred_wall=side, path_wall=wall, wall_fallback=wall != side)
        if target_speed > 0:
            if wall != side:
                details['preferred_wall_failure'] = failure['reason']
                target_speed = min(target_speed, 1.2)
            return target_speed, steering, details
        if failure is None:
            failure = details
        else:
            failure['alternate_wall_failure'] = details['reason']
    return 0., 0., failure


def _pursuit_for_wall(points, side, speed, wheelbase, max_steering, max_speed,
                      lateral_acceleration, offset, scan_age):
    path, details = local_path(points, side, offset)
    if path is None:
        return 0., 0., details
    # 모델 원점은 양 축 중간. Pure Pursuit 원점은 후륜축이다.
    rear = path.copy()
    rear[:, 0] += wheelbase / 2
    distances = np.linalg.norm(rear, axis=1)
    nearest = int(np.argmin(distances))
    ahead = np.flatnonzero((rear[:, 0] > .10) & (np.arange(len(rear)) >= nearest))
    if len(ahead) < 3 or np.max(distances[ahead]) < .25:
        return 0., 0., {**details, "reason": "path_too_short"}
    lookahead = min(.60, max(.30, .30 + .12 * abs(speed)))
    index = int(ahead[np.argmin(np.abs(distances[ahead] - lookahead))])
    target = rear[index]
    curvature = float(2 * target[1] / max(.01, target @ target))
    steering = float(np.clip(math.atan(wheelbase * curvature), -max_steering, max_steering))
    # 관측 끝 뒤를 계속 직선이라고 가정하지 않는다. 새 관측을 못 받더라도
    # 현재 유효한 경로 안에서 감속할 수 있도록 미리 속도를 제한한다.
    path_s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(rear, axis=0), axis=1))]
    remaining = float(path_s[-1]-path_s[nearest])
    observed_cap = braking_speed(remaining, scan_age)
    # 전방의 경로 곡률도 미리 본다. 당장 조향이 0이어도 급커브 전에는 감속한다.
    dx = np.gradient(rear, axis=0)
    ddx = np.gradient(dx, axis=0)
    path_curvature = np.abs(dx[:, 0]*ddx[:, 1]-dx[:, 1]*ddx[:, 0]) / np.maximum(1e-8, np.linalg.norm(dx, axis=1)**3)
    preview_caps = np.sqrt(lateral_acceleration/np.maximum(.15, path_curvature[nearest:]) +
                           4.*np.maximum(0., path_s[nearest:]-path_s[nearest]-.2))
    target_speed = min(max_speed, observed_cap, float(np.min(preview_caps)),
                       math.sqrt(lateral_acceleration / max(.15, abs(curvature))))
    return target_speed, steering, {**details, "lookahead_m": float(distances[index]),
                                   "observed_path_ahead_m": remaining, "observed_path_speed_cap_mps": observed_cap,
                                   "target_xy_rear_m": target.tolist(), "curvature_m_inv": curvature}


def swept_limit(points, steering, wheelbase=.145, horizon=3., margin=.025, length=.20, width=.15):
    """정곡률 예측 차체와 점군의 첫 간섭까지 후륜축 이동 거리를 계산한다."""
    points = np.asarray(points, dtype=float)
    curvature = math.tan(steering) / wheelbase
    distances = np.arange(0., horizon + .001, .04)
    # 같은 4 cm 표본을 여덟 개씩 묶는다. libm 삼각함수와 연산 순서를 유지하여
    # 경계의 포함 판정·첫 간섭·4 cm 차감이 기존 순차 계산과 같도록 한다.
    for start in range(0, len(distances), 8):
        steps = distances[start:start+8]
        yaw = curvature * steps
        c = np.asarray([math.cos(value) for value in yaw])[:, None]
        s = np.asarray([math.sin(value) for value in yaw])[:, None]
        if abs(curvature) < 1e-5:
            rx, ry = steps[:, None] - wheelbase / 2, 0.
        else:
            rx, ry = s / curvature - wheelbase / 2, (1 - c) / curvature
        dx, dy = points[:, 0] - rx, points[:, 1] - ry
        local_x = c * dx + s * dy - wheelbase / 2
        local_y = -s * dx + c * dy
        hit = np.any((np.abs(local_x) <= length/2 + margin) & (np.abs(local_y) <= width/2 + margin), axis=1)
        indices = np.flatnonzero(hit)
        if len(indices):
            return max(0., float(steps[indices[0]]) - .04)  # 이산 검사 간격을 안전 여유로 차감
    return horizon


def braking_speed(clearance, age, deceleration=2.0):
    reaction = max(0., age) + .10
    distance = max(0., clearance - .06)
    return max(0., -deceleration * reaction + math.sqrt((deceleration * reaction) ** 2 + 2 * deceleration * distance))
