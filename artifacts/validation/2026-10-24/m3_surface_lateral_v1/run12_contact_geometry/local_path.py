"""관측 벽의 지역 평행 경로 + 후륜축 Pure Pursuit. 지도/정답 위치 불사용."""
import copy
import math

import numpy as np


def mask_scan(scan, rear_half_angle_deg=30.):
    """후방 60도는 장착 미확정 사각으로 취급. 빈 공간으로 채우지 않는다."""
    result = copy.deepcopy(scan)
    angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
    ranges = np.asarray(scan.ranges, dtype=float).copy()
    ranges[np.abs(angles) > math.radians(180 - rear_half_angle_deg)] = np.nan
    result.ranges = ranges.tolist()
    return result


def local_path(points, side, offset=.425, horizon=1.8):
    """연결된 관측 벽의 호길이 매개곡선. 수직 벽/급커브도 y=f(x)로 강제하지 않는다."""
    sign = 1 if side == "left" else -1
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 2:
        return None, {"reason": "wall_lost"}
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < 12:
        return None, {"reason": "wall_lost"}
    # 보정이 끝난 현재 좌표의 방위 순서. 왼쪽은 감소, 오른쪽은 증가 방향이 전진이다.
    angles = np.arctan2(points[:, 1], points[:, 0])
    points = points[np.argsort(-sign * angles)]
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    # 먼 점의 각도 간격은 넓지만, 떨어진 벽/차량 사이를 잇는 임의 선분은 만들지 않는다.
    limits = .10 + .035 * np.minimum(np.linalg.norm(points[:-1], axis=1), np.linalg.norm(points[1:], axis=1))
    breaks = np.flatnonzero(gaps > limits)
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
        if last-anchor_s >= .28:
            eligible.append((score, wall, arc, anchor, anchor_s, first, last))
    if not eligible:
        return None, {"reason": "wall_segment_short"}
    # 가까운 구간이 기둥에 끊겼다면 그 앞에서 실제 관측한 긴 벽 구간을 선택한다.
    # 구간 사이를 연결하거나 기둥 점을 보호층에서 제거하지 않는다.
    _, wall, arc, anchor, anchor_s, first, last = min(eligible, key=lambda item: item[0])
    # 같은 호길이 간격으로 재표본화하고 이웃 점으로 접선을 구한다. 관측 끝 밖으로 외삽하지 않는다.
    sample_s = np.linspace(first, last, max(12, int((last-first)/.025)+1))
    samples = np.column_stack([np.interp(sample_s, arc, wall[:, axis]) for axis in (0, 1)])
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
    return path, {"reason": "local_path", "boundary_samples": len(wall),
                  "boundary_observed_ahead_m": float(last-anchor_s),
                  "wall_distance_m": float(abs(wall[anchor, 1])), "path_points": path.tolist()}


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
    for distance in np.arange(0., horizon + .001, .04):
        yaw = curvature * distance
        if abs(curvature) < 1e-5:
            rx, ry = distance - wheelbase / 2, 0.
        else:
            rx, ry = math.sin(yaw) / curvature - wheelbase / 2, (1 - math.cos(yaw)) / curvature
        dx, dy = points[:, 0] - rx, points[:, 1] - ry
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy - wheelbase / 2
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if np.any((np.abs(local_x) <= length/2 + margin) & (np.abs(local_y) <= width/2 + margin)):
            return max(0., float(distance) - .04)  # 이산 검사 간격을 안전 여유로 차감
    return horizon


def braking_speed(clearance, age, deceleration=2.0):
    reaction = max(0., age) + .10
    distance = max(0., clearance - .06)
    return max(0., -deceleration * reaction + math.sqrt((deceleration * reaction) ** 2 + 2 * deceleration * distance))
