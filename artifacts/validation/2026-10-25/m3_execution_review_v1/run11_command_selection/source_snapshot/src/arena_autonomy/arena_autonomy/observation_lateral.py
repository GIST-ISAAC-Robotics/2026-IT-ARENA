"""취득 뒤 미관측 영역의 측방 도달 가능성. 후방 사각까지의 안전 보증은 아니다."""
import math

import numpy as np

from arena_autonomy.linear_observation import clip_halfplane


def protected_queries(queries, sensor_pose):
    """기존 전방 ±100° 보호 범위를 현재 센서 pose에서 구분한다. 공간을 채우지 않는다."""
    x, y, yaw = sensor_pose
    if not all(math.isfinite(v) for v in sensor_pose):
        raise ValueError('finite sensor pose required')
    q = np.asarray(queries, dtype=float).reshape(-1, 2)-[x, y]
    angle = np.arctan2(q[:, 1], q[:, 0])-yaw
    angle = np.arctan2(np.sin(angle), np.cos(angle))
    return abs(angle) <= math.radians(100)+1e-12


def polygon_distance(queries, polygon):
    """볼록 다각형까지의 거리. 선분으로 퇴화한 입력도 보수적으로 처리한다."""
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    if not polygon:
        return np.full(len(q), np.inf)
    p = np.asarray(polygon, dtype=float)
    start, end = p, np.roll(p, -1, axis=0)
    step = end-start
    delta = q[:, None]-start[None]
    length2 = np.sum(step*step, axis=1)
    t = np.clip(np.sum(delta*step[None], axis=2)/np.maximum(length2, 1e-30), 0., 1.)
    distance = np.min(np.linalg.norm(delta-t[:, :, None]*step[None], axis=2), axis=1)
    cross = step[None, :, 0]*delta[:, :, 1]-step[None, :, 1]*delta[:, :, 0]
    # 영면적 선분의 '내부'를 전체 평면이라고 해석하지 않는다.
    area = abs(np.sum(start[:, 0]*end[:, 1]-start[:, 1]*end[:, 0]))
    if area > 1e-14:
        inside = np.all(cross >= -1e-12, axis=1) | np.all(cross <= 1e-12, axis=1)
        distance[inside] = 0.
    return distance


def outside_pieces(polygon, length, width):
    for origin, direction, sign in (((length/2, 0.), (0., 1.), -1),
                                     ((-length/2, 0.), (0., 1.), 1),
                                     ((0., width/2), (1., 0.), 1),
                                     ((0., -width/2), (1., 0.), -1)):
        piece = clip_halfplane(polygon, origin, direction, sign)
        if piece:
            yield piece


def reachable_from_unknown(scene, queries, config):
    """실제 취득 시각의 unknown/masked에서 현재 질의까지의 이동 상계를 대조한다.

    현재 명목 몸체 내부는 물체의 시작점에서 제외한다. 물체 운동을 관측한 것이
    아니라 정해진 속도 상계의 가능성 검사다. 현재 몸체 밖 미관측을 free로 바꾸지 않는다.
    """
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    reached = np.zeros(len(q), dtype=bool)
    if not len(q):
        return reached
    if (not np.all(np.isfinite(q)) or scene.source_first_s is None or
            not 0 <= scene.t_ref_s-scene.source_first_s < .3):
        return ~reached
    radius = config.object_motion_bound_mps*(scene.t_ref_s-scene.source_first_s)
    low, high = q.min(axis=0)-radius-.01, q.max(axis=0)+radius+.01
    rectangle = [(low[0], low[1]), (high[0], low[1]), (high[0], high[1]), (low[0], high[1])]
    rectangle = [tuple(map(float, p)) for p in rectangle]
    c, s = math.cos(math.pi/500), math.sin(math.pi/500)
    for ray in scene.rays:
        if ray.status == 'return':
            continue
        dx, dy = ray.direction_xy
        lower, upper = (c*dx+s*dy, -s*dx+c*dy), (c*dx-s*dy, s*dx+c*dy)
        cut = clip_halfplane(clip_halfplane(rectangle, ray.ray_origin_xy_m, lower, 1),
                             ray.ray_origin_xy_m, upper, -1)
        if not cut:
            continue
        budget = config.object_motion_bound_mps*(scene.t_ref_s-ray.acquired_at_s)
        for piece in outside_pieces(cut, config.length_m, config.width_m):
            indices = np.flatnonzero(~reached)
            if not len(indices):
                return reached
            reached[indices] |= polygon_distance(q[indices], piece) <= budget+1e-12
    return reached
