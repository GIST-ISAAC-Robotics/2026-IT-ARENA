"""깊이 질의의 각도 창 후보. 운영 미연결이며 원시 결측을 건너뛰지 않는다.

모든 광선 원점을 포함하는 반경 R 원을 만든다. 그 원 밖 거리 d의 질의는
어느 원점에서 보아도 중심 방위와 asin(R/d) 이상 차이 나지 않는다. 기존 합각
조건 .025 rad를 통과할 광선은 반드시 그 여유를 더한 창 안에 있다. 원 안의
질의/비유한 질의/큰 창은 전체 광선을 그대로 사용한다. 후보 창은 광선의 상태나
반환 깊이로 줄이지 않으며, 원래 배열 순서로 정렬해 argmin 동률도 유지한다.
"""
import math

import numpy as np


def depth_support(scene, queries, padding):
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    result = np.zeros(len(q), dtype=bool)
    if not len(q):
        return result
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    valid = np.array([r.status == 'return' for r in scene.rays])
    depth = np.array([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return' else -1. for r in scene.rays])
    pad = np.asarray(padding)
    n = len(origins)
    center = origins.mean(axis=0)
    radius = float(np.max(np.linalg.norm(origins-center, axis=1)))+1e-12
    theta = np.mod(np.arctan2(directions[:, 1], directions[:, 0]), math.tau)
    order = np.argsort(theta, kind='stable')
    bearings = np.concatenate((theta[order]-math.tau, theta[order], theta[order]+math.tau))
    ray_order = np.tile(order, 3)

    def evaluate(part, chosen):
        present = chosen < n
        take = np.minimum(chosen, n-1)
        dx = part[:, 0, None]-origins[take, 0]
        dy = part[:, 1, None]-origins[take, 1]
        angles = np.arctan2(directions[take, 0]*dy-directions[take, 1]*dx,
                            directions[take, 0]*dx+directions[take, 1]*dy)
        lower = np.argmin(np.where(present & (angles >= -1e-10), np.maximum(angles, 0.), np.inf), axis=1)
        upper = np.argmin(np.where(present & (angles <= 1e-10), np.maximum(-angles, 0.), np.inf), axis=1)
        k = np.arange(len(part))
        lo, hi = take[k, lower], take[k, upper]
        low_radius = np.sqrt(dx[k, lower]**2+dy[k, lower]**2)
        high_radius = np.sqrt(dx[k, upper]**2+dy[k, upper]**2)
        return (present[k, lower] & present[k, upper] & (angles[k, lower] >= -1e-10)
            & (angles[k, upper] <= 1e-10) & ((abs(angles[k, lower])+abs(angles[k, upper])) <= .025)
            & (abs(indices[lo]-indices[hi]) <= 1) & valid[lo] & valid[hi]
            & (low_radius > 1e-9) & (high_radius > 1e-9)
            & (depth[lo]-low_radius-pad[lo] > 0) & (depth[hi]-high_radius-pad[hi] > 0))

    for start in range(0, len(q), 128):
        part = q[start:start+128]
        delta = part-center
        distance = np.linalg.norm(delta, axis=1)
        window = np.isfinite(distance) & (distance > radius+1e-12)
        fallback = np.flatnonzero(~window)
        if len(fallback):
            result[start+fallback] = evaluate(part[fallback], np.broadcast_to(np.arange(n), (len(fallback), n)))
        rows = np.flatnonzero(window)
        if not len(rows):
            continue
        phi = np.mod(np.arctan2(delta[rows, 1], delta[rows, 0]), math.tau)
        half = np.arcsin(np.minimum(1., radius/distance[rows]))+.025+2e-10
        low = np.searchsorted(bearings, phi-half, side='left')
        high = np.searchsorted(bearings, phi+half, side='right')
        width = high-low
        nonempty = width > 0
        rows, low, width = rows[nonempty], low[nonempty], width[nonempty]
        if not len(rows):
            continue
        maximum = int(width.max())
        if maximum >= n*.75:
            chosen = np.broadcast_to(np.arange(n), (len(rows), n))
        else:
            offsets = np.arange(maximum)
            positions = np.minimum(low[:, None]+offsets, len(ray_order)-1)
            chosen = np.sort(np.where(offsets < width[:, None], ray_order[positions], n), axis=1)
        result[start+rows] = evaluate(part[rows], chosen)
    return result
