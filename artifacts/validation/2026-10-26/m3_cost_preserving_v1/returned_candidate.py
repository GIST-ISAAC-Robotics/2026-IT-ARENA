"""두 인접 광선 반환 검사 후보. 기존 depth_support의 보수적 각도 창을 재사용한다.

관측 의미를 바꾸지 않는다. 원점/가까운 질의는 모든 광선을 검사하고,
이전 검사와 같은 atan2·동률 순서·반환 상태·source 인접 규칙을 유지한다.
"""
import math
import numpy as np
from arena_autonomy.scan_topology import cyclic_count, adjacent_indices


def both_returned_window(scene, points, *, cycle=None):
    cycle_count = cyclic_count(scene, cycle)
    q = np.asarray(points, dtype=float).reshape(-1, 2)
    out = np.zeros(len(q), dtype=bool)
    if not len(q): return out
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    valid = np.array([r.status == 'return' for r in scene.rays])
    n = len(origins)
    if not n: raise ValueError('nonempty query requires rays')
    center = origins.mean(axis=0)
    radius = float(np.max(np.linalg.norm(origins-center, axis=1)))+1e-12
    theta = np.mod(np.arctan2(directions[:, 1], directions[:, 0]), math.tau)
    order = np.argsort(theta, kind='stable')
    bearings = np.concatenate((theta[order]-math.tau, theta[order], theta[order]+math.tau))
    ray_order = np.tile(order, 3)

    def evaluate(part, chosen):
        present = chosen < n
        take = np.minimum(chosen, n-1)
        d = part[:, None, :]-origins[take]
        angle = np.arctan2(directions[take, 0]*d[:, :, 1]-directions[take, 1]*d[:, :, 0],
                           directions[take, 0]*d[:, :, 0]+directions[take, 1]*d[:, :, 1])
        low = np.where(present & (angle >= -1e-10), np.maximum(angle, 0.), np.inf)
        high = np.where(present & (angle <= 1e-10), np.maximum(-angle, 0.), np.inf)
        lo, hi = np.argmin(low, axis=1), np.argmin(high, axis=1)
        row = np.arange(len(part)); a, b = take[row, lo], take[row, hi]
        return (np.isfinite(low[row, lo]) & np.isfinite(high[row, hi])
                & (low[row, lo]+high[row, hi] <= .025)
                & adjacent_indices(indices[a], indices[b], cycle_count=cycle_count)
                & valid[a] & valid[b])

    for start in range(0, len(q), 128):
        part = q[start:start+128]
        delta = part-center
        distance = np.linalg.norm(delta, axis=1)
        window = np.isfinite(distance) & (distance > radius+1e-9)
        fallback = np.flatnonzero(~window)
        if len(fallback):
            out[start+fallback] = evaluate(part[fallback], np.broadcast_to(np.arange(n), (len(fallback), n)))
        rows = np.flatnonzero(window)
        if not len(rows): continue
        phi = np.mod(np.arctan2(delta[rows, 1], delta[rows, 0]), math.tau)
        half = np.arcsin(np.minimum(1., radius/distance[rows]))+.025+2e-10
        low = np.searchsorted(bearings, phi-half, side='left')
        high = np.searchsorted(bearings, phi+half, side='right')
        width = high-low
        selected = width > 0
        rows, low, width = rows[selected], low[selected], width[selected]
        if not len(rows): continue
        maximum = int(width.max())
        if maximum >= n*.75:
            chosen = np.broadcast_to(np.arange(n), (len(rows), n))
        else:
            offsets = np.arange(maximum)
            positions = np.minimum(low[:, None]+offsets, len(ray_order)-1)
            chosen = np.sort(np.where(offsets < width[:, None], ray_order[positions], n), axis=1)
        out[start+rows] = evaluate(part[rows], chosen)
    return out
