"""r245 생성 비교 사본. 공통 패키지/기본값과 분리한다."""
import math
import numpy as np
from arena_autonomy.scan_topology import cyclic_count, adjacent_indices
from native_eval import evaluate_native

def depth_support(scene, queries, padding, *, cycle=None):
    """가능한 양옆 광선을 포함하는 각도 상계. 결측/동률/반환 조건은 불변.

    원점 원 내부는 전체 광선으로 계산한다. 어느 광선 원점과든 기존 반경
    수치 경계 1e-9 m 안에 겹친 질의는 방향이 정의되지 않아 지지하지 않는다.
    """
    cycle_count = cyclic_count(scene, cycle)
    if cycle_count is not None:
        checked_pad = np.asarray(padding, dtype=float)
        if (checked_pad.shape != (len(scene.rays),) or not np.all(np.isfinite(checked_pad))
                or np.any(checked_pad < 0)):
            raise ValueError('finite nonnegative per-ray padding required')
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
        return evaluate_native(part, chosen, origins, directions, indices, valid, depth, pad, cycle_count)

    # 광선/질의는 그대로 두고 작은 NumPy 호출의 반복 비용만 줄인다.
    for start in range(0, len(q), 512):
        part = q[start:start+512]
        delta = part-center
        distance = np.linalg.norm(delta, axis=1)
        window = np.isfinite(distance) & (distance > radius+1e-9)
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
