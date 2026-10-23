"""모든 각도를 원래 식으로 계산하되 질의 행을 작은 묶음으로 나누는 후보."""
import math

import numpy as np


def depth_support(scene, queries, padding, block_size=128):
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
    for start in range(0, len(q), block_size):
        part = q[start:start+block_size]
        dx = part[:, 0, None]-origins[None, :, 0]
        dy = part[:, 1, None]-origins[None, :, 1]
        angles = np.arctan2(directions[None, :, 0]*dy-directions[None, :, 1]*dx,
                            directions[None, :, 0]*dx+directions[None, :, 1]*dy)
        lower = np.argmin(np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf), axis=1)
        upper = np.argmin(np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf), axis=1)
        k = np.arange(len(part))
        low_radius = np.sqrt(dx[k, lower]**2+dy[k, lower]**2)
        high_radius = np.sqrt(dx[k, upper]**2+dy[k, upper]**2)
        result[start:start+len(part)] = ((angles[k, lower] >= -1e-10) & (angles[k, upper] <= 1e-10)
            & ((abs(angles[k, lower])+abs(angles[k, upper])) <= .025)
            & (abs(indices[lower]-indices[upper]) <= 1) & valid[lower] & valid[upper]
            & (low_radius > 1e-9) & (high_radius > 1e-9)
            & (depth[lower]-low_radius-pad[lower] > 0)
            & (depth[upper]-high_radius-pad[upper] > 0))
    return result
