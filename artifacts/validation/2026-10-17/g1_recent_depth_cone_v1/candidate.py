"""깊이 검사 후보: 최종 .025 rad 합 조건에 들어갈 수 없는 방향의 atan2만 생략."""
import math

import numpy as np


def depth_support(scene, queries, padding):
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    if len(q) == 0:
        return np.zeros(0, dtype=bool)
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    dx = q[:, 0, None]-origins[None, :, 0]
    dy = q[:, 1, None]-origins[None, :, 1]
    cross = directions[None, :, 0]*dy-directions[None, :, 1]*dx
    dot = directions[None, :, 0]*dx+directions[None, :, 1]*dy
    # arctan(.03) > .025. 최종 양쪽 각도 합이 .025 이하려면 각도 각각도
    # 이 넉넉한 원뿔 안이다. 제외 항목은 NaN으로 남겨 후보 선택에서 제외한다.
    # 실제 광선 원점과 방향·최종 각도/깊이·인접 index 조건은 유지한다.
    angles = np.full_like(cross, np.nan)
    np.arctan2(cross, dot, out=angles, where=(dot > 0) & (abs(cross) <= .03*dot))
    lower = np.argmin(np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf), axis=1)
    upper = np.argmin(np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf), axis=1)
    k = np.arange(len(q))
    low_radius = np.sqrt(dx[k, lower]**2+dy[k, lower]**2)
    high_radius = np.sqrt(dx[k, upper]**2+dy[k, upper]**2)
    indices = np.array([r.source_index for r in scene.rays])
    valid = np.array([r.status == 'return' for r in scene.rays])
    depth = np.array([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return' else -1. for r in scene.rays])
    pad = np.asarray(padding)
    return ((angles[k, lower] >= -1e-10) & (angles[k, upper] <= 1e-10)
        & ((abs(angles[k, lower])+abs(angles[k, upper])) <= .025)
        & (abs(indices[lower]-indices[upper]) <= 1) & valid[lower] & valid[upper]
        & (low_radius > 1e-9) & (high_radius > 1e-9)
        & (depth[lower]-low_radius-pad[lower] > 0)
        & (depth[upper]-high_radius-pad[upper] > 0))
