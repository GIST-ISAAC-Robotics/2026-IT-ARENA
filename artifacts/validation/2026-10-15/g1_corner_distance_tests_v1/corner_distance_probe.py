"""운영 미연결 급커브 기하 후보. 실제 안쪽 연결 관측과 경로의 거리를 직접 대조한다.

바깥 경로의 긴 관측/앵커·운동 기억·점유/보호 판정은 이 함수의 책임이 아니다.
그 조건을 통과하지 않은 입력을 주행 허가로 승격하지 않는다. 실제 점 사이의
연결 간격을 넘는 선분은 만들지 않으며, 센서점/자유 공간을 추가하지 않는다.
"""
import math

import numpy as np


def corner_distance_evidence(points, reference, side, config):
    p, path = np.asarray(points, dtype=float), np.asarray(reference, dtype=float)
    base = dict(supported=False, reason='invalid_corner_distance_input', source_indices=[],
                is_drive_permission=False, free_area_certified=False, unknown_gaps_filled=False,
                added_sensor_points=0)
    if (side not in ('left', 'right') or p.ndim != 2 or p.shape[1] != 2 or
            path.ndim != 2 or path.shape[1] != 2 or len(p) < 12 or len(path) < 12 or
            not np.all(np.isfinite(p)) or not np.all(np.isfinite(path))):
        return base
    sign = 1 if side == 'left' else -1
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    tangent = np.gradient(path, axis=0)
    norms = np.linalg.norm(tangent, axis=1)
    if np.any(steps < 1e-7) or np.any(norms < 1e-7):
        return {**base, 'reason': 'degenerate_reference'}
    tangent /= norms[:, None]
    arc = np.r_[0., np.cumsum(steps)]
    origin = int(np.argmin(np.linalg.norm(path, axis=1)))
    angles = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    turn = sign*(angles[-1]-angles[origin])
    opposing = float(np.maximum(0., -sign*np.diff(angles[origin:])).sum())
    ahead = float(arc[-1]-arc[origin])
    base.update(turn_rad=float(turn), opposing_turn_rad=opposing, reference_ahead_m=ahead)
    if (not math.radians(30) <= turn <= math.radians(135) or opposing > math.radians(5)
            or tangent[origin, 0] <= 0):
        return {**base, 'reason': 'not_consistent_inward_corner'}
    if ahead < config.boundary_min_ahead_m:
        return {**base, 'reason': 'corner_reference_short'}
    query = path[origin:]
    normal = sign*np.column_stack((-tangent[origin:, 1], tangent[origin:, 0]))
    order = np.argsort(-sign*np.arctan2(p[:, 1], p[:, 0]), kind='stable')
    ordered = p[order]
    gaps = np.linalg.norm(np.diff(ordered, axis=0), axis=1)
    limits = .10+.035*np.minimum(np.linalg.norm(ordered[:-1], axis=1),
                                 np.linalg.norm(ordered[1:], axis=1))
    rejected, accepted = [], []
    for component in np.split(order, np.flatnonzero(gaps > limits)+1):
        # 선호 측 가까운 점만 고르되, 제외한 부분을 건너뛰어 연결하지 않는다.
        mask = (sign*p[component, 1] > .10) & (np.linalg.norm(p[component], axis=1) <= 2.)
        starts = np.flatnonzero(mask & np.r_[True, ~mask[:-1]])
        ends = np.flatnonzero(mask & np.r_[~mask[1:], True])+1
        for lo, hi in zip(starts, ends):
            indices = component[lo:hi]
            if len(indices) < 12:
                continue
            wall = p[indices]
            # 잡음의 지그재그 호길이가 아니라 실제 반환점 간 거리를 하한으로 사용한다.
            extrema = wall[np.r_[np.argmin(wall, axis=0), np.argmax(wall, axis=0)]]
            extent = float(np.max(np.linalg.norm(extrema[:, None]-extrema[None], axis=2)))
            if extent < config.boundary_min_ahead_m:
                rejected.append('observed_component_short')
                continue
            a, d = wall[:-1], np.diff(wall, axis=0)
            norm2 = np.sum(d*d, axis=1)
            valid = norm2 > 1e-12
            if not np.any(valid):
                continue
            a, d, norm2 = a[valid], d[valid], norm2[valid]
            projection = np.clip(np.sum((query[:, None]-a[None])*d[None], axis=2)/norm2, 0., 1.)
            nearest = a[None]+projection[:, :, None]*d[None]
            vectors = nearest-query[:, None]
            distances = np.linalg.norm(vectors, axis=2)
            chosen = np.argmin(distances, axis=1)
            distance = distances[np.arange(len(query)), chosen]
            vector = vectors[np.arange(len(query)), chosen]
            error = float(np.max(np.abs(distance-config.wall_offset_m)))
            normal_cosine = np.sum(vector*normal, axis=1)/np.maximum(distance, 1e-9)
            detail = dict(observed_extent_lower_bound_m=extent, maximum_offset_error_m=error,
                          minimum_normal_cosine=float(np.min(normal_cosine)))
            if error > config.boundary_agreement_m:
                rejected.append('observed_distance_band_mismatch')
                continue
            if np.min(normal_cosine) < math.cos(math.radians(30)):
                rejected.append('observed_normal_direction_mismatch')
                continue
            accepted.append({**detail, 'source_indices': indices.tolist()})
    if not accepted:
        return {**base, 'reason': rejected[0] if rejected else 'no_observed_inner_component',
                'rejected_reasons': sorted(set(rejected))}
    best = min(accepted, key=lambda row: row['maximum_offset_error_m'])
    return {**base, **best, 'supported': True, 'reason': 'observed_corner_distance_support'}
