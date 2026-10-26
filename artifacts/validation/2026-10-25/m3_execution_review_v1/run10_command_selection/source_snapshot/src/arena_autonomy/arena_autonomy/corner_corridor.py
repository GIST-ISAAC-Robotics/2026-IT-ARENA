"""미채택 코너 회랑 기하: 긴 바깥 관측과 의미 미확정 안쪽 실제점의 대조.

시간/운동 확인, 점유, 제동, 독립 보호, 명령 발행은 이 모듈에 없다. supported는
기하 지지뿐이며 벽 분류나 자유 공간 판정이 아니다. 기본 주행에서는 비활성이다.
"""
import math

import numpy as np

from arena_autonomy.local_path import local_path
from arena_autonomy.local_scene import MAX_RAYS


def _components(points, side):
    """실제 각도순 간격과 ROI를 유지하며 빠진 부분을 이어 붙이지 않는다."""
    sign = 1 if side == 'left' else -1
    order = np.argsort(-sign*np.arctan2(points[:, 1], points[:, 0]), kind='stable')
    ordered = points[order]
    limits = .10+.035*np.minimum(np.linalg.norm(ordered[:-1], axis=1), np.linalg.norm(ordered[1:], axis=1))
    breaks = np.flatnonzero(np.linalg.norm(np.diff(ordered, axis=0), axis=1) > limits)
    for component in np.split(order, breaks+1):
        mask = (sign*points[component, 1] > .10) & (np.linalg.norm(points[component], axis=1) <= 2.)
        starts = np.flatnonzero(mask & np.r_[True, ~mask[:-1]])
        ends = np.flatnonzero(mask & np.r_[~mask[1:], True])+1
        for lo, hi in zip(starts, ends):
            if hi-lo >= 12:
                yield component[lo:hi]


def corridor_evidence(points, side, config):
    """현재 실제점만으로 공통 경로 prefix를 반환한다. 점유점은 수정하지 않는다.

    긴 바깥 벽 0.65 m/앵커 계약은 config 그대로다. 짧은 면은 관측 지름이 아닌
    0.65 m 경로에 대한 실제점 거리/법선 지지를 요구한다. 두 개 이상이면 공통
    prefix만 남긴다. 짧은 면의 정체성은 미확정이며 벽 기억으로 갱신하면 안 된다.
    """
    base = dict(supported=False, reason='invalid_input', path=None, witnesses=[], rejected=[],
                inner_identity='unknown', inner_semantic_hypotheses=['boundary_surface','other_occupied_object'],
                wall_identity_certified=False, memory_refresh_authorized=False, free_area_certified=False,
                driving_authorized=False, unknown_gaps_filled=False, occupancy_evaluated=False,
                temporal_confirmation_evaluated=False, runtime_connected=False)
    try:
        p = np.asarray(points, dtype=float)
    except (TypeError, ValueError):
        return base
    if (side not in ('left','right') or p.ndim != 2 or p.shape[1] != 2 or
            not 30 <= len(p) <= MAX_RAYS or not np.all(np.isfinite(p))):
        return base
    other = 'right' if side == 'left' else 'left'
    path, info = local_path(p, other, config.wall_offset_m)
    base.update(outer_side=other, outer_info=info)
    if (path is None or info['boundary_observed_ahead_m'] < config.boundary_min_ahead_m or
            abs(info['wall_distance_m']-config.wall_offset_m) > config.boundary_anchor_error_m):
        return {**base, 'reason':'long_outer_reference_unavailable'}
    tangent = np.gradient(path, axis=0)
    norms = np.linalg.norm(tangent, axis=1)
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    if np.any(norms <= 1e-9) or np.any(steps <= 1e-9):
        return {**base, 'reason':'degenerate_outer_path'}
    angles = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    origin = int(np.argmin(np.linalg.norm(path, axis=1)))
    sign = 1 if side == 'left' else -1
    turn = float(sign*(angles[-1]-angles[origin]))
    opposing = float(np.maximum(0., -sign*np.diff(angles[origin:])).sum())
    if (not math.radians(30) <= turn <= math.radians(135) or opposing > math.radians(5) or
            tangent[origin, 0] <= 0):
        return {**base, 'reason':'not_consistent_inward_corner'}
    unit = tangent/norms[:, None]
    normal = sign*np.column_stack((-unit[:, 1], unit[:, 0]))
    arc = np.r_[0., np.cumsum(steps)]
    witnesses, rejected = [], []
    for indices in _components(p, side):
        observed = p[indices]
        delta = observed[None]-path[origin:, None]
        distances = np.linalg.norm(delta, axis=2)
        chosen = np.argmin(distances, axis=1)
        distance = distances[np.arange(len(chosen)), chosen]
        vector = delta[np.arange(len(chosen)), chosen]
        cosine = np.sum(vector*normal[origin:], axis=1)/np.maximum(distance, 1e-9)
        supported = ((np.abs(distance-config.wall_offset_m) <= config.boundary_agreement_m) &
                     (cosine >= math.cos(math.radians(30))))
        invalid = np.flatnonzero(~supported)
        prefix_count = int(invalid[0]) if len(invalid) else len(chosen)
        end = origin+prefix_count
        prefix = float(arc[end-1]-arc[origin]) if prefix_count else 0.
        retained_turn = float(sign*(angles[end-1]-angles[origin])) if prefix_count else 0.
        # 보고용 범위 하한만 계산한다. 8,192점의 O(n²) 전체 지름 배열을 만들지 않는다.
        extrema = observed[np.r_[np.argmin(observed, axis=0), np.argmax(observed, axis=0)]]
        extent = float(np.max(np.linalg.norm(extrema[:,None]-extrema[None], axis=2)))
        detail = dict(source_point_indices=indices.tolist(), observed_extent_lower_bound_m=extent,
                      prefix_m=prefix, end_exclusive=end, turn_rad=retained_turn)
        if prefix < config.boundary_min_ahead_m:
            rejected.append({**detail, 'reason':'short_geometric_support'})
        elif retained_turn < math.radians(30):
            rejected.append({**detail, 'reason':'supported_prefix_not_corner'})
        else:
            witnesses.append(detail)
    if not witnesses:
        return {**base, 'reason':'inner_surface_support_unavailable', 'rejected':rejected}
    end = min(w['end_exclusive'] for w in witnesses)
    return {**base, 'supported':True, 'reason':'paired_geometry_only', 'path':path[:end].tolist(),
            'witnesses':witnesses, 'rejected':rejected, 'candidate_count':1, 'preferred_side':side,
            'common_prefix_m':min(w['prefix_m'] for w in witnesses)}
