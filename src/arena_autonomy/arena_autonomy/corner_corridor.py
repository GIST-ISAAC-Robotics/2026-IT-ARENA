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


def projected_origin_offset(path, nearest):
    """최근접 꼭짓점 arc와 인접 선분의 실제 투영 원점 arc 차이.

    표본이 원점 뒤에 있으면 음수, 앞이면 양수다. 다른 경로 가지로
    원점을 옮기거나 관측 경로를 늘리지 않는다.
    """
    lengths = np.linalg.norm(np.diff(path, axis=0), axis=1)
    arc = np.r_[0., np.cumsum(lengths)]
    choices = []
    for index in (nearest-1, nearest):
        if not 0 <= index < len(path)-1 or lengths[index] <= 1e-12:
            continue
        direction = path[index+1]-path[index]
        fraction = float(np.clip(-np.dot(path[index], direction)/lengths[index]**2, 0., 1.))
        point = path[index]+fraction*direction
        choices.append((float(np.dot(point, point)), index, fraction))
    if not choices:
        raise ValueError('no valid origin segment')
    _, index, fraction = min(choices)
    return float(arc[nearest]-(arc[index]+fraction*lengths[index]))


def short_inner_prefix_evidence(geometry, config):
    """긴 바깥 경로 중 안쪽 실제점이 함께 지지한 첫 구간까지만 남긴다.

    기존 회랑의 거리/법선·연결·코너 검사를 재사용한다. 0.65 m 길이만
    충족하지 못한 경우를 선택형 저속/제동 예산 검사에 넘기며 연장하지 않는다.
    """
    rejected = {**geometry, 'supported':False, 'path':None}
    if geometry.get('reason') != 'inner_surface_support_unavailable':
        return rejected
    try:
        info = geometry['outer_info']
        path = np.asarray(info['path_points'], dtype=float)
        if (path.ndim != 2 or path.shape[1] != 2 or len(path) < 6 or
                not np.all(np.isfinite(path)) or
                not math.isfinite(info['boundary_observed_ahead_m']) or
                not math.isfinite(info['wall_distance_m']) or
                info['boundary_observed_ahead_m'] < config.boundary_min_ahead_m or
                abs(info['wall_distance_m']-config.wall_offset_m) > config.boundary_anchor_error_m):
            return rejected
        origin = int(np.argmin(np.linalg.norm(path, axis=1)))
        if np.linalg.norm(path[origin]) > config.boundary_anchor_error_m:
            return rejected
        arc = np.r_[0.,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]
        correction = projected_origin_offset(path,origin)
        witnesses = []
        for item in geometry['rejected']:
            end = item['end_exclusive']
            if (item['reason'] != 'short_geometric_support' or type(end) is not int or
                    not origin+5 <= end <= len(path) or not math.isfinite(item['turn_rad']) or
                    item['turn_rad'] < math.radians(30)):
                continue
            span = float(arc[end-1]-arc[origin])
            if (not math.isclose(span,item['prefix_m'],abs_tol=1e-9) or
                    not .25 <= span+correction < config.boundary_min_ahead_m):
                continue
            witnesses.append(item)
        if not witnesses:
            return rejected
        end = min(w['end_exclusive'] for w in witnesses)
        ahead = float(arc[end-1]-arc[origin]+correction)
    except (KeyError, TypeError, ValueError, IndexError):
        return rejected
    return {**geometry, 'supported':True, 'reason':'short_inner_supported_prefix_geometry_only',
            'path':path[:end].tolist(), 'witnesses':witnesses, 'short_prefix':True,
            'short_prefix_kind':'paired_inner_observed', 'base_observed_ahead_m':ahead,
            'origin_arc_correction_m':correction, 'common_prefix_m':ahead,
            'opposite_path_used_for_steering':True, 'unknown_filled':False}


def short_prefix_evidence(points, side, config):
    """반대 경로와 일치하는 첫 구간 끝에서 선호 경로를 자르는 기하 후보.

    반대 경로는 대조에만 쓰며 그 먼 끝으로 연장하지 않는다. 편측 지지만
    있는 시작 부분은 차체 길이 이내, 공동 지지는 기존 비교와 같은 0.25 m다.
    이는 시험 가정이며 벽 정체·자유 공간·차량 주행 허가가 아니다.
    """
    base = dict(supported=False, reason='invalid_input', path=None,
                runtime_connected=False, driving_authorized=False,
                memory_refresh_authorized=False, wall_identity_certified=False)
    try:
        p = np.asarray(points, dtype=float)
    except (TypeError, ValueError):
        return base
    if side not in ('left', 'right') or p.ndim != 2 or p.shape[1] != 2 or not np.all(np.isfinite(p)):
        return base
    own, info = local_path(p, side, config.wall_offset_m)
    other, other_info = local_path(p, 'left' if side == 'right' else 'right', config.wall_offset_m)
    base.update(preferred_info=info, opposite_info=other_info)
    if own is None or other is None:
        return {**base, 'reason':'missing_path'}
    if info['boundary_observed_ahead_m'] >= config.boundary_min_ahead_m:
        return {**base, 'reason':'not_a_short_boundary'}
    if abs(info['wall_distance_m']-config.wall_offset_m) > config.boundary_anchor_error_m:
        return {**base, 'reason':'preferred_anchor_unconfirmed'}
    if other_info['boundary_observed_ahead_m'] < config.boundary_min_ahead_m:
        return {**base, 'reason':'opposite_reference_short'}
    nearest = int(np.argmin(np.linalg.norm(own, axis=1)))
    if np.linalg.norm(own[nearest]) > config.boundary_anchor_error_m:
        return {**base, 'reason':'path_origin_displaced'}
    prefix = own[nearest:]
    if len(prefix) < 6 or prefix[1, 0] <= prefix[0, 0]:
        return {**base, 'reason':'path_not_forward'}
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(prefix, axis=0), axis=1))]
    distances = np.linalg.norm(prefix[:, None]-other[None], axis=2)
    selected = np.argmin(distances, axis=1)
    close = (np.min(distances, axis=1) <= config.boundary_agreement_m) & (prefix[:, 0] >= .05)
    starts = np.flatnonzero(close & np.r_[True, ~close[:-1]])
    ends = np.flatnonzero(close & np.r_[~close[1:], True])+1
    # 두 번째 섬으로 넘어가며 중간의 형상 불일치를 덮지 않는다.
    if not len(starts):
        return {**base, 'reason':'no_common_prefix'}
    first, end = int(starts[0]), int(ends[0])
    try:
        origin_offset = projected_origin_offset(own, nearest)
    except ValueError:
        return {**base, 'reason':'degenerate_path_origin'}
    lead_in = float(arc[first]+origin_offset)
    if (not 0. <= lead_in <= config.length_m or end-first < 5 or arc[end-1]-arc[first] < .25
            or np.any(np.diff(selected[first:end]) < 0)):
        return {**base, 'reason':'insufficient_contiguous_common_prefix'}
    return {**base, 'supported':True, 'short_prefix':True, 'reason':'short_observed_prefix_geometry_only',
            'path':own[:nearest+end].tolist(),
            'shared_span_m':float(arc[end-1]-arc[first]),
            'one_sided_lead_in_m':lead_in, 'origin_arc_correction_m':origin_offset,
            'retained_ahead_m':float(arc[end-1]),
            'max_common_distance_m':float(np.max(np.min(distances[first:end],axis=1))),
            'opposite_path_used_for_steering':False, 'unknown_filled':False}
