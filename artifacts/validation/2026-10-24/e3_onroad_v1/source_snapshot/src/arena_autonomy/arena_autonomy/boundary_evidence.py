"""선택형 경계 증거 후보. 조각 간 가림은 자유 공간으로 연결하지 않는다."""
import numpy as np


def fragment_evidence(points, reference, side, config):
    """관측 조각 길이 합계는 세되 미관측 간격 길이는 증거에서 제외한다.

    0.65 m/anchor/0.08 m는 기존 경계 계약 값이다. 조각당 5점·0.10 m,
    평균 방향 cosine >=0.8은 이 오프라인 후보에 추가한 제한이며 실측값이 아니다.
    기대 벽은 점 선택/잔차 계산에만 쓰며 반환점이나 점유를 새로 만들지 않는다.
    """
    p, path = np.asarray(points, float), np.asarray(reference, float)
    result = dict(supported=False, reason='invalid_input', fragments=[],
                  observed_ahead_sum_m=0., unknown_gaps_m=[], anchor_observed=False,
                  added_sensor_points=0, occupied_points=len(p), is_drive_permission=False)
    if (side not in ('left', 'right') or p.ndim != 2 or p.shape[1] != 2 or
            path.ndim != 2 or path.shape[1] != 2 or len(path) < 3 or
            not np.all(np.isfinite(p)) or not np.all(np.isfinite(path))):
        return result
    tangent = np.gradient(path, axis=0)
    norm = np.linalg.norm(tangent, axis=1)
    if np.any(norm < 1e-7):
        return {**result, 'reason': 'reference_tangent_invalid'}
    tangent /= norm[:, None]
    sign = 1 if side == 'left' else -1
    wall = path+sign*config.wall_offset_m*np.column_stack((-tangent[:, 1], tangent[:, 0]))
    steps = np.diff(wall, axis=0)
    lengths = np.linalg.norm(steps, axis=1)
    if (np.any(lengths < 1e-7) or
            np.any(np.sum(steps*(tangent[1:]+tangent[:-1]), axis=1) <= 0)):
        return {**result, 'reason': 'expected_wall_folded'}
    arc = np.r_[0., np.cumsum(lengths)]
    origin_index = int(np.argmin(np.linalg.norm(path, axis=1)))
    anchor_s = arc[origin_index]
    if len(p) == 0:
        return {**result, 'reason': 'no_observed_fragments'}
    delta = p[:, None]-wall[None, :-1]
    raw = np.sum(delta*steps[None], axis=2)/(lengths*lengths)
    fraction = np.clip(raw, 0., 1.)
    residual = np.linalg.norm(delta-fraction[:, :, None]*steps[None], axis=2)
    segment = np.argmin(residual, axis=1)
    row = np.arange(len(p))
    distance = residual[row, segment]
    position = arc[segment]+fraction[row, segment]*lengths[segment]
    outside = ((segment == 0) & (raw[row, segment] < 0)) | ((segment == len(steps)-1) & (raw[row, segment] > 1))
    selected = np.flatnonzero((distance <= config.boundary_agreement_m) & ~outside &
                              (sign*p[:, 1] > .10))
    selected = selected[np.argsort(position[selected], kind='stable')]
    if len(selected) < 5:
        return {**result, 'reason': 'no_observed_fragments', 'selected_count': len(selected)}
    # 같은 기존 거리-의존 연결 한도. 큰 조각 사이 간격을 길이 증거로 합치지 않는다.
    limits = .10+.035*np.minimum(np.linalg.norm(p[selected[:-1]], axis=1),
                                np.linalg.norm(p[selected[1:]], axis=1))
    gap = np.linalg.norm(np.diff(p[selected], axis=0), axis=1)
    breaks = np.flatnonzero((gap > limits) | (np.diff(position[selected]) > limits))
    fragments = []
    for indices in np.split(selected, breaks+1):
        if len(indices) < 5:
            continue
        first, last = float(position[indices[0]]), float(position[indices[-1]])
        chord = p[indices[-1]]-p[indices[0]]
        ref_chord = wall[segment[indices[-1]]+1]-wall[segment[indices[0]]]
        denom = np.linalg.norm(chord)*np.linalg.norm(ref_chord)
        aligned = bool(denom > 1e-9 and chord@ref_chord/denom >= .8)
        if last-first < .10 or not aligned:
            continue
        near = np.abs(p[indices, 0]) < .35
        anchor = bool(np.count_nonzero(near) >= 5 and
                      np.percentile(np.abs(np.abs(p[indices[near], 1])-config.wall_offset_m), 80)
                      <= config.boundary_anchor_error_m)
        fragments.append(dict(source_indices=indices.tolist(), count=len(indices),
                              first_s=first-anchor_s, last_s=last-anchor_s,
                              observed_ahead_m=max(0., last-max(first, anchor_s)),
                              anchor=anchor, p80_residual_m=float(np.percentile(distance[indices], 80))))
    observed = sum(f['observed_ahead_m'] for f in fragments)
    anchor = any(f['anchor'] for f in fragments)
    far = max((f['last_s'] for f in fragments), default=0.)
    supported = bool(anchor and observed >= config.boundary_min_ahead_m and far >= config.boundary_min_ahead_m)
    gaps = [max(0., b['first_s']-a['last_s']) for a, b in zip(fragments, fragments[1:])]
    return {**result, 'supported': supported,
            'reason': 'observed_fragments_support_reference' if supported else
                      ('anchor_not_observed' if not anchor else 'observed_length_insufficient'),
            'fragments': fragments, 'observed_ahead_sum_m': observed, 'unknown_gaps_m': gaps,
            'anchor_observed': anchor, 'selected_count': len(selected)}

