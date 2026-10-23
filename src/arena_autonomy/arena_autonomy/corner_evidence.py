"""선택형 급커브 증거: 관측된 긴 바깥 곡선과 현재 차체를 가로지르는 안쪽 벽.

직선의 전방 벽 길이 계약을 몰래 낮추지 않는다. 별도 후보에서만 사용하는
기하 계약이며, 경계 확인 외에 자유 공간/차량 분류/주행 허가를 주장하지 않는다.
"""
import math

import numpy as np


def corner_anchor_evidence(points, reference, side, config):
    """반대 벽의 충분한 관측/앵커 확인은 호출자 의무. 점유점은 수정하지 않는다.

    바깥쪽 경로가 안쪽으로 30~135도 돌아야 한다. 안쪽의 한 연결 관측은
    차체 앞/뒤를 모두 포함하고 총 길이 0.65m 이상이어야 한다. 실제 점의
    직선성/근처 경로와의 간격/전방 경로의 모서리 이격을 별도로 확인한다.
    30/135/15도는 후보 범위 제한이지 실물로 확인한 한도가 아니다.
    """
    p, path = np.asarray(points, dtype=float), np.asarray(reference, dtype=float)
    result = dict(supported=False, reason='invalid_corner_input', source_indices=[],
                  added_sensor_points=0, occupied_points=len(p) if p.ndim else 0, is_drive_permission=False,
                  free_area_certified=False, unknown_gaps_filled=False)
    if (side not in ('left','right') or p.ndim != 2 or p.shape[1] != 2 or
            path.ndim != 2 or path.shape[1] != 2 or len(path) < 12 or len(p) < 12 or
            not np.all(np.isfinite(p)) or not np.all(np.isfinite(path))):
        return result
    sign = 1 if side == 'left' else -1
    delta = np.diff(path, axis=0)
    distances = np.linalg.norm(delta, axis=1)
    if np.any(distances < 1e-7):
        return {**result, 'reason':'corner_reference_degenerate'}
    tangent = np.gradient(path,axis=0)
    norm = np.linalg.norm(tangent,axis=1)
    if np.any(norm < 1e-7):
        return {**result, 'reason':'corner_reference_degenerate'}
    tangent /= norm[:,None]
    arc = np.r_[0.,np.cumsum(distances)]
    origin = int(np.argmin(np.linalg.norm(path,axis=1)))
    angles = np.unwrap(np.arctan2(tangent[:,1],tangent[:,0]))
    turn = sign*(angles[-1]-angles[origin])
    backturn = np.maximum(0.,-sign*np.diff(angles[origin:])).sum()
    result.update(turn_rad=float(turn), opposing_turn_rad=float(backturn))
    if (not math.radians(30) <= turn <= math.radians(135) or backturn > math.radians(5)
            or tangent[origin,0] <= 0 or sign*(path[-1,1]-path[origin,1]) <= 0):
        return {**result, 'reason':'not_consistent_inward_corner'}
    near = (arc >= arc[origin]-.1) & (arc <= arc[origin]+.1)
    if np.count_nonzero(near) < 3 or np.ptp(arc[near]) < .1:
        return {**result, 'reason':'corner_near_reference_short'}
    # 원시 각도 순서의 연결 구간만 사용한다. 공간적으로 가까운 다른 조각과 합치지 않는다.
    order = np.argsort(-sign*np.arctan2(p[:,1],p[:,0]),kind='stable')
    ordered = p[order]
    limits = .10+.035*np.minimum(np.linalg.norm(ordered[:-1],axis=1),
                                np.linalg.norm(ordered[1:],axis=1))
    breaks = np.flatnonzero(np.linalg.norm(np.diff(ordered,axis=0),axis=1)>limits)
    failures = []
    accepted = []
    for indices in np.split(order,breaks+1):
        # 가까운 실제 관측만 앵커로 쓴다. 제외 구간을 건너뛰어 연결하지 않는다.
        mask = (np.abs(p[indices,0]) <= 2*config.wall_offset_m) & (sign*p[indices,1] > .10)
        starts = np.flatnonzero(mask & np.r_[True,~mask[:-1]])
        ends = np.flatnonzero(mask & np.r_[~mask[1:],True])+1
        for a,b in zip(starts,ends):
            selected = indices[a:b]
            if len(selected) < 12:
                continue
            wall = p[selected]
            center = wall.mean(axis=0)
            _,_,axes = np.linalg.svd(wall-center,full_matrices=False)
            direction = axes[0] if axes[0,0] >= 0 else -axes[0]
            normal = sign*np.array([-direction[1],direction[0]])
            along = (wall-center)@direction
            residual = np.abs((wall-center)@normal)
            length = float(np.ptp(along))
            front, rear = float(np.max(wall[:,0])),float(-np.min(wall[:,0]))
            detail = dict(observed_length_m=length,front_extent_m=front,rear_extent_m=rear,
                          p90_line_error_m=float(np.percentile(residual,90)))
            if length < config.boundary_min_ahead_m or min(front,rear) < config.length_m/2:
                failures.append('corner_anchor_extent_insufficient')
                continue
            if (detail['p90_line_error_m'] > config.measurement_padding_m or
                    float(np.max(residual)) > config.boundary_agreement_m):
                failures.append('corner_anchor_not_straight')
                continue
            # 전체 PCA만으로는 완만한 두 면을 한 직선으로 오인할 수 있다.
            # 같은 연결 관측의 앞/중간/뒤 방향도 대조한다. 잡음에 민감한
            # 인접 두 점 기울기 대신 각 1/3 길이 구간의 PCA를 사용한다.
            headings = []
            cuts = np.linspace(float(np.min(along)), float(np.max(along)), 4)
            for lo, hi in zip(cuts[:-1], cuts[1:]):
                part = wall[(along >= lo-1e-9) & (along <= hi+1e-9)]
                if len(part) < 4:
                    break
                _, _, axes = np.linalg.svd(part-part.mean(axis=0), full_matrices=False)
                direction_part = axes[0] if axes[0, 0] >= 0 else -axes[0]
                headings.append(math.atan2(direction_part[1], direction_part[0]))
            spread = float(np.ptp(np.unwrap(headings))) if len(headings) == 3 else math.inf
            if spread > math.radians(10):
                failures.append('corner_anchor_heading_inconsistent')
                continue
            detail['anchor_heading_spread_rad'] = spread
            if direction@tangent[origin] < math.cos(math.radians(15)):
                failures.append('corner_anchor_heading_mismatch')
                continue
            # 같은 근접 구간에서 두 경로가 맞는지 대신, 원래 안쪽 벽과의 간격을 직접 확인한다.
            separation = -(path[near]-center)@normal
            if (np.any(separation <= 0) or
                    np.max(np.abs(separation-config.wall_offset_m)) > config.boundary_agreement_m):
                failures.append('corner_anchor_offset_mismatch')
                continue
            minimum = float(np.min(np.linalg.norm(path[origin:,None]-wall[None],axis=2)))
            if minimum < config.wall_offset_m-config.boundary_agreement_m:
                failures.append('corner_reference_cuts_inner_wall')
                continue
            detail.update(minimum_inner_distance_m=minimum,
                          maximum_near_offset_error_m=float(np.max(np.abs(separation-config.wall_offset_m))),
                          source_indices=selected.tolist())
            accepted.append(detail)
    if not accepted:
        return {**result,'reason':failures[0] if failures else 'corner_anchor_not_observed',
                'rejected_anchor_reasons':sorted(set(failures))}
    best=min(accepted,key=lambda row:row['maximum_near_offset_error_m'])
    return {**result,**best,'supported':True,'reason':'observed_corner_anchor_support'}
