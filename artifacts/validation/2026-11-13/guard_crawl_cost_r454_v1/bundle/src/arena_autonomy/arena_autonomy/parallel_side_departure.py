"""현재 측방 시차 팽창과만 겹칠 때의 제한 전진. 미래 옆차 접근을 보장하지 않는다.

관측 반환과 고정 측정 여유는 보존한다. 상대 정지/같은 물체라는 가정이나
자유 공간 채움 없이, 두 조향의 차체가 해당 반환에 더 접근하는지 검사한다.
완전 관측·평행 경로·확인 횟수·독립 보호는 호출자의 기존 조건이다.
"""
import math

import numpy as np

from arena_autonomy.local_path import swept_limit


def limited_clearance(points, angles, padding, speed, age, dt, config):
    """0.4 m 이내 선택형 기하 후보와 적용 이유. 2 m/s² 제동은 임시 근사다."""
    window = .4
    fixed = .025 + config.measurement_padding_m
    stopping = speed*(age+.1+dt) + speed*speed/4
    info = dict(applied=False, reason='motion_outside_window', window_m=window,
                fixed_padding_m=fixed, measured_stopping_m=stopping,
                future_side_approach_unresolved=True, unknown_filled=False)
    if (not all(math.isfinite(v) for v in (speed, age, dt, stopping)) or speed < 0
            or stopping > window-config.standstill_gap_m):
        return 0., info
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    padding = np.broadcast_to(np.asarray(padding, dtype=float), (len(points),))
    if (not len(points) or np.any(~np.isfinite(points)) or np.any(~np.isfinite(padding))
            or np.any(padding < fixed-1e-12)):
        return 0., {**info, 'reason': 'invalid_occupancy'}
    length, width, wheelbase = config.length_m, config.width_m, config.wheelbase_m
    overlap = ((np.abs(points[:, 0]) <= length/2+padding)
               & (np.abs(points[:, 1]) <= width/2+padding))
    # 전진하며 뒤로 넘어간 점에도 비접근/고정 여유 검사를 적용한다. 앞 모서리 제한만 유지.
    side = (overlap & (points[:, 0] <= length/2+.025)
            & (np.abs(points[:, 1]) > width/2+fixed))
    info.update(side_points=int(side.sum()), current_overlap_points=int(overlap.sum()))
    if not np.any(side) or np.any(overlap & ~side):
        return 0., {**info, 'reason': 'overlap_not_only_side_timing'}
    clearance = window
    distances = np.linspace(0., window, 81)
    for angle in sorted(set(angles)):
        k = math.tan(angle)/wheelbase
        yaw = k*distances
        rear_x = distances if abs(k) < 1e-9 else np.sin(yaw)/k
        rear_y = np.zeros_like(distances) if abs(k) < 1e-9 else (1-np.cos(yaw))/k
        dx = points[None, side, 0]-(rear_x[:, None]-wheelbase/2)
        dy = points[None, side, 1]-rear_y[:, None]
        x = np.cos(yaw[:, None])*dx+np.sin(yaw[:, None])*dy-wheelbase/2
        y = -np.sin(yaw[:, None])*dx+np.cos(yaw[:, None])*dy
        gap = np.hypot(np.maximum(np.abs(x)-length/2, 0.), np.maximum(np.abs(y)-width/2, 0.))
        if np.any(gap < gap[:1]-.001):
            return 0., {**info, 'reason': 'approaches_observed_side'}
        # 후보 점도 삭제하지 않는다. 이 점들만 기존 고정 측정/기계 여유로 검사한다.
        if swept_limit(points[side], angle, wheelbase, window, fixed, length, width) < window:
            return 0., {**info, 'reason': 'fixed_side_clearance'}
        if np.any(~side):
            clearance = min(clearance, swept_limit(points[~side], angle, wheelbase, window,
                                                   padding[~side], length, width))
    return clearance, {**info, 'applied': clearance > 0, 'reason': 'nonapproaching_side_window',
                       'clearance_m': clearance}
