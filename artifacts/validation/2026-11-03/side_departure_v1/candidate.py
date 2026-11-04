"""격리 비교 전용. 시차 팽창의 현재 측방 겹침에서 비접근 진행을 검사한다."""
import math
import numpy as np


def candidate_limit(original, points, steering, wheelbase=.145, horizon=3., margin=.025,
                    length=.20, width=.15):
    result = original(points, steering, wheelbase, horizon, margin, length, width)
    if result > 1e-9:
        return result
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    padding = np.broadcast_to(np.asarray(margin, dtype=float), (len(points),))
    if (not len(points) or np.any(~np.isfinite(points)) or np.any(~np.isfinite(padding))
            or np.any(padding < .055) or not math.isfinite(steering) or wheelbase <= 0):
        return result
    # 점/관측은 원본에 보존한다. 이 수치는 기존 고정 여유이며 새 실측값이 아니다.
    fixed = .055
    current_overlap = ((np.abs(points[:, 0]) <= length/2+padding)
                       & (np.abs(points[:, 1]) <= width/2+padding))
    side = (current_overlap & (np.abs(points[:, 0]) <= length/2+.025)
            & (np.abs(points[:, 1]) > width/2+fixed))
    if not np.any(side) or np.any(current_overlap & ~side):
        return result
    bound = min(float(horizon), .4)
    ds = np.linspace(0., bound, max(2, math.ceil(bound/.005)+1))
    k = math.tan(steering)/wheelbase
    yaw = k*ds
    rear_x = ds if abs(k) < 1e-9 else np.sin(yaw)/k
    rear_y = np.zeros_like(ds) if abs(k) < 1e-9 else (1-np.cos(yaw))/k
    delta_x = points[None, side, 0]-(rear_x[:, None]-wheelbase/2)
    delta_y = points[None, side, 1]-rear_y[:, None]
    x = np.cos(yaw[:, None])*delta_x+np.sin(yaw[:, None])*delta_y-wheelbase/2
    y = -np.sin(yaw[:, None])*delta_x+np.cos(yaw[:, None])*delta_y
    distance = np.hypot(np.maximum(np.abs(x)-length/2, 0.), np.maximum(np.abs(y)-width/2, 0.))
    if np.any(distance < distance[:1]-.001):
        return result
    # 후보 점도 고정 측정/기계 여유로 전체 구간을 다시 검사한다.
    if original(points[side], steering, wheelbase, bound, fixed, length, width) < bound:
        return result
    if np.any(~side):
        bound = min(bound, original(points[~side], steering, wheelbase, bound,
                                    padding[~side], length, width))
    return bound
