import math
import numpy as np

def swept_limit(points, steering, wheelbase=.145, horizon=3., margin=.025, length=.20, width=.15):
    """정곡률 예측 차체와 점군의 첫 간섭까지 후륜축 이동 거리를 계산한다."""
    points = np.asarray(points, dtype=float)
    curvature = math.tan(steering) / wheelbase
    for distance in np.arange(0., horizon + .001, .04):
        yaw = curvature * distance
        if abs(curvature) < 1e-5:
            rx, ry = distance - wheelbase / 2, 0.
        else:
            rx, ry = math.sin(yaw) / curvature - wheelbase / 2, (1 - math.cos(yaw)) / curvature
        dx, dy = points[:, 0] - rx, points[:, 1] - ry
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy - wheelbase / 2
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        if np.any((np.abs(local_x) <= length/2 + margin) & (np.abs(local_y) <= width/2 + margin)):
            return max(0., float(distance) - .04)  # 이산 검사 간격을 안전 여유로 차감
    return horizon
