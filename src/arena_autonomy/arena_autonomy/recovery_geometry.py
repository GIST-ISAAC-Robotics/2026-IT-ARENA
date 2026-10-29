"""선택형 점 취득 시각 기반 위협 교집합. 2 m/s 가정이며 자유 공간 증명이 아니다."""
import math
import numpy as np


def clip_polygon(polygon, normal, limit):
    """볼록 다각형과 n·x <= limit 반평면의 교집합."""
    if not len(polygon):
        return np.empty((0, 2))
    out = []
    previous = polygon[-1]
    old = float(previous @ normal - limit)
    for current in polygon:
        new = float(current @ normal - limit)
        if (old <= 0.) != (new <= 0.):
            out.append(previous + old / (old - new) * (current - previous))
        if new <= 0.:
            out.append(current)
        previous, old = current, new
    return np.asarray(out, dtype=float).reshape(-1, 2)


def polygon_distance(points, polygon):
    if not len(polygon):
        return np.full(len(points), math.inf)
    if len(polygon) == 1:
        return np.linalg.norm(points - polygon[0], axis=1)
    edges = np.roll(polygon, -1, axis=0) - polygon
    delta = points[:, None, :] - polygon[None, :, :]
    projection = np.sum(delta * edges, axis=2) / np.maximum(np.sum(edges**2, axis=1), 1e-30)
    residual = delta - np.clip(projection, 0., 1.)[:, :, None] * edges
    distances = np.min(np.linalg.norm(residual, axis=2), axis=1)
    # CW/CCW 양쪽을 처리하며, 퇴화 선분을 면으로 취급하지 않는다.
    cross = edges[None, :, 0] * delta[:, :, 1] - edges[None, :, 1] * delta[:, :, 0]
    area = abs(float(np.sum(polygon[:, 0] * np.roll(polygon[:, 1], -1) -
                            polygon[:, 1] * np.roll(polygon[:, 0], -1))))
    if area > 1e-15:
        inside = np.all(cross >= -1e-12, axis=1) | np.all(cross <= 1e-12, axis=1)
        distances[inside] = 0.
    return distances


def sweep_rectangles(speed, age, steering, previous_steering, wheelbase, length, width):
    horizon = speed * (age + .10) + speed**2 / 4 + .06
    distances = np.linspace(0., horizon, max(2, math.ceil(horizon / .02) + 1))
    spacing = float(distances[1] - distances[0])
    # 샘플 사이 차체 이동/회전도 감싸는 추가 반경. 기존 .02 m 여유는 유지한다.
    corner_radius = math.hypot(length / 2 + wheelbase / 2, width / 2)
    for angle in sorted(set((steering, previous_steering))):
        curvature = math.tan(angle) / wheelbase
        padding = .02 + spacing / 2 * (1 + corner_radius * abs(curvature))
        for distance in distances:
            yaw = distance * curvature
            c, s = math.cos(yaw), math.sin(yaw)
            rear_x = distance if abs(curvature) < 1e-8 else s / curvature
            rear_y = 0. if abs(curvature) < 1e-8 else (1 - c) / curvature
            center = np.array([rear_x - wheelbase / 2 + c * wheelbase / 2,
                               rear_y + s * wheelbase / 2])
            yield center, np.array([[c, s], [-s, c]]), np.array([length / 2, width / 2]) + padding


def joint_missing_sweep(points, origins, directions, half_angle, required, speed, age,
                        steering, previous_steering, wheelbase=.135, length=.20, width=.15):
    """각 이전 점→(결측 쐐기 ∩ 감속 차체)의 최소 거리, 하나라도 불명확하면 예외."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    origins = np.asarray(origins, dtype=float).reshape(-1, 2)
    directions = np.asarray(directions, dtype=float).reshape(-1)
    numbers = np.r_[points.ravel(), origins.ravel(), directions, half_angle, required,
                    speed, age, steering, previous_steering, wheelbase, length, width]
    if (not np.all(np.isfinite(numbers)) or len(origins) != len(directions) or
            not 0 <= len(directions) <= 16 or len(points) > 1024 or
            not 0 < half_angle <= math.radians(3) or not .1 <= required <= 2 or
            not 0 <= speed <= 1.2 or not 0 <= age < .3 or
            max(abs(steering), abs(previous_steering)) > .6 or
            min(wheelbase, length, width) <= 0):
        raise ValueError('joint missing-sweep input outside bounded shadow comparison')
    rectangles = list(sweep_rectangles(speed, age, steering, previous_steering, wheelbase, length, width))
    minimum = np.full(len(points), math.inf)
    intersections = []
    for origin, direction in zip(origins, directions):
        along = np.array([math.cos(direction), math.sin(direction)])
        normal = np.array([-along[1], along[0]])
        half_width = required * math.tan(half_angle)
        wedge = np.array([origin, origin + required * along - half_width * normal,
                          origin + required * along + half_width * normal])
        for center, axes, half_sizes in rectangles:
            polygon = wedge.copy()
            for axis, half_size in zip(axes, half_sizes):
                polygon = clip_polygon(polygon, axis, float(axis @ center + half_size))
                polygon = clip_polygon(polygon, -axis, float(-axis @ center + half_size))
            if len(polygon):
                minimum = np.minimum(minimum, polygon_distance(points, polygon))
                intersections.append(polygon.tolist())
    return minimum, intersections


def timed_missing_sweep(points, acquired_s, origins, directions, missing_s,
                        now, half_angle, required, speed, age, steering,
                        previous_steering, *, motion_bound=2., radial_margin=.03,
                        wheelbase=.135, length=.20, width=.15):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    acquired = np.asarray(acquired_s, dtype=float).reshape(-1)
    origins = np.asarray(origins, dtype=float).reshape(-1, 2)
    directions = np.asarray(directions, dtype=float).reshape(-1)
    missing = np.asarray(missing_s, dtype=float).reshape(-1)
    numbers = np.r_[acquired, missing, now, motion_bound, radial_margin]
    if (len(points) != len(acquired) or len(origins) != len(missing)
            or len(directions) != len(missing) or not len(missing)
            or not np.all(np.isfinite(numbers)) or motion_bound < 0
            or radial_margin < 0 or np.any(missing > now)
            or np.any(acquired[:, None] > missing[None, :])):
        raise ValueError('inconsistent acquisition times or motion bounds')
    distances, radii, post = [], [], []
    for origin, direction, stamp in zip(origins, directions, missing):
        # 반경 r 원형으로 팽창한 차체는 각 축으로 r 늘린 사각형에 포함된다.
        # 원형 도달 집합을 포함하는 사각 외곽이므로 거짓 배제를 줄이는 쪽이다.
        extra = motion_bound * (now - stamp)
        dist, _ = joint_missing_sweep(
            points, [origin], [direction], half_angle, required, speed, age,
            steering, previous_steering, wheelbase, length + 2*extra, width + 2*extra)
        distances.append(dist)
        radii.append(radial_margin + motion_bound * (stamp - acquired))
        post.append(extra)
    distances, radii = np.asarray(distances), np.asarray(radii)
    return dict(possible=np.any(distances <= radii + 1e-10, axis=0).tolist(),
                lower_distance_m=distances.tolist(), before_radius_m=radii.tolist(),
                after_radius_m=post,
                note='conservative current-time feasibility only; not object identity or future safety')
