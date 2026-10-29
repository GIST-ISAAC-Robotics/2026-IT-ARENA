"""저장 입력 진단: 옛 반환→결측 취득→현재의 두 이동 시간을 분리한다.

실제 물체 추적·주행 허가가 아니다. 기존 2 m/s 가정에서 결측 시각의
가능 위치가 현재 감속 외곽에 도달할 수 있는지 보수적인 직사각형 팽창으로
검사한다. 미래 물체 운동·실측 센서 오차 분포까지 보장하지 않는다.
"""
import math

import numpy as np

from probe_joint_missing_sweep import joint_missing_sweep


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
