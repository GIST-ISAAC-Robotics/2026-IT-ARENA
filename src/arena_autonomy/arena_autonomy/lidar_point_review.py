"""취득 순서의 비균일 LiDAR 표본을 위한 오프라인 투영 후보. ROS 경로에 미연결.

시각/각도 출처는 호출자의 별도 계약이다. SDK/메시지에서 시각을 복원하거나,
range 복제·정렬·시계 변환·취득 시각 인증을 수행하지 않는다.
"""
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable


MAX_POINTS = 8192  # 유한 오프라인 묶음 상한. 실제 C1의 점 개수 확정값이 아니다.


def _array(values, label, finite):
    # 문자열/bool을 수치로 조용히 바꾸지 않는다. 배열 전체를 검사한 뒤 투영한다.
    try:
        raw = np.asarray(values)
    except (TypeError, ValueError) as error:
        raise MotionUnavailable('point_'+label+'_invalid') from error
    if raw.ndim != 1 or raw.dtype.kind not in 'iuf' or not 2 <= len(raw) <= MAX_POINTS:
        raise MotionUnavailable('point_'+label+'_invalid')
    if any(isinstance(v, (bool, np.bool_)) for v in np.asarray(values, dtype=object)):
        raise MotionUnavailable('point_'+label+'_invalid')
    result = np.asarray(raw, dtype=float)
    if finite and not np.all(np.isfinite(result)):
        raise MotionUnavailable('point_'+label+'_invalid')
    return result


def project_acquisition_points(history, ranges, angles_rad, stamps_s, now,
                               range_min, range_max, mode='both', timing_verified=False):
    """원본 취득 순서 → 최신 점 시각(deskew) 또는 현재(both)의 base XY.

    반환 인덱스는 입력의 원래 인덱스다. 무효 range도 시각 검사를 생략하지 않으며
    자유 공간으로 만들지 않는다. timing_verified는 호출자의 명시적 계약 승인이지
    이 함수가 센서/시계 정확도를 확인했다는 결과가 아니다.
    """
    if mode not in ('deskew', 'both'):
        raise ValueError('explicit point review supports deskew or both only')
    if timing_verified is not True:
        raise MotionUnavailable('scan_timing_unverified')
    for value in (now, range_min, range_max):
        if (not isinstance(value, (int, float, np.integer, np.floating))
                or isinstance(value, (bool, np.bool_)) or not math.isfinite(value)):
            raise MotionUnavailable('point_metadata_invalid')
    if now < 0 or range_min < 0 or range_max <= range_min:
        raise MotionUnavailable('point_metadata_invalid')
    distances = _array(ranges, 'ranges', finite=False)
    angles = _array(angles_rad, 'angles', finite=True)
    stamps = _array(stamps_s, 'stamps', finite=True)
    if not (len(distances) == len(angles) == len(stamps)):
        raise MotionUnavailable('point_array_length_mismatch')
    if stamps[0] < 0 or not np.all(np.diff(stamps) > 0):
        raise MotionUnavailable('point_acquisition_order_invalid')
    # strict acquisition order는 이 후보의 입력 조건이다. clock 해상도 때문에 같은
    # 시각인 실측 표본은 임의 시간을 덧붙이지 말고 별도 표현/계약부터 검토한다.
    first, last = float(stamps[0]), float(stamps[-1])
    if last > now+1e-9:
        raise MotionUnavailable('scan_future')
    if now-first >= history.config.scan_timeout_s:
        raise MotionUnavailable('scan_stale')
    indices = np.flatnonzero(np.isfinite(distances)
                             & (distances >= range_min) & (distances <= range_max))
    ray_angles = angles[indices]+history.config.lidar_yaw_rad
    points = np.column_stack((distances[indices]*np.cos(ray_angles)+history.config.lidar_x_m,
                              distances[indices]*np.sin(ray_angles)+history.config.lidar_y_m))
    reference = float(now) if mode == 'both' else last
    poses, diagnostic = history.poses(np.r_[stamps[indices], reference], now)
    source, target = poses[:-1], poses[-1]
    c, s = np.cos(source[:, 2]), np.sin(source[:, 2])
    wx = c*points[:, 0]-s*points[:, 1]+source[:, 0]-target[0]
    wy = s*points[:, 0]+c*points[:, 1]+source[:, 1]-target[1]
    c, s = math.cos(target[2]), math.sin(target[2])
    transformed = np.column_stack((c*wx+s*wy, -s*wx+c*wy))
    return transformed, indices, dict(diagnostic, mode=mode, scan_stamp_s=first,
        last_ray_stamp_s=last, reference_stamp_s=reference,
        oldest_observation_age_s=float(now-first), newest_observation_age_s=float(now-last),
        valid_points=len(indices), input_points=len(distances),
        point_time_source='caller_verified_explicit_acquisition_stamps',
        source_order_preserved=True, resampled=False, hardware_verified=False,
        review_only=True, ready_to_drive=False)
