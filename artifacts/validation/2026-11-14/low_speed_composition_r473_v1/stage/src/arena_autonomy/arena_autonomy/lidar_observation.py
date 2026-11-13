"""ROS와 분리된 전방 LiDAR 관측 입력 계약.

미관측 거리를 자유 공간으로 채우지 않으며 거리 배열을 수정하지 않는다.
2° 최대 각 간격은 임시 제어 입력 계약으로, C1 제조사 성능 주장이 아니다.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class FrontObservationAudit:
    reason: str
    forward_samples: int = 0
    unknown_samples: int = 0


def audit_front_observation(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    lidar_yaw_rad=0.,
    half_angle_rad=math.radians(100),
):
    """한 회전 배열과 차체 전방 관측을 검사한다.

    정규 한 회전(N*간격≈2π)과 양 끝 포함((N-1)*간격≈2π)을
    허용한다. 한 표본의 회전 길이 허용오차와 별개로, 전방의 각도
    공백은 정상 표본 간격보다 클 수 없다. 기본 ±100° 밖의 결측은
    기존 전방 게이트 범위에 따라 의도적으로 무시한다.
    unknown_samples는 배열 전체가 아니라 전방 거리 결측 개수이다.
    각도 공백만 있는 front_unobserved의 unknown_samples는 0일 수 있다.
    """
    invalid = FrontObservationAudit('scan_invalid')
    try:
        metadata = [np.asarray(value) for value in (
            angle_min, angle_increment, range_min, range_max,
            lidar_yaw_rad, half_angle_rad,
        )]
        if any(value.ndim != 0 or value.dtype.kind not in 'iuf'
               for value in metadata):
            return invalid
        start, increment, minimum, maximum, yaw, half_angle = (
            float(value) for value in metadata
        )
        values = np.asarray(ranges)
        if values.ndim != 1 or values.size < 2 or values.dtype.kind not in 'iuf':
            return invalid
        values = values.astype(float, copy=False)
    except (TypeError, ValueError, OverflowError):
        return invalid
    if not all(math.isfinite(value) for value in (
        start, increment, minimum, maximum, yaw, half_angle,
    )):
        return invalid
    if (not 0 < increment <= math.radians(2) or
            not 0 <= minimum < maximum or not 0 < half_angle <= math.pi):
        return invalid

    turn = 2 * math.pi
    # float32 LaserScan 각도 메타데이터의 누적 반올림도 허용한다.
    # 각 간격 하나의 float32 반올림이 한 회전까지 누적된다. 간격 크기에만
    # 비례시키면 정상 513점 inclusive 배열도 거절될 수 있다.
    tolerance = max(1e-7, turn * np.finfo(np.float32).eps, increment * 1e-5)
    span = (values.size - 1) * increment
    if (not math.isfinite(span) or span > turn + tolerance or
            abs(values.size * increment - turn) > increment + tolerance):
        return invalid
    # 큰 유한 장착 각도를 먼저 축약하여 덧셈 overflow를 피한다.
    origin = math.remainder(start, turn) + math.remainder(yaw, turn)
    angles = (origin + np.arange(values.size) * increment + math.pi) % turn - math.pi
    forward = np.abs(angles) <= half_angle + tolerance
    forward_count = int(np.count_nonzero(forward))
    valid = np.isfinite(values) & (values >= minimum) & (values <= maximum)
    unknown_count = int(np.count_nonzero(forward & ~valid))

    # 회전 길이 허용오차가 전방 wrap seam의 누락을 감추지 않게 한다.
    ordered = np.sort(angles)
    gaps = np.diff(np.concatenate((ordered, [ordered[0] + turn])))
    midpoints = (ordered + gaps / 2 + math.pi) % turn - math.pi
    front_hole = bool(np.any(
        (gaps > increment + tolerance) &
        (np.abs(midpoints) < half_angle + gaps / 2 - tolerance)
    ))
    if forward_count == 0 or unknown_count or front_hole:
        return FrontObservationAudit('front_unobserved', forward_count, unknown_count)
    return FrontObservationAudit('ok', forward_count, 0)
