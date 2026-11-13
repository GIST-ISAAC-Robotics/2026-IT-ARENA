"""낮은 정적 장애물용 최소 안전층. 추월/상대속도 추적/실차 안전 인증이 아닙니다."""
from dataclasses import dataclass
import math

import numpy as np


def cloud_xyz(message):
    """PointCloud2의 필드 위치·엔디언·행 패딩을 보존합니다. NaN은 거리 0이 아닙니다."""
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        raise ValueError("점군에 x/y/z 필드가 없습니다.")
    endian = ">" if message.is_bigendian else "<"
    formats = []
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype not in {7, 8} or field.count != 1:
            raise ValueError("지원하지 않는 좌표 필드 형식입니다.")
        formats.append(endian + ("f4" if field.datatype == 7 else "f8"))
    if not 0 < message.width * message.height <= 4096 or message.point_step <= 0:
        raise ValueError("비정상 ToF 점군 크기입니다.")
    if message.row_step < message.width * message.point_step or len(message.data) < message.row_step * message.height:
        raise ValueError("잘린 ToF 점군입니다.")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": formats,
                      "offsets": [fields[name].offset for name in ("x", "y", "z")],
                      "itemsize": message.point_step})
    values = np.ndarray((message.height, message.width), dtype=dtype, buffer=message.data,
                        strides=(message.row_step, message.point_step))
    points = np.column_stack([values[name].ravel() for name in ("x", "y", "z")])
    return points[np.isfinite(points).all(axis=1)]


def to_body(points, xyz, rpy):
    roll, pitch, yaw = rpy
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                         [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                         [-sp, cp * sr, cp * cr]])
    return np.asarray(points) @ rotation.T + np.asarray(xyz)


def safe_speed(clearance, latency, deceleration):
    """v*t + v²/(2a) <= clearance의 양의 해. 정적 장애물/일정 감속 가정."""
    if not all(math.isfinite(value) for value in (clearance, latency, deceleration)) or latency < 0 or deceleration <= 0:
        raise ValueError("잘못된 정지 거리 매개변수입니다.")
    return max(0.0, math.sqrt((deceleration * latency) ** 2 + 2 * deceleration * max(0, clearance))
               - deceleration * latency)


@dataclass(frozen=True)
class SafetyGeometry:
    wheelbase: float = .145
    length: float = .20
    width: float = .17  # 조향 시 바퀴 돌출을 보수적으로 포함. 검차 외형과 별개.
    margin: float = .015
    minimum_height: float = .015
    maximum_height: float = .22
    horizon: float = .50


def swept_clearance(points, steering, direction, geometry=SafetyGeometry()):
    """현재 조향각을 유지할 때 확장 직사각형이 점과 만나는 최초 주행 거리.

    점군 좌표는 base_link(축간 중앙/명목 지면) 기준입니다. 평지 높이 마스크이며
    실제 지면 추정이나 서스펜션/큰 롤·피치 보정은 아닙니다.
    """
    points = np.asarray(points).reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    points = points[(points[:, 2] >= geometry.minimum_height) &
                    (points[:, 2] <= geometry.maximum_height)]
    if not len(points):
        return math.inf
    curvature = math.tan(steering) / geometry.wheelbase
    # 표본 사이의 길이 공백은 추가 팽창으로 덮어, 얇은 표적을 건너뛰지 않습니다.
    step = .02
    half_length = geometry.length / 2 + geometry.margin + step / 2
    half_width = geometry.width / 2 + geometry.margin + step / 2
    for distance in np.linspace(0, geometry.horizon, math.ceil(geometry.horizon / step) + 1):
        s = distance * (1 if direction >= 0 else -1)
        angle = curvature * s
        ca, sa = math.cos(angle), math.sin(angle)
        if abs(curvature) < 1e-8:
            x, y = s, 0
        else:
            # 뒤축 기준 원운동을 축간 중앙 base_link 위치로 옮깁니다.
            x = sa / curvature + geometry.wheelbase / 2 * (ca - 1)
            y = (1 - ca) / curvature + geometry.wheelbase / 2 * sa
        dx, dy = points[:, 0] - x, points[:, 1] - y
        local_x, local_y = ca * dx + sa * dy, -sa * dx + ca * dy
        if np.any((np.abs(local_x) <= half_length) & (np.abs(local_y) <= half_width)):
            return float(max(0, distance - step / 2))
    return math.inf
