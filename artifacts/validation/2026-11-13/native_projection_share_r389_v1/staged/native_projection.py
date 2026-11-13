"""동일 호출의 검증된 native scene 기하를 raw points 투영에 공유하는 후보.

호출자가 한 판단 안에서만 이 객체를 소유한다. 시간/운동 이력 검증은 기존
scene_motion에서 각각 수행하며, 입력이 조금이라도 다르면 원 함수로 돌아간다.
장면의 masked/unknown은 변경하지 않는다. 점 배열은 매번 새로 반환한다.
"""
import math

import numpy as np

from arena_autonomy.local_scene import _finite, _xy, _reusable_blind_spec
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.native_geometry import native_motion_points, native_motion_scene


def _extrinsics(config):
    x, y = _xy((config.lidar_x_m, config.lidar_y_m), 'sensor_xy_m')
    yaw = _finite(config.lidar_yaw_rad, 'sensor_yaw_rad')
    return x.hex(), y.hex(), yaw.hex()


class SameCallNativeProjection:
    """성공한 장면 하나의 기하. 전역/다른 판단 사이 저장 용도가 아니다."""
    __slots__ = ('_scan', '_config', '_extrinsics', '_now', '_epoch',
                 '_dtype', '_shape', '_transforms', '_scene')

    def __init__(self, scan, transforms, now, config, epoch, scene):
        # native_motion_scene 성공 뒤에만 내부 factory가 호출한다.
        self._scan, self._config = scan, config
        self._extrinsics = _extrinsics(config)
        self._now, self._epoch = _finite(now, 't_ref_s').hex(), epoch
        self._dtype, self._shape = transforms.dtype.str, transforms.shape
        self._transforms = transforms.tobytes(order='C')
        self._scene = scene

    def matches(self, scan, transforms, now, config, epoch):
        # 배열 값 비교에서 +0/-0를 같게 보지 않는다. deepcopy/동일 내용의
        # 다른 scan, 다른 config, 다른 epoch도 공유하지 않는 보수적 경계다.
        if (scan is not self._scan or type(scan) is not MeasurementScan
                or config is not self._config or type(epoch) is not int
                or epoch != self._epoch or type(transforms) is not np.ndarray
                or transforms.dtype.str != self._dtype or transforms.shape != self._shape):
            return False
        try:
            return (_finite(now, 't_ref_s').hex() == self._now
                    and _extrinsics(config) == self._extrinsics
                    and transforms.tobytes(order='C') == self._transforms)
        except (AttributeError, TypeError, ValueError, OverflowError):
            # 실패를 성공으로 바꾸지 않고 원 함수에서 원 예외를 발생시킨다.
            return False

    def points(self, scan, transforms, now, config, epoch):
        if not self.matches(scan, transforms, now, config, epoch):
            return native_motion_points(scan, transforms, now, config, epoch)
        return self._points()

    def _points(self):
        scan = self._scan
        points, indices = [], []
        for index, (distance, ray) in enumerate(zip(scan.ranges_m, self._scene.rays)):
            if not (math.isfinite(distance) and scan.range_min_m <= distance <= scan.range_max_m):
                continue
            hit = ray.hit_xy_m
            if hit is None:
                # scene이 masked여도 raw points에서는 유효 반환을 보존한다.
                # masked overflow는 장면 성공 이후 원 points 요청 시점에만 거절한다.
                origin, direction = ray.ray_origin_xy_m, ray.direction_xy
                hit = (origin[0] + distance*direction[0], origin[1] + distance*direction[1])
                if not all(math.isfinite(v) for v in hit):
                    raise ValueError('hit must be finite numeric')
            points.append(hit)
            indices.append(index)
        return np.asarray(points, dtype=float).reshape(-1, 2), np.asarray(indices, dtype=int)


def native_motion_scene_with_projection(scan, transforms, now, config, epoch, mask):
    """원 scene과 같은 호출에서만 쓸 공유 객체. 비표준 mask는 원 경로만 쓴다."""
    scene = native_motion_scene(scan, transforms, now, config, epoch, mask)
    shared = None
    if (type(scan) is MeasurementScan and type(transforms) is np.ndarray
            and type(config) is MotionConfig and type(now) in (int, float)
            and type(epoch) is int and type(mask) in (int, float, dict)
            and _reusable_blind_spec(mask)):
        shared = SameCallNativeProjection(scan, transforms, now, config, epoch, scene)
    return scene, shared
