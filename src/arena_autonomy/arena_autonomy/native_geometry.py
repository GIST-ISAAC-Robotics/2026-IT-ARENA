"""검증된 불변 원천 측정의 투영. 공개 광선 생성자의 검증은 유지한다.

점군만 필요한 호출은 광선 객체를 만들지 않는다. 장면 생성은 공통 입력을 한 번
검증하고 출력 기하를 검사한 뒤 내부에서 광선을 구성한다. 재표본화하지 않는다.
취득 시각/운동 이력의 신뢰도 검사는 scene_motion 호출 경계의 책임이다.
"""
import math
import numpy as np

from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.local_scene import (
    SensorRay, LocalScene, _finite, _xy, _source, _blind_limits, _masked_by,
    _reusable_blind_spec, scene_from_measurements,
)


def _geometry(scan, transforms, now, config, epoch, mask):
    if type(scan) is not MeasurementScan:
        raise ValueError('validated immutable MeasurementScan required')
    if (not isinstance(transforms, np.ndarray) or transforms.dtype.kind != 'f'
            or transforms.shape != (len(scan.ranges_m), 3)
            or not np.isfinite(transforms).all()):
        raise ValueError('finite floating Nx3 transforms required')
    sx, sy = _xy((config.lidar_x_m, config.lidar_y_m), 'sensor_xy_m')
    yaw = _finite(config.lidar_yaw_rad, 'sensor_yaw_rad')
    now = _finite(now, 't_ref_s')
    _source(scan.source_frame_id, epoch)
    if scan.acquired_at_s[-1] > now:
        raise ValueError('source interval/reference time mismatch')
    limits = _blind_limits(mask)
    self_masked = frozenset(scan.self_masked_indices)
    for index, (distance, pose, source_angle) in enumerate(zip(
            scan.ranges_m, transforms.tolist(), scan.angles_rad)):
        dx, dy, turn = pose
        c, s = math.cos(turn), math.sin(turn)
        origin = (dx + c*sx - s*sy, dy + s*sx + c*sy)
        angle = math.remainder(source_angle, math.tau)
        masked = index in self_masked or _masked_by(math.remainder(angle, math.tau), limits,distance)
        valid = math.isfinite(distance) and scan.range_min_m <= distance <= scan.range_max_m
        status = 'masked' if masked else ('return' if valid else 'unknown')
        bearing = angle + yaw + turn
        direction = (math.cos(bearing), math.sin(bearing))
        hit = ((origin[0] + distance*direction[0], origin[1] + distance*direction[1])
               if status == 'return' else None)
        # 유한한 입력끼리의 계산에서도 overflow가 생길 수 있다.
        if not all(math.isfinite(v) for v in origin):
            raise ValueError('origin must be finite numeric')
        if hit is not None and not all(math.isfinite(v) for v in hit):
            raise ValueError('hit must be finite numeric')
        if abs(math.hypot(*direction) - 1.) > 1e-10:
            raise ValueError('direction_xy must be unit length')
        yield index, origin, direction, hit, status


def native_motion_points(scan, transforms, now, config, epoch):
    """마스크 없는 유효 반환 좌표와 취득 순서 인덱스만 반환한다."""
    points, indices = [], []
    for index, _, _, hit, _ in _geometry(scan, transforms, now, config, epoch, 0.):
        if hit is not None:
            points.append(hit)
            indices.append(index)
    return np.asarray(points, dtype=float).reshape(-1, 2), np.asarray(indices, dtype=int)


def native_motion_scene(scan, transforms, now, config, epoch, mask):
    """동일한 기하와 unknown/시간/출처를 보존하되 반복 형식 검사를 줄인다."""
    if not _reusable_blind_spec(mask):
        return scene_from_measurements(scan.ranges_m, angles_rad=scan.angles_rad,
            acquired_at_s=scan.acquired_at_s, t_ref_s=now, range_min=scan.range_min_m,
            range_max=scan.range_max_m, source_frame_id=scan.source_frame_id, source_epoch=epoch,
            sensor_xy_m=(config.lidar_x_m, config.lidar_y_m), sensor_yaw_rad=config.lidar_yaw_rad,
            acquisition_to_ref_se2=transforms.tolist(), rear_half_angle_deg=mask,
            preserve_sampling=True, self_masked_indices=scan.self_masked_indices)
    rays = []
    assign = object.__setattr__
    for index, origin, direction, hit, status in _geometry(scan, transforms, now, config, epoch, mask):
        # 범위/시각/출처는 불변 MeasurementScan, 상태/인덱스는 이 루프,
        # 좌표/방향은 위 계산에서 검사했다. 공개 SensorRay 생성자는 그대로 둔다.
        ray = object.__new__(SensorRay)
        assign(ray, 'ray_origin_xy_m', origin)
        assign(ray, 'hit_xy_m', hit)
        assign(ray, 'acquired_at_s', scan.acquired_at_s[index])
        assign(ray, 'status', status)
        assign(ray, 'source_frame_id', scan.source_frame_id)
        assign(ray, 'source_epoch', epoch)
        assign(ray, 'source_index', index)
        assign(ray, 'direction_xy', direction)
        rays.append(ray)
    return LocalScene(now, scan.acquired_at_s[0], scan.acquired_at_s[-1],
                      scan.source_frame_id, epoch, tuple(rays), source_angles_rad=scan.angles_rad)
