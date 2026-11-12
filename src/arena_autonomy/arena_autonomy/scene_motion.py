"""취득 시각을 보존하는 LocalScene ↔ 엔코더/자이로 평면 운동 어댑터.

Gazebo 자세·명령 속도를 사용하지 않는다. 수평 LiDAR, 검증된 순차 취득 시각,
기존 MotionHistory의 ZOH/공백/외삽 한도를 전제로 한다.
"""
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_scene import MAX_RAYS, scene_from_scan, scene_from_measurements
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.lidar_observation import audit_front_observation


def relative_poses(history, source_times, reference_time):
    """각 base_link(source)에서 base_link(reference)로 가는 SE(2)."""
    poses, _ = history.poses(np.r_[source_times, reference_time], reference_time)
    source, target = poses[:-1], poses[-1]
    delta = source[:, :2] - target[:2]
    c, s = math.cos(target[2]), math.sin(target[2])
    return np.column_stack((c*delta[:, 0]+s*delta[:, 1],
                            -s*delta[:, 0]+c*delta[:, 1], source[:, 2]-target[2]))


def transport_points(points, history, source_time, reference_time):
    dx, dy, yaw = relative_poses(history, [source_time], reference_time)[0]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray(points, dtype=float) @ np.array([[c, s], [-s, c]]) + [dx, dy]


def _measurement_transforms(scan, history, now, *, timing_verified, motion_epoch=0,
                            timing_policy=None):
    """원천 배열을 같은 점별 시각으로 보정한다. 기본값은 검증된 시각만 사용한다.

    timing_verified는 기존 운영 관문과 별도 호출 증거다. simulation_acquisition은
    시뮬의 광선 취득 시각이며 실물 시간 검증이 아니다. 추정 시각은 별도 명시
    정책을 받아야 하며 scan.time_basis나 timing_verified를 바꾸지 않는다.
    """
    if not isinstance(scan, MeasurementScan):
        raise MotionUnavailable('measurement_scan_required')
    timing_policy = history.scan_time_policy if timing_policy is None else timing_policy
    if timing_policy not in ('verified_only', 'host_estimated_v1'):
        raise MotionUnavailable('scan_timing_policy_invalid')
    estimated = scan.time_basis == 'unverified_estimate'
    if timing_policy == 'host_estimated_v1' and not estimated:
        raise MotionUnavailable('scan_timing_policy_basis_mismatch')
    if (estimated and timing_policy != 'host_estimated_v1') or (not estimated and not timing_verified):
        raise MotionUnavailable('scan_timing_unverified')
    if scan.source_frame_id != 'laser_frame':
        raise MotionUnavailable('scan_frame_mismatch')
    if type(motion_epoch) is not int or motion_epoch<0:
        raise MotionUnavailable('motion_epoch_invalid')
    if not math.isfinite(now) or scan.acquired_at_s[-1] > now:
        raise MotionUnavailable('scan_future')
    if now-scan.acquired_at_s[0] >= min(.3, history.config.scan_timeout_s):
        raise MotionUnavailable('scan_stale')
    transforms = relative_poses(history, scan.acquired_at_s, now)
    config = history.config
    # 두 독립 세대의 충돌 없는 정수 쌍 부호화. 센서 재시작 또는 운동 이력 리셋
    # 어느 쪽도 이전 관측 캐시와 같은 세대로 섞이지 않게 한다.
    total=scan.source_epoch+motion_epoch
    epoch=total*(total+1)//2+motion_epoch
    return transforms, config, epoch


def measurement_motion_scene(scan, history, now, *, timing_verified, rear_half_angle_deg=30., motion_epoch=0,
                             timing_policy=None):
    transforms, config, epoch = _measurement_transforms(
        scan, history, now, timing_verified=timing_verified, motion_epoch=motion_epoch,
        timing_policy=timing_policy)
    if type(scan) is MeasurementScan:
        from arena_autonomy.native_geometry import native_motion_scene
        return native_motion_scene(scan, transforms, now, config, epoch, rear_half_angle_deg)
    # 공개 입력의 subclass 동작은 원래의 완전 검증 경로로 보존한다.
    return scene_from_measurements(scan.ranges_m, angles_rad=scan.angles_rad,
        acquired_at_s=scan.acquired_at_s, t_ref_s=now, range_min=scan.range_min_m,
        range_max=scan.range_max_m, source_frame_id=scan.source_frame_id,
        source_epoch=epoch, sensor_xy_m=(config.lidar_x_m, config.lidar_y_m),
        sensor_yaw_rad=config.lidar_yaw_rad, acquisition_to_ref_se2=transforms.tolist(),
        rear_half_angle_deg=rear_half_angle_deg, preserve_sampling=True)


def measurement_motion_points(scan, history, now, *, timing_verified, timing_policy=None):
    if type(scan) is not MeasurementScan:
        scene = measurement_motion_scene(scan, history, now, timing_verified=timing_verified,
                                         rear_half_angle_deg=0., timing_policy=timing_policy)
        return (np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2),
                np.asarray([r.source_index for r in scene.rays if r.status == 'return'], dtype=int))
    transforms, config, epoch = _measurement_transforms(scan, history, now, timing_verified=timing_verified,
                                                      timing_policy=timing_policy)
    from arena_autonomy.native_geometry import native_motion_points
    return native_motion_points(scan, transforms, now, config, epoch)


def motion_scene(scan, history, now, *, timing_verified, epoch=0, rear_half_angle_deg=30.,
                 timing_policy=None):
    timing_policy = history.scan_time_policy if timing_policy is None else timing_policy
    if isinstance(scan, MeasurementScan):
        return measurement_motion_scene(scan,history,now,timing_verified=timing_verified,
                                       rear_half_angle_deg=rear_half_angle_deg,motion_epoch=epoch,
                                       timing_policy=timing_policy)
    if timing_policy != 'verified_only':
        raise MotionUnavailable('estimated_policy_requires_native_measurements')
    if not timing_verified:
        raise MotionUnavailable('scan_timing_unverified')
    if scan.header.frame_id != 'laser_frame':
        raise MotionUnavailable('scan_frame_mismatch')
    coverage = audit_front_observation(scan.ranges, scan.angle_min, scan.angle_increment,
        scan.range_min, scan.range_max, history.config.lidar_yaw_rad)
    # 무응답은 장면에 보존하되 잘못된 한 회전 메타데이터는 아예 변환하지 않는다.
    if coverage.reason == 'scan_invalid':
        raise MotionUnavailable('scan_invalid')
    count = len(scan.ranges)
    first = float(scan.header.stamp.sec) + float(scan.header.stamp.nanosec)*1e-9
    step = float(scan.time_increment)
    if (count < 2 or count > MAX_RAYS or not all(math.isfinite(v) for v in (first, step, now))
            or first < 0 or step <= 0):
        raise MotionUnavailable('scan_timing_invalid')
    stamps = first + np.arange(count)*step
    if stamps[-1] > now + 1e-9:
        raise MotionUnavailable('scan_future')
    if now-first >= min(.3, history.config.scan_timeout_s):
        raise MotionUnavailable('scan_stale')
    # timestamp를 잘라 맞추지 않는다. ROS 정수→실수의 미세 차이도 계약과 일치시킨다.
    if stamps[-1] > now:
        raise MotionUnavailable('scan_future')
    transforms = relative_poses(history, stamps, now)
    config = history.config
    try:
        return scene_from_scan(list(map(float, scan.ranges)), angle_min=float(scan.angle_min),
            angle_increment=float(scan.angle_increment), range_min=float(scan.range_min),
            range_max=float(scan.range_max), acquired_at_s=stamps.tolist(), t_ref_s=float(now),
            source_frame_id=scan.header.frame_id, source_epoch=epoch,
            sensor_xy_m=(config.lidar_x_m, config.lidar_y_m), sensor_yaw_rad=config.lidar_yaw_rad,
            acquisition_to_ref_se2=transforms.tolist(), rear_half_angle_deg=rear_half_angle_deg)
    except (ValueError, TypeError) as error:
        raise MotionUnavailable('scene_invalid') from error
