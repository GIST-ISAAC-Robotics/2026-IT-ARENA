"""취득 시각을 보존하는 LocalScene ↔ 엔코더/자이로 평면 운동 어댑터.

Gazebo 자세·명령 속도를 사용하지 않는다. 수평 LiDAR, 검증된 순차 취득 시각,
기존 MotionHistory의 ZOH/공백/외삽 한도를 전제로 한다.
"""
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_scene import MAX_RAYS, scene_from_scan
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


def motion_scene(scan, history, now, *, timing_verified, epoch=0, rear_half_angle_deg=30.):
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
