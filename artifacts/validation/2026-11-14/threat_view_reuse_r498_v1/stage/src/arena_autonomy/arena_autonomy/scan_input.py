"""균일 LaserScan과 원천 측정 배열의 공통 접근. 원천 배열은 재표본화하지 않는다."""
import math
import numpy as np

from arena_autonomy.measurement_scan import MeasurementScan, sampling_gaps
from arena_autonomy.lidar_observation import audit_front_observation, FrontObservationAudit
from arena_autonomy.lidar_motion import MotionUnavailable


def scan_first_stamp(scan):
    if isinstance(scan, MeasurementScan):
        return scan.acquired_at_s[0]
    return scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9


def scan_angles(scan):
    if isinstance(scan, MeasurementScan):
        return np.asarray(scan.angles_rad)
    return scan.angle_min + np.arange(len(scan.ranges))*scan.angle_increment


def scan_times(scan):
    if isinstance(scan, MeasurementScan):
        return np.asarray(scan.acquired_at_s)
    first=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
    return first+np.arange(len(scan.ranges))*float(scan.time_increment)


def scan_cell_widths(scan):
    if not isinstance(scan, MeasurementScan):
        return np.full(len(scan.ranges),float(scan.angle_increment))
    gaps=np.abs(sampling_gaps(scan.angles_rad))
    if not len(gaps): raise ValueError('no sampling gaps')
    return (np.r_[gaps[0],gaps]+np.r_[gaps,gaps[-1]])/2


def scan_max_gap(scan):
    if isinstance(scan,MeasurementScan):
        return max(abs(v) for v in sampling_gaps(scan.angles_rad))
    return float(scan.angle_increment)


def audit_scan_front(scan, yaw=0., *, metadata_only=False):
    values=np.full(len(scan.ranges),scan.range_min) if metadata_only else scan.ranges
    if not isinstance(scan,MeasurementScan):
        return audit_front_observation(values,scan.angle_min,scan.angle_increment,
                                       scan.range_min,scan.range_max,yaw)
    invalid=FrontObservationAudit('scan_invalid')
    gaps=np.asarray(sampling_gaps(scan.angles_rad))
    if (not math.isfinite(yaw) or not len(gaps)
            or not (np.all((gaps>0)&(gaps<=math.radians(2)))
                    or np.all((gaps<0)&(gaps>=-math.radians(2))))
            or not 5.5<=abs(float(sum(gaps)))<=7.):
        return invalid
    angles=(scan_angles(scan)+math.remainder(yaw,math.tau)+math.pi)%math.tau-math.pi
    front=abs(angles)<=math.radians(100)+1e-6
    values=np.asarray(values);valid=np.isfinite(values)&(values>=scan.range_min)&(values<=scan.range_max)
    missing=int(np.count_nonzero(front&~valid));count=int(np.count_nonzero(front))
    ordered=np.sort(angles);gap=np.diff(np.r_[ordered,ordered[0]+math.tau])
    mid=(ordered+gap/2+math.pi)%math.tau-math.pi
    # 원천 내부는 기존 최대 각 간격2° 계약을 만족해야 한다. 회전 경계의 빈 각도도
    # 같은 최대 간격으로 확인하며, 부족한 방향을 마지막 점 복제로 메우지 않는다.
    hole=np.any((gap>math.radians(2)+1e-7)&(abs(mid)<math.radians(100)+gap/2-1e-7))
    return FrontObservationAudit('front_unobserved' if hole or missing or not count else 'ok',count,missing)


def project_scan(scan, history, now, mode='both', timing_verified=False, *, timing_policy=None):
    timing_policy = history.scan_time_policy if timing_policy is None else timing_policy
    if not isinstance(scan,MeasurementScan):
        if timing_policy != 'verified_only':
            raise MotionUnavailable('estimated_policy_requires_native_measurements')
        return history.project(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,
            scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9,float(scan.time_increment),now,mode,timing_verified)
    if mode!='both':
        raise MotionUnavailable('native_scan_requires_both_compensation')
    from arena_autonomy.scene_motion import measurement_motion_points
    points,ids=measurement_motion_points(scan,history,now,timing_verified=timing_verified,
                                       timing_policy=timing_policy)
    first,last=scan.acquired_at_s[0],scan.acquired_at_s[-1]
    return points,ids,dict(mode=mode,scan_stamp_s=first,last_ray_stamp_s=last,
        reference_stamp_s=now,oldest_observation_age_s=now-first,newest_observation_age_s=now-last,
        valid_points=len(ids),coordinate_time_assumption=False,input_representation='native_measurements',
        time_basis=scan.time_basis,
        **({'timing_policy':timing_policy,'hardware_timing_verified':False}
           if timing_policy!='verified_only' else {}))
