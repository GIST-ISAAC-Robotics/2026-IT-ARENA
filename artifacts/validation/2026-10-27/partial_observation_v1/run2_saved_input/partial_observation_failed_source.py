"""좁은 측방 결측의 선택형 운용 정책. 빈 공간 증명/거리 보간이 아니다.

3°/6°/30°와 0.5 m/s는 ADR 0027 후속 시험값이며 제조사/규정값이 아니다.
스캔·운동의 무결성 검사는 호출자가 먼저 수행한다. 알려진 근거리 반환의
소실은 래치하며, 시간 만료만으로 장애물을 잊고 진행하지 않는다.
"""
import math

import numpy as np

from arena_autonomy.lidar_observation import audit_front_observation
from arena_autonomy.scene_motion import relative_poses, transport_points


class PartialObservationPolicy:
    speed_cap_mps = .5

    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.latched = False
        self.clear_sources = 0
        self.last_source = None

    def evaluate(self, scan, points, history, now, speed):
        yaw = history.config.lidar_yaw_rad
        audit = audit_front_observation(scan.ranges, scan.angle_min,
            scan.angle_increment, scan.range_min, scan.range_max, yaw)
        source = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        report = dict(allowed=False, reason='partial_invalid', unknown_filled=False,
                      residual_hidden_object_risk=True, speed_cap_mps=self.speed_cap_mps,
                      unknown_samples=audit.unknown_samples)
        if (not math.isfinite(now) or not math.isfinite(speed) or speed < 0 or
                audit.reason == 'scan_invalid' or not 0 <= now-source < .3):
            return report
        if ((self.last_source is not None and source < self.last_source) or
                (self.previous is not None and now < self.previous[1])):
            self.reset()
            return {**report, 'reason':'partial_time_reset'}
        fresh = source != self.last_source
        if audit.reason == 'ok':
            if fresh:
                self.clear_sources += 1
            if self.clear_sources >= 2:
                self.latched = False
            self._remember(points, now, source, fresh)
            return {**report, 'allowed':not self.latched,
                    'reason':'known_obstacle_latched' if self.latched else 'fully_observed',
                    'speed_cap_mps':None}
        self.clear_sources = 0
        values = np.asarray(scan.ranges, dtype=float)
        step = float(scan.angle_increment)
        angles = (scan.angle_min + np.arange(len(values))*step + yaw + math.pi) % math.tau - math.pi
        order = np.argsort(angles)
        front = order[np.abs(angles[order]) <= math.radians(100)+1e-6]
        valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
        ids = np.flatnonzero(~valid[front])
        if not len(ids):
            return {**report, 'reason':'angular_metadata_hole'}
        runs = np.split(ids, np.flatnonzero(np.diff(ids)>1)+1)
        widest, total = max(len(run)*step for run in runs), len(ids)*step
        report.update(max_gap_deg=math.degrees(widest), total_gap_deg=math.degrees(total))
        if (widest > math.radians(3)+1e-7 or total > math.radians(6)+1e-7 or
                len(ids)/len(front) > .05):
            return {**report, 'reason':'wide_or_many_missing'}
        unknown = front[ids]
        if np.any(np.abs(angles[unknown]) <= math.radians(30)+step/2):
            return {**report, 'reason':'central_missing'}
        # 기존 2 m/s² 제동·입력 나이에 차체/여유를 더한 센서 원점 기준 거리.
        required = max(.6, .16 + speed*(now-source+.10) + speed*speed/4)
        report['required_neighbor_range_m'] = required
        for run in runs:
            if run[0] == 0 or run[-1] == len(front)-1:
                return {**report, 'reason':'unbounded_missing'}
            if np.min(values[front[[run[0]-1, run[-1]+1]]]) < required:
                self.latched = True
                self._remember(points, now, source, fresh)
                return {**report, 'reason':'near_gap_boundary'}
        # 각 무반환의 실제 취득 방향을 현재 차체로 옮긴다. 거리는 만들지 않는다.
        stamps = source + unknown*float(scan.time_increment)
        poses = relative_poses(history, stamps, now)
        direction = angles[unknown] + poses[:,2]
        if np.any(np.abs((direction+math.pi)%math.tau-math.pi) <= math.radians(30)+step/2):
            return {**report, 'reason':'central_missing_after_motion'}
        sensor = history.config
        c, s = np.cos(poses[:,2]), np.sin(poses[:,2])
        origins = poses[:,:2] + np.column_stack((c*sensor.lidar_x_m-s*sensor.lidar_y_m,
                                               s*sensor.lidar_x_m+c*sensor.lidar_y_m))
        if self.previous is None:
            return {**report, 'reason':'partial_history_warmup'}
        if now-self.previous[1] > .3:
            self.latched = True
            return {**report, 'reason':'partial_history_expired'}
        old, reference, _ = self.previous
        carried = transport_points(old, history, reference, now)
        if len(carried):
            delta = carried[None,:,:] - origins[:,None,:]
            along = delta[:,:,0]*np.cos(direction[:,None]) + delta[:,:,1]*np.sin(direction[:,None])
            across = np.abs(-delta[:,:,0]*np.sin(direction[:,None]) + delta[:,:,1]*np.cos(direction[:,None]))
            # 이전 근거리 점이 새 공백으로 들어가는지 검사. 2 m/s 상대 이동
            # 여유는 시험 가정이며 모든 상대 속도/형상을 보장하지 않는다.
            spread = .03 + 2*max(0., now-reference) + np.maximum(0.,along)*math.tan(step/2)
            if np.any((along >= 0) & (along <= required) & (across <= spread)):
                self.latched = True
        self._remember(points, now, source, fresh)
        return {**report, 'allowed':not self.latched,
                'reason':'known_obstacle_latched' if self.latched else 'bounded_side_missing'}

    def _remember(self, points, now, source, fresh):
        if fresh:
            points = np.asarray(points, dtype=float).reshape(-1,2)
            points = points[np.all(np.isfinite(points),axis=1)]
            self.previous = (points[np.linalg.norm(points,axis=1) <= 3.].copy(), now, source)
        self.last_source = source
