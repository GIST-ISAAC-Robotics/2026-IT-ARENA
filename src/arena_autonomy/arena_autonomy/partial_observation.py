"""좁은 결측의 선택형 운용 정책. 빈 공간 증명/거리 보간이 아니다.

측방 3°/6°/.5 m/s, 정면 1.5°/1 m 이웃/.2 m/s는 ADR 0027 후속
시험값이며 제조사/규정값이 아니다. v1은 ±30° 정면을 허용하지 않는다.
스캔·운동의 무결성 검사는 호출자가 먼저 수행한다. 알려진 근거리 반환의
소실은 래치하며, 시간 만료만으로 장애물을 잊고 진행하지 않는다.
"""
import math

import numpy as np

from arena_autonomy.lidar_observation import audit_front_observation
from arena_autonomy.scene_motion import relative_poses, transport_points


def near_sweep(points, speed, age, steering, previous_steering, wheelbase, length, width, margin):
    """현재/직전 조향의 감속 외곽 근처 점만 고른다. 전체 충돌 검사는 독립 층에 남는다."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    selected = np.zeros(len(points), dtype=bool)
    horizon = speed*(age+.10) + speed*speed/4 + .06
    distances = np.linspace(0., horizon, max(2, math.ceil(horizon/.02)+1))
    for angle in (steering, previous_steering):
        curvature = math.tan(angle)/wheelbase
        yaw = distances*curvature
        rear_x = distances if abs(curvature)<1e-8 else np.sin(yaw)/curvature
        rear_y = np.zeros_like(distances) if abs(curvature)<1e-8 else (1-np.cos(yaw))/curvature
        dx = points[None,:,0] - (rear_x[:,None]-wheelbase/2)
        dy = points[None,:,1] - rear_y[:,None]
        x = np.cos(yaw[:,None])*dx + np.sin(yaw[:,None])*dy-wheelbase/2
        y = -np.sin(yaw[:,None])*dx + np.cos(yaw[:,None])*dy
        selected |= np.any((np.abs(x)<=length/2+margin+.02) & (np.abs(y)<=width/2+margin+.02),axis=0)
    return selected


def short_observed_surface(point, current, source_indices):
    """연속 유효 광선 세 개의 짧은 표면 대응. 결측 사이를 잇지 않는다.

    각 현 <=6 cm, 방향 변화 <=30°, 옛 점의 현 수직 거리 <=1 cm.
    자유 공간/물체 신원 증명이 아니며 이 현을 충돌 검사 점군에 추가하지 않는다.
    """
    current=np.asarray(current,dtype=float).reshape(-1,2)
    ids=np.asarray(source_indices)
    if len(current)!=len(ids) or len(current)<3:
        return None
    vectors=np.diff(current,axis=0)
    sizes=np.linalg.norm(vectors,axis=1)
    continuous=(np.diff(ids)==1) & (sizes>1e-6) & (sizes<=.06)
    cosine=np.sum(vectors[:-1]*vectors[1:],axis=1)/np.maximum(1e-12,sizes[:-1]*sizes[1:])
    triples=np.flatnonzero(continuous[:-1] & continuous[1:] & (cosine>=math.cos(math.radians(30))))
    for index in triples:
        for start in (index,index+1):
            delta=point-current[start]
            fraction=float(delta@vectors[start]/(sizes[start]*sizes[start]))
            if 0<=fraction<=1:
                normal=float(np.linalg.norm(delta-fraction*vectors[start]))
                if normal<=.01:
                    return dict(source_indices=ids[index:index+3].tolist(),
                        current_points=current[index:index+3].tolist(),normal_distance_m=normal,
                        segment_lengths_m=sizes[index:index+2].tolist())
    return None


class PartialObservationPolicy:
    speed_cap_mps = .5

    def __init__(self, allow_narrow_forward=False, surface_translation_enabled=False, lateral_metric_enabled=False):
        if any(type(v) is not bool for v in (allow_narrow_forward,surface_translation_enabled,lateral_metric_enabled)):
            raise ValueError('forward candidate flag must be bool')
        self.allow_narrow_forward = allow_narrow_forward
        self.surface_translation_enabled = surface_translation_enabled
        self.lateral_metric_enabled = lateral_metric_enabled
        self.reset()

    def reset(self):
        self.previous = None
        self.previous_distinct = None
        self.latched = False
        self.latch_reason = None
        self.latch_time = None
        self.latch_evidence = None
        self.clear_sources = 0
        self.last_source = None
        self.surface_cache = None

    def evaluate(self, scan, points, history, now, speed, steering=0., previous_steering=0.,
                 wheelbase=.135, length=.20, width=.15):
        yaw = history.config.lidar_yaw_rad
        audit = audit_front_observation(scan.ranges, scan.angle_min,
            scan.angle_increment, scan.range_min, scan.range_max, yaw)
        source = scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9
        report = dict(allowed=False, reason='partial_invalid', unknown_filled=False,
                      residual_hidden_object_risk=True, speed_cap_mps=self.speed_cap_mps,
                      unknown_samples=audit.unknown_samples)
        def finished(**values):
            return {**report, **values, 'latch_reason':self.latch_reason,
                    'latch_time_s':self.latch_time, 'latch_evidence':self.latch_evidence}
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
                self.latch_reason = self.latch_time = None
                self.latch_evidence = None
            self._remember(points, now, source, fresh)
            return finished(allowed=not self.latched,
                    reason='known_obstacle_latched' if self.latched else 'fully_observed',
                    speed_cap_mps=None)
        self.clear_sources = 0
        values = np.asarray(scan.ranges, dtype=float)
        if speed > 1.2:
            return {**report, 'reason':'partial_speed_outside_trial'}
        # 실제 배열의 범위 밖/무반환과 별개로 각도 메타데이터 이음부 누락은 허용하지 않는다.
        # 유효 상수 배열은 메타데이터 검사 전용이며 거리/점군으로 반환하지 않는다.
        metadata = audit_front_observation(np.full(len(values),scan.range_min),
            scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,yaw)
        if metadata.reason != 'ok':
            return {**report, 'reason':'angular_metadata_hole'}
        step = float(scan.angle_increment)
        angles = (scan.angle_min + np.arange(len(values))*step + yaw + math.pi) % math.tau - math.pi
        order = np.argsort(angles)
        front = order[np.abs(angles[order]) <= math.radians(100)+1e-6]
        # 최소거리 미만의 유한값/음의 무한대는 멀리 있는 벽의 무반환과 다르다.
        # 가까운 표적 또는 잘못된 입력일 수 있어 좁은 측방 공백 예외로 보내지 않는다.
        if np.any(values[front] < scan.range_min):
            return {**report, 'reason':'invalid_near_return'}
        valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
        ids = np.flatnonzero(~valid[front])
        if not len(ids):
            return {**report, 'reason':'angular_metadata_hole'}
        runs = np.split(ids, np.flatnonzero(np.diff(ids)>1)+1)
        widest, total = max(len(run)*step for run in runs), len(ids)*step
        report.update(max_gap_deg=math.degrees(widest), total_gap_deg=math.degrees(total))
        if total > math.radians(6)+1e-7 or len(ids)/len(front) > .05:
            return {**report, 'reason':'wide_or_many_missing'}
        unknown = front[ids]
        metric_gap=widest>math.radians(3)+1e-7
        if metric_gap and not self.lateral_metric_enabled:
            return {**report, 'reason':'wide_or_many_missing'}
        # v4는 3도 초과 측방 공백만 별도 후보로 다룬다. 정면/전체 고장 완화가 아니다.
        if metric_gap:
            radius=max(.6,.16+speed*(now-source+.10)+speed*speed/4)
            # 양쪽 실제 유효 광선 사이 전체 간격을 쓴다(무반환 셀 폭보다 한 step 큼).
            chord=2*radius*math.sin((widest+step)/2)
            report['lateral_metric']=dict(radius_m=radius,missing_chord_m=chord,
                maximum_chord_m=.05,minimum_angle_deg=60.,maximum_measured_or_requested_speed_mps=.6,
                scope='bounded_hidden_sliver_risk_not_measured_free_space')
            if speed>.6 or chord>.05 or np.any(np.abs(angles[unknown])<math.radians(60)):
                return {**report,'reason':'lateral_metric_outside_trial'}
            report['speed_cap_mps']=.2
        central = bool(np.any(np.abs(angles[unknown]) <= math.radians(30)+step/2))
        if central and not self.allow_narrow_forward:
            return {**report, 'reason':'central_missing'}
        # 기존 2 m/s² 제동·입력 나이에 차체/여유를 더한 센서 원점 기준 거리.
        required = max(.6, .16 + speed*(now-source+.10) + speed*speed/4)
        report['history_distance_m'] = required
        # 진행 방향 제동 거리를 옆벽 거리에도 요구하지 않는다. 차체 바로 곁의
        # 반환만 별도 거절하고, 모든 현재 반환의 조향별 제동은 기존 보호층이 담당한다.
        near_boundary = math.hypot(length/2, width/2) + math.hypot(history.config.lidar_x_m, history.config.lidar_y_m) + .03
        report['near_boundary_range_m'] = near_boundary
        boundary_ranges = []
        for run in runs:
            if run[0] == 0 or run[-1] == len(front)-1:
                return {**report, 'reason':'unbounded_missing'}
            boundary_ranges.extend(values[front[[run[0]-1, run[-1]+1]]])
            if min(boundary_ranges) < near_boundary:
                self._latch('near_gap_boundary',now)
                self._remember(points, now, source, fresh)
                return finished(reason='near_gap_boundary')
        # 각 무반환의 실제 취득 방향을 현재 차체로 옮긴다. 거리는 만들지 않는다.
        stamps = source + unknown*float(scan.time_increment)
        poses = relative_poses(history, stamps, now)
        direction = angles[unknown] + poses[:,2]
        central_after = bool(np.any(np.abs((direction+math.pi)%math.tau-math.pi) <= math.radians(30)+step/2))
        if central_after and not self.allow_narrow_forward:
            return {**report, 'reason':'central_missing_after_motion'}
        central = central or central_after
        if metric_gap:
            transported_angles=np.abs((direction+math.pi)%math.tau-math.pi)
            if (np.any(transported_angles<math.radians(60)) or
                    np.any(transported_angles>math.radians(100)) or min(boundary_ranges)<1.):
                return {**report,'reason':'lateral_metric_observation_unconfirmed'}
        if central:
            report['speed_cap_mps'] = .2
            report['forward_neighbor_required_m'] = max(1.,required)
            report['minimum_gap_neighbor_m'] = float(min(boundary_ranges))
        sensor = history.config
        c, s = np.cos(poses[:,2]), np.sin(poses[:,2])
        origins = poses[:,:2] + np.column_stack((c*sensor.lidar_x_m-s*sensor.lidar_y_m,
                                               s*sensor.lidar_x_m+c*sensor.lidar_y_m))
        previous = self.previous if fresh else self.previous_distinct
        if previous is None:
            return {**report, 'reason':'partial_history_warmup'}
        if now-previous[1] > .3:
            self._latch('partial_history_expired',now)
            return finished(reason='partial_history_expired')
        old, reference, previous_source = previous
        report['previous_scan_source_s'] = previous_source
        carried = transport_points(old, history, reference, now)
        if len(carried):
            delta = carried[None,:,:] - origins[:,None,:]
            along = delta[:,:,0]*np.cos(direction[:,None]) + delta[:,:,1]*np.sin(direction[:,None])
            across = np.abs(-delta[:,:,0]*np.sin(direction[:,None]) + delta[:,:,1]*np.cos(direction[:,None]))
            # 이전 근거리 점이 새 공백으로 들어가는지 검사. 2 m/s 상대 이동
            # 여유는 시험 가정이며 모든 상대 속도/형상을 보장하지 않는다.
            spread = .03 + 2*max(0., now-reference) + np.maximum(0.,along)*math.tan(step/2)
            relevant = near_sweep(carried,speed,now-source,steering,previous_steering,
                                  wheelbase,length,width,.03+2*max(0.,now-reference))
            threatening = (along >= 0) & (along <= required) & (across <= spread) & relevant[None,:]
            # 옛 점 위치를 현재 유효 반환이 다시 관측하면 '소실' 가설에서 제외한다.
            # 물체 신원/정지 증명은 아니며 모든 현재 점은 독립 충돌 검사에 남는다.
            # 2 cm는 이 저속 후보의 대응 거리다. C1 거리 오차의 보장값이 아니다.
            current = np.asarray(points, dtype=float).reshape(-1,2)
            current_indices = np.flatnonzero(valid)
            candidates = np.flatnonzero(np.any(threatening,axis=0))
            nearest = {}
            surface_witnesses = []
            observed_count = 0
            for index in candidates:
                distances = np.linalg.norm(current-carried[index],axis=1)
                best = int(np.argmin(distances)) if len(distances) else None
                distance = float(distances[best]) if best is not None else None
                nearest[int(index)] = (best,distance)
                surface = (short_observed_surface(carried[index],current,current_indices)
                           if distance is not None and .02 < distance <= .04 else None)
                if surface is not None and len(surface_witnesses)<3:
                    surface_witnesses.append(dict(previous_point_now_xy=carried[index].tolist(),**surface))
                if distance is not None and (distance <= .02 or surface is not None):
                    threatening[:,index] = False
                    observed_count += 1
            report['reobserved_previous_points'] = observed_count
            report['short_surface_witnesses'] = surface_witnesses
            if self.surface_translation_enabled and np.any(threatening):
                # 공통 이동 후보는 재관측 대응에만 쓴다. 대응하지 않은 옛 점의
                # 위협 위치/현재 점군/모터 속도/경로에는 적용하지 않는다.
                from arena_autonomy.partial_surface_alignment import surface_translation
                key=(previous_source,source)
                cached=self.surface_cache is not None and self.surface_cache[0]==key
                if not cached:
                    fit=surface_translation(carried,current,current_indices)
                    self.surface_cache=(key,now,fit)
                _,fit_time,fit=self.surface_cache
                report['surface_correspondence']={**fit,'cached':cached,'associated_previous_points':0}
                if fit['accepted']:
                    turn=relative_poses(history,[fit_time],now)[0,2]
                    c,s=math.cos(turn),math.sin(turn)
                    offset=np.array([[c,-s],[s,c]])@fit['translation_m']
                    report['surface_correspondence']['translation_at_reference_m']=offset.tolist()
                    for index in np.flatnonzero(np.any(threatening,axis=0)):
                        moved=carried[index]+offset
                        distance=float(np.min(np.linalg.norm(current-moved,axis=1)))
                        if distance<=.02 or (distance<=.04 and short_observed_surface(moved,current,current_indices) is not None):
                            threatening[:,index]=False
                            report['surface_correspondence']['associated_previous_points']+=1
            if np.any(threatening):
                if not self.latched:
                    witnesses = []
                    for ray, index in np.argwhere(threatening)[:8]:
                        best,distance = nearest[int(index)]
                        witnesses.append(dict(source_index=int(unknown[ray]),
                            direction_rad=float(direction[ray]), origin_xy=origins[ray].tolist(),
                            previous_point_now_xy=carried[index].tolist(),
                            nearest_current_xy=current[best].tolist() if best is not None else None,
                            nearest_current_distance_m=distance,
                            along_m=float(along[ray,index]), across_m=float(across[ray,index]),
                            allowed_spread_m=float(spread[ray,index])))
                    self.latch_evidence = dict(scope='sensor_points_only_not_object_identity',
                        pairs=int(np.count_nonzero(threatening)), witnesses=witnesses,
                        previous_reference_s=reference, previous_source_first_s=previous_source,
                        reference_s=now, source_first_s=source,
                        sweep_speed_mps=speed, steering_rad=steering,
                        previous_steering_rad=previous_steering,
                        # 최초 실패의 센서 좌표만 보존한다. 정답 자세를 제어 입력에 넣지 않는다.
                        # 이 자료로 후속 대응 가설을 저장 입력에서 비교하며 반복 주행을 줄인다.
                        correspondence_context=dict(
                            previous_points_at_reference_xy=old.tolist(),
                            previous_points_now_xy=carried.tolist(),
                            current_return_points_xy=current.tolist(),
                            current_return_source_indices=current_indices.tolist(),
                            estimated_reference_to_now_se2=relative_poses(history,[reference],now)[0].tolist()))
                self._latch('previous_point_in_missing_sweep',now)
        self._remember(points, now, source, fresh)
        # 좁은 정면 예외라도 장애물 기억을 먼저 계산한다. 진행을 거절한 동안
        # 살아 있는 관측까지 갱신하지 않아 인위적으로 기억을 만료시키지 않는다.
        if central and not self.latched:
            if total > math.radians(1.5)+1e-7:
                return finished(reason='forward_gap_too_wide')
            if min(boundary_ranges) < max(1.,required):
                return finished(reason='forward_gap_near_boundary')
        return finished(allowed=not self.latched,
                reason='known_obstacle_latched' if self.latched else
                       'bounded_lateral_metric_missing' if metric_gap else
                       'bounded_forward_missing' if central else 'bounded_side_missing')

    def _latch(self,reason,now):
        if not self.latched:
            self.latch_reason,self.latch_time = reason,now
        self.latched = True

    def _remember(self, points, now, source, fresh):
        if fresh:
            points = np.asarray(points, dtype=float).reshape(-1,2)
            points = points[np.all(np.isfinite(points),axis=1)]
            self.previous_distinct = self.previous
            self.previous = (points[np.linalg.norm(points,axis=1) <= 3.].copy(), now, source)
        self.last_source = source
