"""회랑 비교 관측기. 기본 경계 기억/LocalFollow 상태를 변경하지 않는다.

evaluate는 명령 tuple 대신 비교용 evidence dict를 반환한다. 이 모듈을 호출하는
기본 주행 경로는 비활성이다. 명시 선택형 LocalFollow가 제한된 HOLD에서만 제안을
채택할 수 있으며 신호/고장 허가와 독립 보호층은 별도다. positive 자체는 구동 허가가 아니다.
"""
import math

import numpy as np

from arena_autonomy.corner_corridor import corridor_evidence
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import path_agrees, pursue_path
from arena_autonomy.local_path import braking_speed, swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import transport_points


def current_proposal(scene, speed, side, previous_steering, dt, config):
    base = dict(geometry_supported=False, point_occupancy_supported=False, proposed_speed_mps=0.,
                reason='invalid_motion_state', driving_authorized=False, runtime_connected=False,
                memory_refresh_authorized=False, free_area_certified=False)
    if (side not in ('left','right') or not all(math.isfinite(v) for v in (speed,previous_steering,dt)) or
            not 0 < dt <= .2 or abs(previous_steering) > config.max_steering_rad):
        return base
    if scene.source_frame_id != 'laser_frame':
        return {**base, 'reason':'source_frame_mismatch'}
    if scene.source_first_s is None or not 0 <= scene.t_ref_s-scene.source_first_s < config.memory_ttl_s:
        return {**base, 'reason':'scene_stale'}
    front = [r for r in scene.rays if abs(math.atan2(r.direction_xy[1],r.direction_xy[0])) <= math.radians(100)]
    if not front or any(r.status != 'return' for r in front):
        from arena_autonomy.observation_support import side_missing_candidate
        if not (config.recent_observation_enabled and side_missing_candidate(scene)):
            return {**base, 'reason':'front_unobserved'}
        # 선택형 LocalFollow의 관측 확인 전까지는 여전히 경로/점유 제안일 뿐이다.
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1,2)
    geometry = corridor_evidence(points, side, config)
    base.update(geometry_supported=geometry['supported'], geometry=geometry,
                occupied_points=len(points), all_return_points_retained=True)
    if not geometry['supported']:
        return {**base, 'reason':geometry['reason']}
    age = scene.t_ref_s-scene.source_first_s
    try:
        cap, desired, ahead = pursue_path(geometry['path'], speed, config, age)
    except MotionUnavailable as error:
        return {**base, 'reason':'pursuit_unavailable', 'error':str(error)}
    steering = float(np.clip(desired,previous_steering-3*dt,previous_steering+3*dt))
    padding = (per_ray_padding(scene,.025+config.measurement_padding_m,config.object_motion_bound_mps)
               if config.occupancy_timing_mode == 'per_ray_candidate' else
               .025+config.measurement_padding_m+config.object_motion_bound_mps*age)
    clearances = [swept_limit(points,angle,config.wheelbase_m,horizon=2.5,margin=padding,
                             length=config.length_m,width=config.width_m) for angle in (steering,previous_steering)]
    clearance = min(clearances)
    available = max(0.,clearance-config.standstill_gap_m)
    target = min(cap,braking_speed(available,age),available/config.headway_s)
    base.update(steering_rad=steering, swept_clearance_m=clearance, observed_path_ahead_m=ahead,
                point_padding_min_m=float(np.min(padding)), point_padding_max_m=float(np.max(padding)))
    if speed < 0:
        return {**base, 'reason':'reverse_motion'}
    if clearance < config.resume_clearance_m:
        return {**base, 'reason':'existing_occupancy_margin_hold'}
    return {**base, 'reason':'geometry_and_point_occupancy_only', 'point_occupancy_supported':target > 0,
            'proposed_speed_mps':float(target)}


class CorridorObserver:
    """현재 기하 필수. 과거 path는 확인 전용이며 과거 path만의 출력은 불가하다."""
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.key = self.last_ref = self.last_source = None
        self.path = self.path_ref = self.path_source = None
        self.confirmations = self.release_count = 0

    def clear_evidence(self):
        self.path = self.path_ref = self.path_source = None
        self.confirmations = self.release_count = 0

    def evaluate(self, scene, history, speed, side, previous_steering=0., dt=.02):
        now, source = scene.t_ref_s, scene.source_first_s
        key = (scene.source_frame_id,scene.source_epoch,side)
        reset_cause = None
        if self.key is not None and key != self.key:
            reset_cause = 'frame_epoch_or_side_changed'
            self.reset()
        elif ((self.last_ref is not None and now < self.last_ref) or
              (self.last_source is not None and source is not None and source < self.last_source)):
            self.reset()
            self.key, self.last_ref, self.last_source = key, now, source
            return dict(positive=False, reason='reference_or_source_regressed', fresh=False,
                        confirmations=0, release_count=0, proposed_speed_mps=0.,
                        driving_authorized=False, runtime_connected=False)
        fresh = source is not None and (self.last_source is None or source > self.last_source)
        self.key, self.last_ref, self.last_source = key, now, source
        proposal = current_proposal(scene,speed,side,previous_steering,dt,self.config)
        result = dict(positive=False, reason=proposal['reason'], fresh=fresh, proposed_speed_mps=0.,
                      confirmations=0, release_count=0, geometry_supported=proposal['geometry_supported'],
                      current_point_occupancy_supported=proposal['point_occupancy_supported'],
                      source_first_s=source, reference_s=now, reset_cause=reset_cause,
                      swept_clearance_m=proposal.get('swept_clearance_m'),
                      driving_authorized=False, runtime_connected=False, memory_refresh_authorized=False,
                      wall_identity_certified=False, old_path_only_allowed=False)
        if not proposal['geometry_supported'] or proposal['reason'] in ('reverse_motion','pursuit_unavailable'):
            self.clear_evidence()
            return result
        path = np.asarray(proposal['geometry']['path'])
        compatible, displacement, transport_error = False,None,None
        prior_age = None if self.path_source is None else now-self.path_source
        if self.path is not None and 0 <= prior_age < self.config.memory_ttl_s:
            try:
                transported = transport_points(self.path,history,self.path_ref,now)
                displacement = float(np.max(np.linalg.norm(transported-self.path,axis=1)))
                compatible = (displacement <= self.config.memory_motion_limit_m and
                              path_agrees(path,transported,self.config.boundary_agreement_m))
            except (ValueError,MotionUnavailable) as error:
                transport_error = str(error)
        result.update(previous_source_age_s=prior_age, previous_path_agrees=bool(compatible),
                      transport_displacement_m=displacement, transport_error=transport_error)
        if not fresh and not compatible:
            self.clear_evidence()
            return {**result, 'reason':'reused_scan_path_unconfirmed'}
        if fresh:
            self.confirmations = self.confirmations+1 if compatible else 1
            if not compatible:
                self.release_count = 0
            self.path, self.path_ref, self.path_source = path.copy(),now,source
            if self.confirmations >= self.config.confirmations and proposal['point_occupancy_supported']:
                self.release_count += 1
        if not proposal['point_occupancy_supported']:
            self.release_count = 0
        result.update(confirmations=self.confirmations,release_count=self.release_count)
        if self.confirmations < self.config.confirmations:
            result['reason'] = 'distinct_geometry_confirmation_wait'
        elif not proposal['point_occupancy_supported']:
            result['reason'] = proposal['reason']
        elif self.release_count < self.config.confirmations:
            result['reason'] = 'distinct_motion_release_wait'
        else:
            result.update(positive=True,reason='confirmed_shadow_candidate_only',path=path.tolist(),
                          proposed_speed_mps=proposal['proposed_speed_mps'],steering_rad=proposal['steering_rad'])
        return result


def compare_boundary_hold(baseline, proposal):
    """저장 기본 출력과의 대조만. 보호/점유/고장 HOLD는 절대로 대체하지 않는다.

    반환값은 ROS 명령이 아니다. 우선순위와 전환 시점의 비교 준비이며 실제 주행,
    동적 handoff 안정성이나 독립 보호층 연결 완료를 의미하지 않는다.
    """
    speed, steering, detail = baseline
    selected = bool(speed == 0 and detail.get('reason') == 'follow_boundary_unconfirmed' and proposal.get('positive'))
    return dict(selected=selected, reason='corridor_shadow_for_boundary_hold' if selected else 'baseline_retained',
                proposed_speed_mps=proposal['proposed_speed_mps'] if selected else speed,
                proposed_steering_rad=proposal['steering_rad'] if selected else steering,
                baseline_reason=detail.get('reason'),
                driving_authorized=False,runtime_connected=False,boundary_memory_modified=False,
                transition_dynamics_verified=False)
