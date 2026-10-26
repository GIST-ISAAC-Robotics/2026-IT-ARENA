"""명시적으로 선택하는 M3 시뮬레이션 시험용 순수 제어기.

G1이 확보한 경계에서만 두 바퀴 잔디/관측 측방 후보를 검토한다. 센서 무효·역행·전방
미관측·경계 부재 정지는 덮어쓰지 않는다. 숨은 차량의 미래 진입은 미해결이며 실차용이 아니다.
현재 ROS/launch에는 연결하지 않는다. 선택 경로와 발행 조향의 이산 검사는 연속 안전 증명이 아니다.
"""
import math
from dataclasses import replace

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow, pursue_path
from arena_autonomy.local_path import swept_limit, braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.lateral_intent import LateralIntentConfig, committed_progress_m, wheel_contact_corners, GEOMETRY_EPS_M
from arena_autonomy.observed_lateral_trial import ObservedLateralTrial, path_observation_queries
from arena_autonomy.observation_support import depth_support


def command_arc(steering, progress, cfg):
    """후륜축에서 적분한 정곡률 차체 자세. 지상 정답/속도 명령을 사용하지 않는다."""
    travel = np.linspace(0., progress, max(2, int(math.ceil(progress/.01))+1))
    k = math.tan(steering)/cfg.wheelbase_m
    yaw = k*travel
    if abs(k) < 1e-9:
        rx, ry = travel-cfg.wheelbase_m/2, np.zeros_like(travel)
    else:
        rx = np.sin(yaw)/k-cfg.wheelbase_m/2
        ry = (1-np.cos(yaw))/k
    return np.column_stack((rx+cfg.wheelbase_m/2*np.cos(yaw),
                            ry+cfg.wheelbase_m/2*np.sin(yaw), yaw))


def command_observation(scene, speed, steering, previous, cfg):
    """후보 경로와 다른 이번 발행각/이전각의 현재 관측도 별도로 검사한다."""
    distance = committed_progress_m(speed, scene.t_ref_s-scene.source_first_s, cfg)
    points, sectors = [], 0
    for angle in set((steering, previous)):
        poses = command_arc(angle, distance, cfg)
        query, count = path_observation_queries(scene, poses, cfg)
        wheel = wheel_contact_corners(poses, np.full(len(poses), angle),
                                     replace(cfg, contact_patch_length_m=2*cfg.wheel_radius_m)).reshape(-1, 2)
        wheel = wheel[(abs(wheel[:, 0]) > cfg.length_m/2+GEOMETRY_EPS_M) |
                      (abs(wheel[:, 1]) > cfg.width_m/2+GEOMETRY_EPS_M)]
        points.extend((query, wheel))
        sectors += count
    query = np.unique(np.vstack(points), axis=0)
    pad = np.array([.025+cfg.budget.range_bias_m+.5*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])
    supported = depth_support(scene, query, pad)
    return dict(supported=bool(np.all(supported)), queries=len(query),
                unsupported=int(np.sum(~supported)), unknown_intersections=sectors,
                distance_m=distance, continuous_sweep_certified=False)


class M3ObservedFollow:
    def __init__(self, config=None, *, allow_simulation_trial=False):
        if allow_simulation_trial is not True:
            raise ValueError('M3 requires explicit simulation trial selection')
        self.config = config or FollowConfig(occupancy_timing_mode='per_ray_candidate')
        c = self.config
        if c.max_speed_mps > 1. or c.occupancy_timing_mode != 'per_ray_candidate' or c.recent_observation_enabled:
            raise ValueError('M3 trial requires <=1 m/s, per-ray occupancy, no recent-observation mode')
        # 새 측방 후보만 저속으로 비교한다. 원래 단독/추종 속도는 바꾸지 않는다.
        self.trial_speed_cap_mps = min(.35, c.max_speed_mps)
        self.lateral_config = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass',
            wheelbase_m=c.wheelbase_m, length_m=c.length_m, width_m=c.width_m,
            max_steering_rad=c.max_steering_rad)
        if not math.isclose(c.wall_offset_m, self.lateral_config.wall_offset_m, abs_tol=1e-12):
            raise ValueError('M3 requires the registered wall-to-road geometry')
        self.base = LocalFollow(c)
        self.lateral = ObservedLateralTrial(self.lateral_config)
        self.reset()

    def reset(self):
        self.base.reset()
        self.lateral.reset()
        self.epoch = self.side = self.frame = self.last_ref = self.last_source = None
        self.held = True
        self.release_count = 0

    def _hold(self, reason, detail, steering=0.):
        self.held = True
        self.release_count = 0
        return 0., float(steering), {**detail, 'reason': reason, 'behavior': 'HOLD'}

    def command(self, scene, history, speed, side, previous_steering=0., control_dt=.02):
        invalid = (side not in ('left', 'right') or not all(math.isfinite(v) for v in
                   (speed, previous_steering, control_dt)) or not 0 < control_dt <= .2)
        changed = (self.epoch != scene.source_epoch or self.side != side or self.frame != scene.source_frame_id or
                   (self.last_ref is not None and scene.t_ref_s < self.last_ref) or
                   (self.last_source is not None and scene.source_first_s is not None and scene.source_first_s < self.last_source))
        if invalid or changed:
            self.reset()
        self.epoch, self.side, self.frame, self.last_ref = scene.source_epoch, side, scene.source_frame_id, scene.t_ref_s
        fresh = self.last_source is None or (scene.source_first_s is not None and scene.source_first_s > self.last_source)
        self.last_source = scene.source_first_s
        baseline = self.base.command(scene, history, speed, side, previous_steering, control_dt)
        detail = {**baseline[2], 'mode': 'm3_observed_trial', 'simulation_trial_only': True,
                  'hidden_future_risk_unresolved': True, 'm3_selected': False,
                  'baseline_output': [baseline[0], baseline[1]], 'baseline_reason': baseline[2]['reason']}
        # 경로/입력 게이트까지 통과한 간격 HOLD만 새 경로로 다시 평가할 수 있다.
        if baseline[2]['reason'] not in ('local_follow', 'follow_gap_hold', 'follow_release_wait', 'local_follow_corridor_candidate'):
            self.held, self.release_count = True, 0
            return baseline[0], baseline[1], detail
        was_active = self.lateral.active
        # 정지 상태에서 양수 목표를 낼 때도 그 목표까지의 관측 외곽을 먼저 확인한다.
        envelope_speed = max(speed, self.trial_speed_cap_mps)
        trial = self.lateral.step(scene, baseline[2].get('path_points'), envelope_speed)
        detail['m3_lateral'] = {k: v for k, v in trial.items() if k != 'path'}
        if trial['mode'] == 'BASELINE':
            self.held = baseline[2]['behavior'] == 'HOLD'
            self.release_count = self.base.release_count
            return baseline[0], baseline[1], detail
        if not was_active:
            self.held = baseline[2]['behavior'] == 'HOLD'
            self.release_count = 0
        # 실패 시 새 회피각을 만들어내지 않는다. 0 목표/이전 조향 유지도 안전 정지 증명은 아니다.
        if trial['path'] is None or trial['selected'] is None:
            return self._hold('m3_candidate_unavailable', detail, previous_steering)
        cfg = self.config
        age = scene.t_ref_s-scene.source_first_s
        try:
            cap, desired, remaining = pursue_path(trial['path'], speed, cfg,
                max(age, baseline[2].get('boundary_age_s') or 0.))
        except MotionUnavailable:
            return self._hold('m3_path_too_short', detail, previous_steering)
        steering = float(np.clip(desired, previous_steering-3*control_dt, previous_steering+3*control_dt))
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        padding = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
        clearance = min(swept_limit(points, a, cfg.wheelbase_m, horizon=2.5,
                                   margin=padding, length=cfg.length_m, width=cfg.width_m)
                        for a in (steering, previous_steering))
        support = command_observation(scene, envelope_speed, steering, previous_steering, self.lateral_config)
        detail.update(m3_selected=True, m3_command_observation=support, path_points=np.asarray(trial['path']).tolist(),
            occupied_points=len(points), unfiltered_occupancy=True, swept_clearance_m=clearance,
            observed_path_ahead_m=remaining, occupancy_padding_min_m=float(np.min(padding)),
            occupancy_padding_max_m=float(np.max(padding)))
        if not support['supported']:
            return self._hold('m3_command_unobserved', detail, previous_steering)
        available = max(0., clearance-cfg.standstill_gap_m)
        target = min(self.trial_speed_cap_mps, cap, braking_speed(available, age), available/cfg.headway_s)
        detail['target_speed_before_hold_mps'] = target
        if clearance <= cfg.hold_clearance_m:
            return self._hold('m3_gap_hold', detail, previous_steering)
        if self.held:
            if clearance < cfg.resume_clearance_m:
                self.release_count = 0
            elif fresh:
                self.release_count += 1
            if self.release_count < cfg.confirmations:
                return 0., steering, {**detail, 'reason': 'm3_release_wait', 'behavior': 'HOLD'}
            self.held = False
        return float(target), steering, {**detail, 'reason': 'm3_observed_trial',
            'behavior': 'FOLLOW' if target < cap-1e-6 else 'CRUISE'}
