"""운영 미연결 R0+ 비교 후보: 현재 양측 R0 경로 일치 + G1 간격 법칙.

경계 이력/점 제거/조각 연결/회랑 폴백은 없다. 양쪽을 관측하지 못하면
진행을 포기한다. 경로 일치는 물체의 정체를 증명하지 않으므로 여러 차가
그럴듯한 벽을 만드는 장면까지 안전하다고 주장하지 않는다.
"""
import math

import numpy as np

from arena_autonomy.local_follow import FollowConfig, path_agrees
from arena_autonomy.local_path import local_path, pursuit_command, swept_limit, braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding


class R0Plus:
    def __init__(self, config=None):
        self.config = config or FollowConfig(occupancy_timing_mode='per_ray_candidate')
        if (self.config.max_speed_mps > 1. or self.config.boundary_evidence_mode != 'connected'
                or self.config.occupancy_timing_mode != 'per_ray_candidate'
                or self.config.corridor_fallback_enabled or self.config.recent_observation_enabled):
            raise ValueError('R0+ screen requires <=1 m/s, per-ray occupancy and no G1 boundary options')
        self.reset()

    def reset(self):
        self.epoch = self.side = self.frame = self.last_ref = self.last_source = None
        self.release_count = 0
        self.held = True

    def _hold(self, reason, detail):
        self.held = True
        self.release_count = 0
        return 0., 0., {**detail, 'reason': reason, 'behavior': 'HOLD'}

    def command(self, scene, history, speed, side, previous_steering=0., control_dt=.02):
        # history는 G1과 같은 호출 인터페이스를 위해 받는다. 경로 이력을 이송하지 않는다.
        cfg, now = self.config, scene.t_ref_s
        detail = {'mode': 'r0_plus_candidate', 'reference_stamp_s': now,
                  'source_first_s': scene.source_first_s, 'source_last_s': scene.source_last_s,
                  'source_epoch': scene.source_epoch, 'preferred_wall': side,
                  'relative_velocity_credit': False, 'passing_enabled': False,
                  'boundary_from_memory': False, 'measured_speed_mps': speed}
        if (side not in ('left', 'right') or not all(math.isfinite(v) for v in
                (speed, previous_steering, control_dt)) or not 0 < control_dt <= .2):
            self.reset()
            return self._hold('r0plus_invalid_state', detail)
        if (self.epoch != scene.source_epoch or self.side != side or self.frame != scene.source_frame_id
                or (self.last_ref is not None and now < self.last_ref)
                or (self.last_source is not None and scene.source_first_s is not None
                    and scene.source_first_s < self.last_source)):
            self.reset()
        self.epoch, self.side, self.frame = scene.source_epoch, side, scene.source_frame_id
        self.last_ref = now
        if scene.source_first_s is None or now-scene.source_first_s >= .3:
            self.reset()
            return self._hold('r0plus_scene_stale', detail)
        front = [r for r in scene.rays if abs(math.atan2(r.direction_xy[1], r.direction_xy[0])) <= math.radians(100)]
        if not front or any(r.status != 'return' for r in front):
            self.reset()
            return self._hold('r0plus_front_unobserved', detail)
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        if len(points) < 30:
            self.reset()
            return self._hold('r0plus_scan_invalid', detail)
        fresh = self.last_source is None or scene.source_first_s > self.last_source
        self.last_source = scene.source_first_s
        age = now-scene.source_first_s
        cap, desired, r0 = pursuit_command(points, side, speed, cfg.wheelbase_m,
            cfg.max_steering_rad, cfg.max_speed_mps, cfg.lateral_acceleration_mps2,
            cfg.wall_offset_m, age)
        detail.update(r0_path_reason=r0['reason'], r0_speed_cap_mps=cap)
        if cap <= 0:
            return self._hold('r0plus_path_missing', detail)
        path = np.asarray(r0['path_points'], dtype=float)
        opposite_side = 'right' if r0['path_wall'] == 'left' else 'left'
        opposite, other = local_path(points, opposite_side, cfg.wall_offset_m)
        detail.update(path_wall=r0['path_wall'], wall_fallback=r0['wall_fallback'],
                      path_points=path.tolist(), opposite_reason=other['reason'])
        if opposite is None or not path_agrees(path, opposite, cfg.boundary_agreement_m):
            return self._hold('r0plus_bilateral_unconfirmed', detail)
        steering = float(np.clip(desired, previous_steering-3*control_dt, previous_steering+3*control_dt))
        margin = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
        clearance = min(swept_limit(points, a, cfg.wheelbase_m, horizon=2.5,
            margin=margin, length=cfg.length_m, width=cfg.width_m) for a in (steering, previous_steering))
        available = max(0., clearance-cfg.standstill_gap_m)
        target = min(cap, braking_speed(available, age), available/cfg.headway_s)
        detail.update(bilateral_confirmed=True, unfiltered_occupancy=True, occupied_points=len(points),
            occupancy_timing_mode='per_ray_candidate', occupancy_padding_min_m=float(np.min(margin)),
            occupancy_padding_max_m=float(np.max(margin)), swept_clearance_m=clearance,
            target_speed_before_hold_mps=target, observed_path_ahead_m=r0['observed_path_ahead_m'])
        if speed < 0:
            return self._hold('r0plus_reverse_motion', detail)
        if clearance <= cfg.hold_clearance_m:
            return self._hold('r0plus_gap_hold', detail)
        if self.held:
            if clearance < cfg.resume_clearance_m:
                self.release_count = 0
            elif fresh:
                self.release_count += 1
            if self.release_count < cfg.confirmations:
                return 0., steering, {**detail, 'reason': 'r0plus_release_wait', 'behavior': 'HOLD'}
            self.held = False
        return float(target), steering, {**detail, 'reason': 'r0plus',
            'behavior': 'FOLLOW' if target < cap-1e-6 else 'CRUISE'}
