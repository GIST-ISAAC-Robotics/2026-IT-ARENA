"""저장 입력 전용: 느린 판단의 경로를 현재 관측으로 다시 검사한다.

ROS 연결/명령 발행/계획 상태 전이 없음. 우회·추월 계획은 아직 지원하지 않는다.
"""
from dataclasses import dataclass
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import pursue_path
from arena_autonomy.local_path import braking_speed, swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.observation_support import source_valid
from arena_autonomy.pass_release_trial import motion_region, region_blocked, lateral_config, SelectorConfig
from arena_autonomy.scene_motion import transport_points


@dataclass(frozen=True)
class PathProposal:
    path: tuple
    reference_s: float
    source_first_s: float
    motion_epoch: int
    sensor_epoch: int
    side: str
    speed_cap_mps: float
    selector_choice: str


def proposal(data, output):
    speed, steering, detail = output
    normal = detail.get('normal') or {}
    if (detail.get('handoff_state') != 'FOLLOW'
            or normal.get('reason') not in ('local_follow', 'parallel_hold_follow')
            or normal.get('selector_choice') not in ('baseline', 'parallel', 'guard_baseline')
            or not math.isfinite(speed) or speed <= 0 or not math.isfinite(steering)):
        return None
    path = np.asarray(normal.get('path_points', []), dtype=float).reshape(-1, 2)
    if len(path) < 6 or not np.all(np.isfinite(path)):
        return None
    raw = data['scan']
    if raw.get('representation') != 'native_measurements_v1':
        return None
    return PathProposal(tuple(map(tuple, path)), data['reference_s'],
        raw['measurements']['acquired_at_s'][0], data['epoch'],
        raw['measurements']['source_epoch'], data['side'], float(speed), normal['selector_choice'])


def current_candidate(plan, data, scan, history, scene, config, *, output_dt_s=.02):
    """현재 움직임/점유/관측 외곽을 검사한 연구 후보. 노면·모드 통합은 미완료."""
    def reject(reason, **details):
        return dict(eligible=False, reason=reason, **details)

    now = data['reference_s']
    if plan is None:
        return reject('no_positive_follow_plan')
    if not 0 <= now-plan.reference_s < .3:
        return reject('plan_reference_expired_or_future')
    if (data['epoch'] != plan.motion_epoch or scan.source_epoch != plan.sensor_epoch
            or data['side'] != plan.side):
        return reject('plan_epoch_or_side_changed')
    if not data['timing_verified'] or not source_valid(scene):
        return reject('current_observation_invalid')
    if not .0 < output_dt_s <= .1:
        raise ValueError('positive bounded command interval required')
    speed = abs(data['speed_mps'])
    previous = data['previous_steering_rad']
    if not all(math.isfinite(v) for v in (speed, previous)):
        return reject('current_motion_invalid')
    age = now-scene.source_first_s
    try:
        path = transport_points(plan.path, history, plan.reference_s, now)
        cap, desired, remaining = pursue_path(path, speed, config, age,
                                             speed_cap=plan.speed_cap_mps)
        steering = float(np.clip(desired, previous-3*output_dt_s, previous+3*output_dt_s))
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        padding = per_ray_padding(scene, .025+config.measurement_padding_m,
                                  config.object_motion_bound_mps)
        clearance = min(swept_limit(points, a, config.wheelbase_m, horizon=2.5,
            margin=padding, length=config.length_m, width=config.width_m)
            for a in (steering, previous))
        available = max(0., clearance-config.standstill_gap_m)
        target = min(plan.speed_cap_mps, cap, braking_speed(available, age),
                     available/config.headway_s)
        if clearance <= config.hold_clearance_m or target <= 0:
            return reject('current_gap_hold', clearance_m=clearance)
        region = motion_region(scene, speed, target, steering, previous,
                               output_dt_s, config, lateral_config(config))
        blocked = region_blocked(region, SelectorConfig())
        if blocked:
            return reject(blocked, region=region)
        return dict(eligible=True, reason='current_observed_tracking_candidate',
            speed_mps=target, steering_rad=steering, plan_reference_s=plan.reference_s,
            original_source_first_s=plan.source_first_s, current_reference_s=now,
            current_source_first_s=scene.source_first_s, original_speed_cap_mps=plan.speed_cap_mps,
            path_points=path.tolist(), observed_path_ahead_m=remaining,
            clearance_m=clearance, region=region, runtime_authorized=False,
            independent_guard_required=True, road_and_mode_contract_pending=True)
    except (MotionUnavailable, ValueError) as error:
        return reject('current_geometry_unavailable', error=str(error))
