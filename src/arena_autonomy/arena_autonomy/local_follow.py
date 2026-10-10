"""G1-a 선택형 지역 경계 확인 + 거리 기반 FOLLOW/HOLD.

새 물체를 차량으로 완벽히 분류하지 않는다. 경로 형상만 짧게 기억하며,
점유 검사에서는 벽 후보를 포함한 모든 현재 반환을 유지한다. 경계 기억을
자유 공간 증거로 사용하지 않는다. 우회/추월·상대 속도 이득은 없다.
"""
from dataclasses import dataclass
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_path import local_path, braking_speed, swept_limit
from arena_autonomy.scene_motion import transport_points


@dataclass(frozen=True)
class FollowConfig:
    wheelbase_m: float = .135
    length_m: float = .20
    width_m: float = .15
    wall_offset_m: float = .425
    max_steering_rad: float = .37
    max_speed_mps: float = 1.0
    lateral_acceleration_mps2: float = 3.5
    memory_ttl_s: float = .30
    memory_motion_limit_m: float = .45
    boundary_min_ahead_m: float = .65
    boundary_anchor_error_m: float = .12
    boundary_agreement_m: float = .08
    measurement_padding_m: float = .03
    # G1-a 저속 표적 시험의 과거 취득 시차 팽창. 미래 cut-in 보증이 아니다.
    object_motion_bound_mps: float = .5
    standstill_gap_m: float = .16
    hold_clearance_m: float = .20
    resume_clearance_m: float = .28
    headway_s: float = .8
    confirmations: int = 2

    def __post_init__(self):
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('follow configuration must be finite and positive')
        if (self.memory_ttl_s > .3 or self.max_speed_mps > 2.5 or self.confirmations < 2
                or not isinstance(self.confirmations, int)
                or not self.standstill_gap_m < self.hold_clearance_m < self.resume_clearance_m):
            raise ValueError('invalid follow time/speed/hysteresis contract')


def path_agrees(candidate, reference, tolerance):
    """가까운 전방 구간의 대칭 거리. 짧은 한 점 접촉을 형상 일치로 보지 않는다."""
    paths = [np.asarray(p)[(np.asarray(p)[:, 0] >= .05) & (np.asarray(p)[:, 0] <= .65)]
             for p in (candidate, reference)]
    if any(len(p) < 5 or np.ptp(p[:, 0]) < .25 for p in paths):
        return False
    distances = np.linalg.norm(paths[0][:, None, :]-paths[1][None, :, :], axis=2)
    return max(np.percentile(np.min(distances, axis=0), 80),
               np.percentile(np.min(distances, axis=1), 80)) <= tolerance


def boundary_support(points, path, side, config):
    """유효한 경계 이력 안에서 일치하는 실제 반환만 경로 후보로 사용한다.

    경로 법선에서 예상 벽을 계산하되 관측점을 만들어 추가하지 않는다.
    이력의 양 끝 너머는 새 경계를 관측할 수 있도록 그대로 둔다. 결과에도
    기존 연결 간격·적합·길이·형상 검사를 적용하며 모든 원래 점유는 별도 유지한다.
    """
    tangent = np.gradient(path, axis=0)
    tangent /= np.maximum(1e-9, np.linalg.norm(tangent, axis=1))[:, None]
    sign = 1 if side == 'left' else -1
    wall = path + sign*config.wall_offset_m*np.column_stack((-tangent[:, 1], tangent[:, 0]))
    delta = points[:, None, :]-wall[None, :, :]
    distances = np.linalg.norm(delta, axis=2)
    nearest = np.argmin(distances, axis=1)
    index = np.arange(len(points))
    longitudinal = np.sum(delta[index, nearest]*tangent[nearest], axis=1)
    outside = ((nearest == 0) & (longitudinal < 0)) | ((nearest == len(wall)-1) & (longitudinal > 0))
    return points[outside | (distances[index, nearest] <= config.boundary_agreement_m)]


def drop_protrusions(points, path, side, config):
    """경계 지원점에서 기대 벽보다 측정 여유 이상 벗어난 다른 물체의 가장자리를 뺀다.

    boundary_agreement_m는 기억/반대 벽 형상 오차의 허용치라 벽 바로 앞의 얇은
    기둥 모서리도 통과시킬 수 있고, 그 한 점이 방위 순서의 벽 연결을 끊는다.
    기대 벽 선분까지의 부호 있는 수직 거리에서 기준 경로의 일정한 치우침(중앙값)을
    빼고 measurement_padding_m을 넘는 점을 제외한다. 점을 만들거나 옮기지 않으며
    양 끝 밖 점은 boundary_support와 같이 둔다. 제외한 점도 점유 검사에는 남는다.
    """
    tangent = np.gradient(path, axis=0)
    tangent /= np.maximum(1e-9, np.linalg.norm(tangent, axis=1))[:, None]
    sign = 1 if side == 'left' else -1
    wall = path + sign*config.wall_offset_m*np.column_stack((-tangent[:, 1], tangent[:, 0]))
    start, step = wall[:-1], np.diff(wall, axis=0)
    length2 = np.maximum(1e-12, np.sum(step*step, axis=1))
    delta = points[:, None, :]-start[None, :, :]
    raw = np.sum(delta*step[None], axis=2)/length2
    fraction = np.clip(raw, 0., 1.)
    residual = delta-fraction[:, :, None]*step[None]
    nearest = np.argmin(np.linalg.norm(residual, axis=2), axis=1)
    index = np.arange(len(points))
    outside = (((nearest == 0) & (raw[index, nearest] < 0)) |
               ((nearest == len(step)-1) & (raw[index, nearest] > 1)))
    # 도로 쪽(경로 방향)이 양수인 선분 법선 성분.
    inward = -sign*np.column_stack((-step[:, 1], step[:, 0]))/np.sqrt(length2)[:, None]
    offset = np.sum(residual[index, nearest]*inward[nearest], axis=1)
    if not np.any(~outside):
        return points
    return points[outside | (np.abs(offset-np.median(offset[~outside])) <= config.measurement_padding_m)]


def pursue_path(path, speed, config, age):
    """기존 후륜축 Pure Pursuit와 같은 계산. 보호층 공통 함수를 변경하지 않는다."""
    rear = np.asarray(path, dtype=float).copy()
    rear[:, 0] += config.wheelbase_m/2
    distances = np.linalg.norm(rear, axis=1)
    nearest = int(np.argmin(distances))
    ahead = np.flatnonzero((rear[:, 0] > .10) & (np.arange(len(rear)) >= nearest))
    if len(ahead) < 3 or np.max(distances[ahead]) < .25:
        raise MotionUnavailable('path_too_short')
    lookahead = min(.60, max(.30, .30+.12*abs(speed)))
    target = rear[ahead[np.argmin(np.abs(distances[ahead]-lookahead))]]
    curvature = float(2*target[1]/max(.01, target@target))
    steering = float(np.clip(math.atan(config.wheelbase_m*curvature),
                             -config.max_steering_rad, config.max_steering_rad))
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(rear, axis=0), axis=1))]
    remaining = float(arc[-1]-arc[nearest])
    dx = np.gradient(rear, axis=0)
    ddx = np.gradient(dx, axis=0)
    curves = np.abs(dx[:, 0]*ddx[:, 1]-dx[:, 1]*ddx[:, 0])/np.maximum(1e-8, np.linalg.norm(dx, axis=1)**3)
    preview = np.sqrt(config.lateral_acceleration_mps2/np.maximum(.15, curves[nearest:]) +
                      4*np.maximum(0., arc[nearest:]-arc[nearest]-.2))
    cap = min(config.max_speed_mps, braking_speed(remaining, age), float(np.min(preview)),
              math.sqrt(config.lateral_acceleration_mps2/max(.15, abs(curvature))))
    return cap, steering, remaining


class LocalFollow:
    def __init__(self, config=None):
        self.config = config or FollowConfig()
        self.reset()

    def reset(self):
        self.path = self.pending = None
        self.path_ref = self.path_source = self.pending_ref = None
        self.pending_count = self.release_count = 0
        self.last_source = self.last_ref = None
        self.epoch = self.side = None
        self.path_wall = None
        self.held = True

    def _hold(self, reason, detail):
        self.held = True
        self.release_count = 0
        return 0., 0., {**detail, 'reason': reason, 'behavior': 'HOLD'}

    def command(self, scene, history, speed, side, previous_steering=0., control_dt=.02):
        cfg, now = self.config, scene.t_ref_s
        detail = {'mode': 'g1_follow', 'source_first_s': scene.source_first_s,
                  'source_last_s': scene.source_last_s, 'source_epoch': scene.source_epoch,
                  'reference_stamp_s': now, 'preferred_wall': side,
                  'relative_velocity_credit': False, 'passing_enabled': False,
                  'measured_speed_mps': speed, 'reverse_motion': math.isfinite(speed) and speed < 0.}
        if (side not in ('left', 'right') or not all(math.isfinite(v) for v in (speed, previous_steering, control_dt))
                or not 0 < control_dt <= .2):
            self.reset()
            return self._hold('follow_invalid_state', detail)
        if (self.epoch != scene.source_epoch or self.side != side or
                (self.last_ref is not None and now < self.last_ref) or
                (self.last_source is not None and scene.source_first_s is not None and
                 scene.source_first_s < self.last_source)):
            self.reset()
        self.epoch, self.side, self.last_ref = scene.source_epoch, side, now
        if scene.source_first_s is None or now-scene.source_first_s >= .3:
            self.reset()
            return self._hold('follow_scene_stale', detail)
        # 후방 사각은 직진 전체를 금지하지 않지만 전방 미관측은 현 보호와 같이 정지.
        front = [ray for ray in scene.rays if abs(math.atan2(ray.direction_xy[1], ray.direction_xy[0])) <= math.radians(100)]
        if not front or any(ray.status != 'return' for ray in front):
            self.reset()
            return self._hold('follow_front_unobserved', detail)
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        if len(points) < 30:
            self.reset()
            return self._hold('follow_scan_invalid', detail)
        fresh = self.last_source is None or scene.source_first_s > self.last_source
        self.last_source = scene.source_first_s
        age = now-scene.source_first_s
        carried = None
        try:
            if self.path is not None and now-self.path_source < cfg.memory_ttl_s:
                carried = transport_points(self.path, history, self.path_ref, now)
                displacement = np.max(np.linalg.norm(carried-self.path, axis=1))
                if displacement > cfg.memory_motion_limit_m:
                    carried = None
            if fresh:
                candidate, info = local_path(points, side, cfg.wall_offset_m)
                preferred, preferred_info = candidate, info
                candidate_wall, corroborated = side, False
                def sufficient(path, details):
                    return (path is not None and details['boundary_observed_ahead_m'] >= cfg.boundary_min_ahead_m
                            and abs(details['wall_distance_m']-cfg.wall_offset_m) <= cfg.boundary_anchor_error_m)

                def supported(reference, wall_side):
                    # 기준 경로와 일치하는 실제 반환만으로 wall_side 벽 후보를 다시 만든다.
                    # 그 후보 자체가 긴 경계·anchor 조건과 형상 일치를 모두 만족해야 한다.
                    selected = boundary_support(points, reference, wall_side, cfg)
                    path, details = local_path(selected, wall_side, cfg.wall_offset_m)
                    ok = sufficient(path, details) and path_agrees(path, reference, cfg.boundary_agreement_m)
                    dropped = 0
                    if not ok:
                        refined = drop_protrusions(selected, reference, wall_side, cfg)
                        if len(refined) < len(selected):
                            path, details = local_path(refined, wall_side, cfg.wall_offset_m)
                            ok = sufficient(path, details) and path_agrees(path, reference, cfg.boundary_agreement_m)
                            dropped = len(selected)-len(refined)
                    return path, details, ok, {'boundary_support_points': len(selected),
                                               'boundary_protrusions_removed': dropped}
                eligible = sufficient(candidate, info)
                detail['boundary_selection'] = 'unfiltered'
                if carried is not None:
                    eligible = eligible and path_agrees(candidate, carried, cfg.boundary_agreement_m)
                    if not eligible:
                        candidate, info, eligible, support = supported(carried, side)
                        detail.update(boundary_selection='observed_history_support', **support)
                other_side = 'right' if side == 'left' else 'left'
                opposite_observed = []

                def opposite_wall():
                    if not opposite_observed:
                        opposite_observed.append(local_path(points, other_side, cfg.wall_offset_m))
                    return opposite_observed[0]
                if not eligible and preferred is not None and abs(preferred_info['wall_distance_m']-cfg.wall_offset_m) <= cfg.boundary_anchor_error_m:
                    # 급커브 안쪽 벽은 짧아질 수 있다. 반대 벽만 보고 전환하지 않고,
                    # 같은 스캔의 실제 선호 벽 경로와 중심 형상이 일치할 때만 사용한다.
                    # 채택할 반대 벽에는 원래의 긴 경계 조건을 그대로 요구한다.
                    opposite, other = opposite_wall()
                    if (sufficient(opposite, other) and path_agrees(opposite, preferred, cfg.boundary_agreement_m)
                            and (carried is None or path_agrees(opposite, carried, cfg.boundary_agreement_m))):
                        candidate, info, candidate_wall = opposite, other, other_side
                        eligible, corroborated = True, True
                        detail['boundary_selection'] = 'bilateral_confirmed_alternative'
                bilateral = corroborated
                if not eligible:
                    # 선호 벽 앞의 얇은 물체가 연결을 끊었거나 그 물체가 기억 경로 범위 밖이면
                    # 같은 스캔의 긴 반대 벽을 기준으로 선호 벽의 실제 반환을 다시 고른다.
                    # 선호 벽 후보 자체가 긴 경계·앵커·형상 일치를 모두 만족해야 하는 양측
                    # 확인이며, 기억이 있으면 그 기억과도 일치해야 한다.
                    opposite, other = opposite_wall()
                    if sufficient(opposite, other):
                        reselected, reselected_info, bilateral, support = supported(opposite, side)
                        bilateral = bilateral and (carried is None or
                                                   path_agrees(reselected, carried, cfg.boundary_agreement_m))
                        if bilateral:
                            candidate, info, eligible = reselected, reselected_info, True
                            detail.update(boundary_selection='bilateral_reference_support', **support)
                detail['boundary_candidate_reason'] = info['reason']
                detail['boundary_candidate_ahead_m'] = info.get('boundary_observed_ahead_m')
                detail['bilateral_alternative_confirmed'] = corroborated
                if eligible and carried is None and not bilateral:
                    # 초기/만료 뒤 재획득에는 반대쪽의 독립된 긴 관측도 일치해야 한다.
                    opposite, other = opposite_wall()
                    eligible = sufficient(opposite, other) and path_agrees(candidate, opposite, cfg.boundary_agreement_m)
                    if not eligible:
                        # 반대 벽 앞의 얇은 물체도 같은 실제 반환 선택으로만 제외한다.
                        _, _, eligible, support = supported(candidate, other_side)
                        if eligible:
                            detail.update(bilateral_reference='opposite_support',
                                          opposite_support_points=support['boundary_support_points'],
                                          opposite_protrusions_removed=support['boundary_protrusions_removed'])
                if eligible:
                    confirmed = carried is not None
                    if not confirmed:
                        old = (transport_points(self.pending, history, self.pending_ref, now)
                               if self.pending is not None and now-self.pending_ref <= cfg.memory_ttl_s else None)
                        self.pending_count = self.pending_count+1 if old is not None and path_agrees(candidate, old, cfg.boundary_agreement_m) else 1
                        self.pending, self.pending_ref = candidate.copy(), now
                        confirmed = self.pending_count >= cfg.confirmations
                    if confirmed:
                        self.path, self.path_ref, self.path_source = candidate.copy(), now, scene.source_first_s
                        self.path_wall = candidate_wall
                        carried = candidate
                else:
                    self.pending = None
                    self.pending_count = 0
                detail['fresh_boundary_accepted'] = bool(eligible and self.path_source == scene.source_first_s)
            if carried is None:
                self.path = None
                return self._hold('follow_boundary_unconfirmed', detail)
            memory_age = now-self.path_source
            cap, desired, remaining = pursue_path(carried, speed, cfg, max(age, memory_age))
            # WallFollow의 3 rad/s 명령 변화율을 반영한 실제 이번 발행각도 평가한다.
            steering = float(np.clip(desired, previous_steering-3*control_dt, previous_steering+3*control_dt))
            margin = .025 + cfg.measurement_padding_m + cfg.object_motion_bound_mps*age
            clearance = min(swept_limit(points, angle, cfg.wheelbase_m, horizon=2.5, margin=margin,
                                         length=cfg.length_m, width=cfg.width_m)
                            for angle in (steering, previous_steering))
            available = max(0., clearance-cfg.standstill_gap_m)
            target = min(cap, braking_speed(available, age), available/cfg.headway_s)
            detail.update(path_source_s=self.path_source, boundary_age_s=memory_age,
                boundary_from_memory=self.path_source != scene.source_first_s, path_wall=self.path_wall,
                wall_fallback=self.path_wall != side, path_points=carried.tolist(), observed_path_ahead_m=remaining,
                swept_clearance_m=clearance, occupancy_padding_m=margin, occupied_points=len(points),
                unfiltered_occupancy=True, target_speed_before_hold_mps=target)
            if speed < 0.:
                # 유한한 signed 피드백은 입력 무효/시간 세대 변경이 아니다. 현재
                # 관측·signed 운동 보정·경계/점유 검사는 유지하되 전진은 금지한다.
                # 매 역방향 표본마다 재개 확인을 지워 지속 역행 중 주행하지 않으며,
                # 정상 측정 복귀 뒤에도 기존 서로 다른 두 fresh 관측을 요구한다.
                return self._hold('follow_reverse_motion', detail)
            if clearance <= cfg.hold_clearance_m:
                return self._hold('follow_gap_hold', detail)
            if self.held:
                if clearance < cfg.resume_clearance_m:
                    self.release_count = 0
                elif fresh:
                    self.release_count += 1
                if self.release_count < cfg.confirmations:
                    return 0., steering, {**detail, 'reason': 'follow_release_wait', 'behavior': 'HOLD'}
                self.held = False
            behavior = 'FOLLOW' if target < cap-1e-6 else 'CRUISE'
            return float(target), steering, {**detail, 'reason': 'local_follow', 'behavior': behavior}
        except MotionUnavailable as error:
            self.reset()
            return self._hold('follow_'+str(error), detail)
