"""M3 유한 실행·제동 전이 후보. 기본 비활성 시뮬레이션 옵션으로만 연결한다.

run17 동결 판본(`m3_observed_follow.M3ObservedFollow`)의 분해에서 확인한 시간·좌표 불일치를 바꾼다.

1. **시작 자세:** 후보 경로의 첫 차체 자세를 실제 현재 차체 (0, 0, 0)에 강체 보정으로 맞춘다. 경로 좌표의
   투영(구간 법선)과 재구성(보간 법선) 차이(μm~mm)가 후방 마스크 옆을 '새 영역'으로 만들던 결함이다.
   보정량이 수치 오차 범위를 넘으면 후보를 무효로 한다(실제 자세 차이를 숨기지 않는다).
2. **이번 명령의 검사 궤적:** 한 제어 주기 동안 이번 발행각(바퀴가 아직 따라오는 중인 이전각 포함)으로
   이동한 뒤, HOLD가 실제로 내보내는 '조향 유지·0 목표'로 기존 보호 상수(반응 0.10 s+데이터 나이,
   2 m/s²)에 따라 정지하는 원호다. 속도 상계는 현재 측정 속도와 이번(재개 시) 목표 중 큰 값이다.
   동결 판본의 0.5 s 확약 동안 같은 각을 유지한다는, 실제로 내보내지 않는 궤적은 쓰지 않는다.
   매 제어 주기가 이 검사를 다시 하며 실패하면 그 주기에서 정지한다.
3. **계획:** 후보 경로는 동결 판본과 같은 확약 거리에서 관측·노면·측방 간격을 검사한다(선택 조건).
   단, 계획 차체 앞면이 경로 위 관측 반환에 닿는 자세부터는 기존 종방향 간격·제동·HOLD가 맡으므로
   그 직전 자세까지만 요구한다. 옆 물체는 측방 간격·관측 검사를 그대로 받는다.
4. **후방 마스크 쪽 질의(선택):** 현재 장면에서 차체 가림 마스크 때문에만 지지되지 않은 질의를, 같은 세대
   0.3 s 안의 이전 장면 기준 좌표로 역변환해 기존 물체 이동 상계 0.5 m/s × (현재-취득) 시간을 더한 여유로
   다시 확인한다. 과거 미관측 영역에서 질의점까지 같은 속도 상계로 옆 방향 진입이 가능하면 재사용하지
   않는다. 무응답(unknown)·현재 반환 뒤(가림)는 현재 장면만 인정한다. 정지 중에는 이전 장면도
   같은 마스크라 지지하지 못한다. 이동 상계를 넘는 숨은 차량은 미해결이다.
5. **직진 출발(선택):** 현재 진행 방향 직진 후보를 마지막 선택지로 둔다. 최근 0.1 s 모터축 0·낮은 자이로가
   유효한 표본으로 함께 확인되고
   직진 후보만 남았을 때 0 목표를 유지한 채 조향만 0으로 정렬한다. 차체 정지 자체는 확인하지 못한다.

6. **기본 G1 복귀(선택):** 측방 위협이 사라진 뒤, 마지막으로 위협이 보인 장면의 가능 몸체 영역을 운동 이력으로
   현재 좌표에 옮기고 이동 상계·거리 오차만큼 넓힌다. 그 영역을 지금 스캔의 양의 free ray로 확인하고(관측 표면과
   측방 창보다 앞에서 반환 뒤에 가려진 칸만 예외), 위협이 있을 수 있는 칸이 자차의 복귀 회랑·정지 거리 안에 없을
   때만 시험을 끝낸다. 반환 부재·점 개수 미달·fresh 2회만으로는 해제하지 않는다. fresh 2회는 깜박임 방지일 뿐이다.

7. **작은 중앙 보정 생략(선택):** 위협이 없고 중앙 후보가 촉발된 단계에서, 현재 방향 직진의 후륜축 경로가 그 계획
   거리 전체에서 기준 경로로부터 직선 추종 오차 예산(`tracking_straight_m`) 안이고 곡선 구간이 아니면 직진을 먼저
   검사한다. 현재 횡오차·방향 오차·기준 경로 곡률이 모두 그 예측 편차에 들어가며, 매 판단 현재 자세에서 다시 계산하므로
   누적 편차가 예산을 넘으면 중앙 후보가 다시 먼저다. 직진도 같은 계획·발행 외곽 검사를 받고, 거절되면 기존 순서로
   돌아간다. 0이 아닌 미세 조향의 이전각 원호가 다음 주기의 후방 마스크 꼬리 띠 검사에서 전 후보를 막던 정지 반복을
   줄이려는 순서 비교이며, 관측·점유·노면·위협 기록 기준을 바꾸지 않는다.

연속 외곽·숨은 차량 미래 진입·실차 제동/슬립은 미해결이며 `runtime_authorized`는 항상 false다.
"""
from collections import deque
from dataclasses import dataclass, replace
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow, pursue_path
from arena_autonomy.local_path import swept_limit, braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.lateral_intent import (LateralIntentConfig, ObservationGrid, PathFrame, ego_state,
    cell_coordinates, corridor_reference, side_threats, offset_candidate, surface_check, lateral_clearance_check,
    committed_progress_m, wheel_contact_corners, compact_return_mask, GEOMETRY_EPS_M, SIDES)
from arena_autonomy.observed_lateral_trial import path_observation_queries
from arena_autonomy.observation_support import depth_support, source_valid
from arena_autonomy.observation_lateral import reachable_from_unknown
from arena_autonomy.linear_observation import quiet_rotation, QUIET_INTERVAL_S
from arena_autonomy.scene_motion import relative_poses, transport_points
from arena_autonomy.m3_observed_follow import command_arc

# 경로 좌표 왕복의 수치 차이만 보정한다. 실제 자세 차이를 덮는 허용치가 아니다.
START_TOLERANCE_M = .005
START_TOLERANCE_RAD = .02
# 단일 모터축 측정 0. 바퀴 정지일 뿐 차체 정지 증명이 아니다.
STOPPED_SPEED_MPS = 1e-6


class ExecutionFrame(PathFrame):
    """한 판단 안에서 같은 점열의 동일 경로 투영만 재사용한다. 시각 사이 캐시는 없다."""

    def __init__(self, path):
        super().__init__(path)
        self._project_cache = {}

    def project(self, points):
        q = np.asarray(points, dtype=float)
        key = (q.shape, q.tobytes())
        if key in self._project_cache:
            return tuple(v.copy() for v in self._project_cache[key])
        result = super().project(q)
        if len(self._project_cache) >= 16:
            self._project_cache.pop(next(iter(self._project_cache)))
        self._project_cache[key] = tuple(v.copy() for v in result)
        return result


@dataclass(frozen=True)
class ExecutionConfig:
    masked_history_support: bool = True
    straight_start: bool = True
    command_fallback: bool = False  # 계획 통과 뒤 실제 명령이 거절되면 다음 후보도 검사하는 비교 옵션
    # 마지막 위협의 가능 영역을 지금 스캔으로 확인한 뒤에만 기본 G1 출력으로 돌려주는 비교 옵션(모듈 설명 6).
    # 해제 뒤에는 측방 시험의 관측 계약 밖이며, 기본 G1은 후방 마스크·측방 예산을 검사하지 않는다.
    release_to_baseline: bool = False
    # 중앙 복귀 단계에서 보정이 직선 추종 오차 예산 안이면 현재 방향 직진을 중앙 후보보다 먼저 검사하는 비교 옵션
    # (모듈 설명 7). 검사 순서만 바꾸고 직진도 같은 계획·발행 검사를 받는다. 마지막 조향각 유지가 아니다.
    small_center_straight: bool = False
    # 직진 계획의 노면 범위를 같은 계획의 확약+정지 거리로 맞춘다. 곡선 여유와 명령 외곽 검사는 유지한다.
    straight_surface_commitment: bool = False
    reaction_s: float = .10          # 기존 보호 반응 상수(local_path.braking_speed)
    deceleration_mps2: float = 2.0   # 기존 보호 감속 상수. 실차 제동 검증값이 아니다
    history_motion_bound_mps: float = .5   # 기존 점유 취득 시차 팽창과 같은 물체 이동 상계

    def __post_init__(self):
        if any(type(v) is not bool for v in (self.masked_history_support, self.straight_start, self.command_fallback,
                                             self.release_to_baseline, self.small_center_straight,
                                             self.straight_surface_commitment)):
            raise ValueError('execution options must be bool')
        if self.small_center_straight and not self.release_to_baseline:
            # 기존 '중앙 후보 선택 = 시험 종료' 경로를 직진 선택이 건너뛰므로 관측 기반 복귀와만 비교한다.
            raise ValueError('small center straight requires observed release to baseline')
        if self.straight_surface_commitment and not (self.release_to_baseline and self.command_fallback
                                                      and self.straight_start):
            raise ValueError('straight surface commitment requires observed release, command fallback and straight start')
        for value in (self.reaction_s, self.deceleration_mps2, self.history_motion_bound_mps):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('execution constants must be finite and positive')


def envelope_distance(speed, age, control_dt, cfg):
    """이번 주기 이동 + 반응(데이터 나이 포함) + 제동 거리. 기존 braking_speed와 같은 상수."""
    v = abs(speed)
    return v*(control_dt+max(0., age)+cfg.reaction_s)+v*v/(2*cfg.deceleration_mps2)


def presteer_motion_quiet(history, now):
    """모터축·자이로의 최근 정지 일치만 검사한다. 미끄러지는 차체의 정지 증명은 아니다."""
    if not quiet_rotation(history, now):
        return False
    try:
        # 공백/노후/미래 입력은 원래 운동 이력의 동일한 관문으로 거절한다.
        history.poses([now-QUIET_INTERVAL_S, now], now)
    except MotionUnavailable:
        return False
    stamps, values = history.wheels.available(now)
    first = max(0, int(np.searchsorted(stamps, now-QUIET_INTERVAL_S, side='right'))-1)
    return bool(len(values[first:]) and np.all(np.abs(values[first:]) <= STOPPED_SPEED_MPS))


def anchored(candidate):
    """첫 차체 자세가 정확히 (0, 0, 0)이 되도록 후보 전체를 강체 보정한다."""
    poses = np.asarray(candidate['poses'], dtype=float)
    x0, y0, h0 = poses[0]
    h0 = math.remainder(h0, math.tau)
    c, s = math.cos(-h0), math.sin(-h0)
    rotation = np.array([[c, -s], [s, c]])
    moved = (poses[:, :2]-[x0, y0])@rotation.T
    out = {**candidate, 'poses': np.column_stack((moved, poses[:, 2]-h0)),
           'path': (np.asarray(candidate['path'], dtype=float)-[x0, y0])@rotation.T,
           'start_correction_m': float(math.hypot(x0, y0)), 'start_correction_rad': float(abs(h0))}
    if math.hypot(x0, y0) > START_TOLERANCE_M or abs(h0) > START_TOLERANCE_RAD:
        out.update(valid=False, problems=list(candidate['problems'])+['start_pose_mismatch'])
    return out


def straight_candidate(frame, state, cfg):
    """현재 차체 방향 직진. 후륜축 경로가 x축 위라 Pure Pursuit 조향도 0이다."""
    lr = cfg.wheelbase_m/2
    end = min(frame.end_s, state['s_rear0']+cfg.horizon_m)
    problems = [] if end-state['s0'] >= cfg.min_ahead_m else ['short_ahead']
    progress = np.arange(0., max(end-state['s_rear0'], .05)+1e-9, .025)
    zero = np.zeros_like(progress)
    rear = np.column_stack((progress-lr, zero))
    s, l = frame.project(rear)
    return {'name': 'straight', 'd_target_m': float(l[-1]), 'delta_m': float(l[-1]-state['d_rear0']),
            'blend_m': 0., 'progress': progress, 'offset': l, 'poses': np.column_stack((progress, zero, zero)),
            'path': rear, 'path_heading': frame.heading(s), 'max_abs_curvature_m_inv': 0.,
            'max_abs_steering_rad': 0., 'steering_rad': zero.copy(),
            'curved': bool(np.max(np.abs(frame.curvature(s))) > cfg.curved_curvature_m_inv),
            'valid': not problems, 'problems': problems, 'start_correction_m': 0., 'start_correction_rad': 0.}


def small_center_check(straight, cfg):
    """직진의 후륜축 경로가 계획 거리 전체에서 기준 경로와 직선 추종 오차 예산 안인지. 순서 판단일 뿐 안전 판정이 아니다.

    현재 횡오차만 보는 deadband가 아니다. 기준 경로에 투영한 편차에 시작 횡오차·방향 오차·기준 곡률이 함께 들어간다.
    """
    offset = np.abs(np.asarray(straight['offset'], dtype=float))
    worst = float(np.max(offset)) if len(offset) else math.inf
    limit = cfg.budget.tracking_straight_m
    return {'ok': bool(straight['valid'] and not straight['curved'] and worst <= limit),
            'max_abs_offset_m': worst, 'limit_m': limit, 'curved': bool(straight['curved']),
            'valid': bool(straight['valid']), 'planned_length_m': float(straight['progress'][-1])}


def ray_padding(scene, cfg, bound):
    return np.array([.025+cfg.budget.range_bias_m+bound*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])


def masked_only(scene, queries, pad):
    """양옆 광선 중 반환이 아닌 쪽이 모두 차체 가림 마스크인 질의. 무응답·가림 뒤는 False.

    depth_support와 같은 양옆 광선 규칙의 원인 판별이다. 지지 여부 자체는 depth_support가 정한다.
    """
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    out = np.zeros(len(q), dtype=bool)
    if not len(q):
        return out
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    status = np.array([r.status for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    depth = np.array([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return' else -1. for r in scene.rays])
    for k, point in enumerate(q):
        d = point-origins
        angle = np.arctan2(directions[:, 0]*d[:, 1]-directions[:, 1]*d[:, 0],
                           directions[:, 0]*d[:, 0]+directions[:, 1]*d[:, 1])
        radius = np.hypot(d[:, 0], d[:, 1])
        low = np.where(angle >= -1e-10, np.maximum(angle, 0.), np.inf)
        high = np.where(angle <= 1e-10, np.maximum(-angle, 0.), np.inf)
        lo, hi = int(np.argmin(low)), int(np.argmin(high))
        if (not np.isfinite(low[lo]) or not np.isfinite(high[hi]) or low[lo]+high[hi] > .025
                or abs(int(indices[lo])-int(indices[hi])) > 1):
            continue
        pair = (lo, hi)
        if any(status[i] == 'unknown' for i in pair) or not any(status[i] == 'masked' for i in pair):
            continue
        out[k] = all(depth[i]-radius[i]-pad[i] > 0 for i in pair if status[i] == 'return')
    return out


class SceneHistory:
    """같은 세대·0.3 s 안의 이전 장면(원래 취득 좌표 그대로). 같은 스캔으로 갱신하지 않고 역행 시 비운다."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.scenes = deque(maxlen=3)
        self.last_ref = self.epoch = None

    def update(self, scene, history):
        """현재 장면을 기록하고, 현재 기준 좌표로의 (dx, dy, yaw)와 함께 이전 장면 목록을 반환한다."""
        if (not source_valid(scene) or (self.last_ref is not None and scene.t_ref_s < self.last_ref) or
                (self.scenes and scene.source_first_s < self.scenes[-1].source_first_s)):
            self.reset()
            return []
        if self.epoch is not None and self.epoch != scene.source_epoch:
            self.reset()
        self.last_ref, self.epoch = scene.t_ref_s, scene.source_epoch
        if not self.scenes or scene.source_first_s > self.scenes[-1].source_first_s:
            self.scenes.append(scene)
        while self.scenes and scene.t_ref_s-self.scenes[0].source_first_s >= .3:
            self.scenes.popleft()
        frames = []
        for old in reversed(self.scenes):
            if old.source_first_s >= scene.source_first_s or old.source_epoch != scene.source_epoch:
                continue
            try:
                dx, dy, yaw = relative_poses(history, [old.t_ref_s], scene.t_ref_s)[0]
            except MotionUnavailable:
                continue
            frames.append((old, (float(dx), float(dy), float(yaw))))
        return frames


def support(scene, frames, queries, cfg, execution):
    """현재 장면 지지 + (선택) 마스크 원인 질의의 이전 장면 지지.

    이전 장면 광선을 현재 좌표로 옮기는 대신 질의점을 이전 장면의 기준 좌표로 역변환한다(같은 강체
    관계). 여유는 현재 시각 기준 광선 나이(기존 0.5 m/s 이동 상계)로 계산한다. 거리 방향
    여유만으로는 옆 미관측 영역의 진입을 막지 못하므로 과거 unknown/masked의 도달 가능성도 검사한다.
    """
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = ray_padding(scene, cfg, execution.history_motion_bound_mps)
    ok = depth_support(scene, q, pad) if len(q) else np.zeros(0, dtype=bool)
    detail = {'queries': int(len(q)), 'current_unsupported': int(np.sum(~ok)), 'masked_candidates': 0,
              'history_supported': 0, 'history_sources': 0, 'history_reachable_rejections': 0}
    if execution.masked_history_support and np.any(~ok) and frames:
        missing = np.flatnonzero(~ok)
        masked = masked_only(scene, q[missing], pad)
        detail['masked_candidates'] = int(masked.sum())
        remaining = missing[masked]
        reach_config = FollowConfig(length_m=cfg.length_m, width_m=cfg.width_m,
                                    object_motion_bound_mps=execution.history_motion_bound_mps)
        for old, (dx, dy, yaw) in frames:
            if not len(remaining):
                break
            if (not source_valid(old) or old.source_epoch != scene.source_epoch
                    or old.source_first_s >= scene.source_first_s):
                continue
            c, sn = math.cos(yaw), math.sin(yaw)
            local = (q[remaining]-[dx, dy])@np.array([[c, -sn], [sn, c]])
            old_pad = np.array([.025+cfg.budget.range_bias_m+execution.history_motion_bound_mps*(
                scene.t_ref_s-r.acquired_at_s) for r in old.rays])
            accepted = depth_support(old, local, old_pad)
            possible = np.flatnonzero(accepted)
            if len(possible):
                # 좌표는 과거 장면 그대로, 도달 시간만 현재 기준이다. 새 관측을 생성하지 않는다.
                reached = reachable_from_unknown(replace(old, t_ref_s=scene.t_ref_s),
                                                  local[possible], reach_config)
                accepted[possible[reached]] = False
                detail['history_reachable_rejections'] += int(reached.sum())
            ok[remaining[accepted]] = True
            detail['history_supported'] += int(accepted.sum())
            detail['history_sources'] += 1
            remaining = remaining[~accepted]
    detail.update(supported=bool(np.all(ok)), unsupported=int(np.sum(~ok)))
    return detail


def body_queries(scene, poses, steering, cfg):
    """차체 외곽·얇은 unknown 교차·바퀴 지름 외곽(동결 판본과 같은 질의)."""
    q, thin = path_observation_queries(scene, poses, cfg)
    wheel = wheel_contact_corners(poses, steering, replace(cfg, contact_patch_length_m=2*cfg.wheel_radius_m))
    wheel = wheel.reshape(-1, 2)
    wheel = wheel[(abs(wheel[:, 0]) > cfg.length_m/2+GEOMETRY_EPS_M) | (abs(wheel[:, 1]) > cfg.width_m/2+GEOMETRY_EPS_M)]
    return np.unique(np.vstack((q.reshape(-1, 2), wheel)), axis=0), thin


def arc_checks(scene, frames, reference_path, angles, distance, cfg, execution):
    """발행각/이전각 정곡률 원호의 관측·노면·측방 간격. 원호 길이는 정지 분기까지만이다."""
    frame = ExecutionFrame(reference_path)
    points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
    ages = np.asarray([scene.t_ref_s-r.acquired_at_s for r in scene.rays if r.status == 'return'])
    queries, thin, checks = [], 0, []
    for angle in sorted(set(angles)):
        poses = command_arc(angle, distance, cfg)
        q, n = body_queries(scene, poses, np.full(len(poses), angle), cfg)
        queries.append(q)
        thin += n
        rear = poses[:, :2]-cfg.wheelbase_m/2*np.column_stack((np.cos(poses[:, 2]), np.sin(poses[:, 2])))
        s, _ = frame.project(rear)
        candidate = {'poses': poses, 'steering_rad': np.full(len(poses), angle), 'max_abs_steering_rad': abs(angle),
                     'progress': np.linspace(0., distance, len(poses)),
                     'curved': bool(np.max(np.abs(frame.curvature(s))) > cfg.curved_curvature_m_inv)}
        surface = surface_check(frame, candidate, cfg, current_pose=(0., 0., 0.))
        reasons = [] if surface['ok'] else [surface['reason']]
        if surface.get('grass_side'):
            lo, hi = surface['s_range_needing_edge']
            edge = corridor_reference(frame, points, lo-.05, hi+.05, cfg)
            if not edge[surface['grass_side']]['edge_verified']:
                reasons.append('grass_edge_reference_unverified')
        corridor = corridor_reference(frame, points, float(np.min(s))-.1, float(np.max(s))+.1, cfg)
        lateral = lateral_clearance_check(candidate, points, ages, frame, corridor, cfg, distance)
        if not lateral['ok']:
            reasons.append('lateral_clearance_short')
        checks.append({'steering_rad': angle, 'reasons': reasons,
                       'surface': {k: surface.get(k) for k in ('ok', 'grass_side', 'reason', 'edge_margin_m')},
                       'lateral_min_margin_m': lateral.get('min_margin_m')})
    observation = support(scene, frames, np.unique(np.vstack(queries), axis=0), cfg, execution)
    observation['unknown_sector_intersections'] = thin
    return {'distance_m': distance, 'observation': observation, 'geometry': checks,
            'observation_ok': observation['supported'], 'geometry_ok': all(not c['reasons'] for c in checks),
            'continuous_sweep_certified': False}


def presteer_support(scene, frames, angles, cfg, execution):
    """정지 중 조향 변경의 현재 자세 바퀴 외곽(차체 밖 부분) 관측. 차체 이동은 검사 대상이 아니다."""
    pose = np.zeros((1, 3))
    wheel_cfg = replace(cfg, contact_patch_length_m=2*cfg.wheel_radius_m)
    q = [wheel_contact_corners(pose, np.array([a]), wheel_cfg).reshape(-1, 2) for a in sorted(set(angles))]
    q = np.unique(np.vstack(q), axis=0)
    q = q[(abs(q[:, 0]) > cfg.length_m/2+GEOMETRY_EPS_M) | (abs(q[:, 1]) > cfg.width_m/2+GEOMETRY_EPS_M)]
    return support(scene, frames, q, cfg, execution)


def in_path_free_progress(scene, candidate, cfg, follow_padding):
    """계획 차체 앞면이 관측 반환(점유 팽창 포함)에 처음 닿는 진행 거리. 옆 물체는 세지 않는다.

    경로 위 앞 물체는 기존 종방향 간격·제동·HOLD가 맡는다(lateral_clearance_check와 같은 분담).
    그 너머의 계획 관측을 이번 선택 조건으로 요구하지 않을 뿐, 점유·발행 궤적 검사는 그대로다.
    """
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    if not len(points):
        return math.inf
    margin = np.asarray(follow_padding, dtype=float)
    for pose, progress in zip(candidate['poses'], candidate['progress']):
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        dx, dy = points[:, 0]-pose[0], points[:, 1]-pose[1]
        lx, ly = c*dx+sn*dy, -sn*dx+c*dy
        front = (lx > cfg.length_m/2) & (lx <= cfg.length_m/2+margin) & (np.abs(ly) <= cfg.width_m/2+margin)
        if np.any(front):
            return float(progress)
    return math.inf


def straight_surface_prefix(candidate, distance):
    """직진 노면 검사에만 쓸 확약 접두 구간. 끝 자세를 보간하고 원래 곡률 여유를 보존한다.

    경로/Pure Pursuit 목표를 자르거나 반응·제동 거리/간격을 줄이는 함수가 아니다.
    직진의 x=진행 거리, y=yaw=조향=0 불변식을 검사해 곡선의 선형 보간으로 오용하지 않는다.
    """
    progress = np.asarray(candidate['progress'], dtype=float)
    poses = np.asarray(candidate['poses'], dtype=float)
    steering = np.asarray(candidate['steering_rad'], dtype=float)
    if (candidate['name'] != 'straight' or not math.isfinite(distance) or distance < 0
            or progress.ndim != 1 or len(progress) < 2 or poses.shape != (len(progress), 3)
            or steering.shape != progress.shape or not np.all(np.isfinite(progress))
            or not np.all(np.isfinite(poses)) or not np.all(np.isfinite(steering))
            or progress[0] != 0 or np.any(np.diff(progress) <= 0)
            or np.any(poses[:, 0] != progress) or np.any(poses[:, 1:] != 0) or np.any(steering != 0)):
        raise ValueError('invalid straight surface prefix')
    if distance > progress[-1]:
        raise ValueError('stopping horizon unavailable')
    before = progress < distance
    selected = np.r_[progress[before], distance]
    result = dict(candidate)
    # 직진 자세는 정확한 선형 보간이다. 경계 샘플을 내려 버리지 않는다.
    result.update(progress=selected, poses=np.column_stack((selected, np.zeros((len(selected), 2)))),
                  steering_rad=np.zeros(len(selected)))
    return result


def plan_check(scene, frames, candidate, context, cfg, execution, speed):
    """동결 observed_candidate와 같은 계획 조건. 시작 보정·마스크 이력 지지·앞 물체 분담만 다르다."""
    result = {'target_m': candidate['d_target_m'], 'eligible_for_trial': False, 'reasons': [],
              'start_correction_m': candidate.get('start_correction_m'),
              'hidden_future_risk_unresolved': True, 'continuous_sweep_certified': False}
    if not candidate['valid']:
        result['reasons'].append('geometry_invalid')
        result['problems'] = list(candidate['problems'])
        return result
    surface_candidate = candidate
    if execution.straight_surface_commitment and candidate['name'] == 'straight':
        committed = committed_progress_m(speed, context['age'], cfg)
        try:
            surface_candidate = straight_surface_prefix(candidate, committed)
        except ValueError as error:
            result['reasons'].append('straight_surface_horizon_unavailable')
            result['surface_error'] = str(error)
            return result
        result['surface_scope'] = {'kind': 'straight_commitment', 'end_m': committed,
                                   'full_end_m': float(candidate['progress'][-1]),
                                   'curved_margin_preserved': bool(candidate['curved'])}
    surface = surface_check(context['frame'], surface_candidate, cfg, current_pose=(0., 0., 0.))
    result['surface'] = {k: surface.get(k) for k in ('ok', 'grass_side', 'reason', 'edge_margin_m')}
    if not surface['ok']:
        result['reasons'].append(surface['reason'])
    if surface.get('grass_side'):
        lo, hi = surface['s_range_needing_edge']
        ref = corridor_reference(context['frame'], context['grid'].returns, lo-.05, hi+.05, cfg)
        if not ref[surface['grass_side']]['edge_verified']:
            result['reasons'].append('grass_edge_reference_unverified')
    committed = committed_progress_m(speed, context['age'], cfg)
    blocked = in_path_free_progress(scene, candidate, cfg, context['follow_padding'])
    distance = min(committed, blocked)
    result.update(committed_progress_m=committed, in_path_free_progress_m=None if math.isinf(blocked) else blocked)
    lateral = lateral_clearance_check(candidate, context['grid'].returns, context['grid'].ages,
                                      context['frame'], context['corridor'], cfg, distance)
    result['lateral_min_margin_m'] = lateral.get('min_margin_m')
    if not lateral['ok']:
        result['reasons'].append('lateral_clearance_short')
    if result['reasons']:
        # 이미 기하/노면/간격에서 탈락했다. 비싼 관측 검사를 생략해도 후보 선택·출력은 바뀌지 않는다.
        # 검사하지 않은 관측을 통과/미관측으로 기록하지 않고 생략 사유를 남긴다.
        result.update(observation=None, observation_skipped='geometry_rejected', required_progress_m=distance)
        return result
    # 앞 물체에 닿는 자세 자체는 종방향 정지 대상이므로 그 직전 자세까지만 계획 관측을 요구한다.
    keep = (candidate['progress'] <= committed+GEOMETRY_EPS_M) & (candidate['progress'] < blocked)
    if candidate['progress'][-1] < committed-GEOMETRY_EPS_M and candidate['progress'][-1] < blocked:
        result['reasons'].append('stopping_horizon_unavailable')
    q, thin = body_queries(scene, candidate['poses'][keep], candidate['steering_rad'][keep], cfg)
    observation = support(scene, frames, q, cfg, execution)
    result.update(observation=observation, unknown_sector_intersections=thin, required_progress_m=distance)
    if not observation['supported']:
        result['reasons'].append('current_sweep_unobserved')
    result['eligible_for_trial'] = not result['reasons']
    return result


class ReturnGrid:
    """측방 반환이 하나도 없을 때 쓰는 반환 정보. 이때 side_threats는 칸 상태/좌표를 읽지 않는다."""

    def __init__(self, scene, cfg):
        rays = [r for r in scene.rays if r.status == 'return']
        self.returns = np.asarray([r.hit_xy_m for r in rays], dtype=float).reshape(-1, 2)
        self.ages = np.asarray([scene.t_ref_s-r.acquired_at_s for r in rays], dtype=float)
        self.compact_returns, self.return_components = compact_return_mask(scene, cfg)


def side_return_masks(frame, state, grid, cfg):
    """side_threats와 같은 창·벽·횡위치·compact 조건의 측방 반환 선택(좌, 우)과 반환별 ds."""
    if not len(grid.returns):
        empty = np.zeros(0, dtype=bool)
        return np.zeros(0), (empty, empty)
    s, l = frame.project(grid.returns)
    ds = s-state['s0']
    window = (ds >= -cfg.band_behind_m) & (ds <= cfg.band_ahead_m)
    wall = np.zeros(len(l), dtype=bool)
    for _, sign in SIDES:
        wall |= np.abs(l-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m
    interior = np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m
    return ds, tuple(window & interior & ~wall & (sign*(l-state['d0']) >= state['half_span_m'])
                     & grid.compact_returns for _, sign in SIDES)


def side_return_ds(frame, state, grid, cfg):
    """side_threats와 같은 조건의 측방 반환별 종방향 위치 ds(좌, 우)."""
    ds, masks = side_return_masks(frame, state, grid, cfg)
    return tuple(ds[m] for m in masks)


def side_points(frame, state, grid, cfg):
    """side_threats와 같은 창·벽·횡위치·compact 조건의 측방 반환 개수(좌, 우)."""
    return tuple(int(len(ds)) for ds in side_return_ds(frame, state, grid, cfg))


# 위협 가능 영역의 이산 표본 간격. 2 cm 외곽 표본과 같은 해상도이며 연속 영역 증명이 아니다.
RELEASE_GRID_M = .02
# 위협 기록을 옮기고 넓히는 최대 경과 시간. 넘은 기록은 해제를 허용하지 않는다(시험 유지).
RELEASE_MAX_AGE_S = 1.0


@dataclass(frozen=True)
class ThreatRecord:
    """위협이 마지막으로 확인된 장면과 그 장면 기준 좌표의 위협 반환·반환별 취득 시각.

    반환은 보이는 면일 뿐 물체 전체가 아니다. 몸체는 "반환에서 등록 최대 차체 대각선 이내이고 그 장면에서
    관측 자유가 아니었던 칸"으로만 추정한다. 더 큰 물체·여러 물체가 붙은 경우는 이 계약 밖이다.
    """
    side: str
    scene: object
    points: np.ndarray
    acquired: np.ndarray
    possible_xy_m: object = None   # 이후 스캔에서 자유로 확인되지 않은 전체 영역. 반환점만으로 축소하지 않는다.
    detected_at_s: object = None


def _cells(lo, hi, step=RELEASE_GRID_M):
    xs, ys = np.arange(lo[0], hi[0]+step/2, step), np.arange(lo[1], hi[1]+step/2, step)
    return np.stack(np.meshgrid(xs, ys, indexing='ij'), axis=-1).reshape(-1, 2)


def _within(cells, centers, radius):
    out = np.zeros(len(cells), dtype=bool)
    for k in range(0, len(centers), 256):
        d = cells[:, None, :]-centers[None, k:k+256, :]
        out |= np.min(d[..., 0]**2+d[..., 1]**2, axis=1) <= radius*radius
    return out


def _inside_body(points, cfg):
    return (np.abs(points[:, 0]) <= cfg.length_m/2) & (np.abs(points[:, 1]) <= cfg.width_m/2)


def _both_sides_returned(scene, points):
    """depth_support와 같은 양옆 광선 규칙에서 두 광선이 모두 반환인지(무응답·마스크가 아닌지)."""
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    status = np.array([r.status for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    out = np.zeros(len(points), dtype=bool)
    for k, point in enumerate(points):
        d = point-origins
        angle = np.arctan2(directions[:, 0]*d[:, 1]-directions[:, 1]*d[:, 0],
                           directions[:, 0]*d[:, 0]+directions[:, 1]*d[:, 1])
        low = np.where(angle >= -1e-10, np.maximum(angle, 0.), np.inf)
        high = np.where(angle <= 1e-10, np.maximum(-angle, 0.), np.inf)
        lo, hi = int(np.argmin(low)), int(np.argmin(high))
        out[k] = bool(np.isfinite(low[lo]) and np.isfinite(high[hi]) and low[lo]+high[hi] <= .025
                      and abs(int(indices[lo])-int(indices[hi])) <= 1 and status[lo] == status[hi] == 'return')
    return out


def threat_record(side, scene, grid, mask):
    rays = [r for r in scene.rays if r.status == 'return']   # ReturnGrid.returns와 같은 순서
    acquired = np.asarray([r.acquired_at_s for r in rays], dtype=float)
    return ThreatRecord(side, scene, np.asarray(grid.returns, dtype=float)[mask].copy(), acquired[mask].copy())


def release_gate(record, scene, history, frame, state, grid, cfg, execution, stop_distance):
    """마지막 위협 기록의 가능 영역을 지금 스캔으로 확인한다. 해제 여부와 갱신 기록을 돌려준다.

    계약: 기록 장면에서 반환점으로부터 hypot(등록 길이, 최대 폭) 이내이고 관측 자유가 아니었던 칸(당시 자차 차체
    제외)을 운동 이력으로 현재 좌표에 옮긴 뒤, 0.5 m/s × (현재 − 가장 오래된 반환 취득) + 0.025 m + 거리 오차만큼
    넓힌다(도로 안·현재 자차 밖). 그 칸은 지금 관측 자유이거나, 양옆 광선이 모두 반환이면서 관측 표면에 붙어
    있거나 측방 창보다 앞에서 가려진 칸이어야 한다. 무응답·마스크·창 안 가림이 하나라도 있으면 확인 실패다.
    위협이 있을 수 있는 칸(관측 자유가 아닌 칸)이 위협 쪽 복귀 회랑과 기본 G1 상한 속도의 정지 거리 안에 있으면
    해제하지 않는다. 이산 2 cm 표본이며 이동 상계를 넘는 물체·마스크 안 다른 차의 미래 진입은 다루지 않는다.
    """
    detail = {'side': record.side, 'ok': False, 'release': False, 'reason': None, 'record': record}
    old = record.scene
    age = scene.t_ref_s-old.t_ref_s
    detection_age = scene.t_ref_s-(old.t_ref_s if record.detected_at_s is None else record.detected_at_s)
    detail['record_age_s'] = float(age)
    detail['detection_age_s'] = float(detection_age)
    if (old.source_epoch != scene.source_epoch or not scene.source_first_s > old.source_first_s
            or not 0 < age <= RELEASE_MAX_AGE_S or detection_age > RELEASE_MAX_AGE_S
            or not len(record.points)):
        return {**detail, 'reason': 'record_stale_or_inconsistent'}
    bound = execution.history_motion_bound_mps
    if record.possible_xy_m is None:
        reach = math.hypot(cfg.max_vehicle_length_m, cfg.max_vehicle_width_m)
        body = _cells(record.points.min(axis=0)-reach, record.points.max(axis=0)+reach)
        body = body[_within(body, record.points, reach) & ~_inside_body(body, cfg)]
        body = body[~depth_support(old, body, ray_padding(old, cfg, bound))]
        grow = bound*(scene.t_ref_s-float(np.min(record.acquired)))+.025+cfg.budget.range_bias_m
    else:
        body = record.possible_xy_m
        # 표본 영역을 다시 표본화하며 두 격자의 반대각선만큼 보수적으로 더한다.
        # 거리 오차는 최초 가능 몸체에 이미 포함했다. 실제 반환이 없는 영역도 끝까지 보존한다.
        grow = bound*age+math.sqrt(2.)*RELEASE_GRID_M
    if not len(body):   # 반환점 자체는 관측 표면이므로 비면 기록이 장면과 맞지 않는다
        return {**detail, 'reason': 'record_stale_or_inconsistent'}
    try:
        current = transport_points(body, history, old.t_ref_s, scene.t_ref_s)
    except MotionUnavailable:
        return {**detail, 'reason': 'motion_unavailable'}
    region = _cells(current.min(axis=0)-grow, current.max(axis=0)+grow)
    region = region[_within(region, current, grow+1e-9)]
    _, l = frame.project(region)
    region = region[(np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m) & ~_inside_body(region, cfg)]
    pad = ray_padding(scene, cfg, bound)
    rest = region[~depth_support(scene, region, pad)] if len(region) else region
    s, l = frame.project(rest) if len(rest) else (np.zeros(0), np.zeros(0))
    ds = s-state['s0']
    returned = _both_sides_returned(scene, rest)
    hits = np.asarray(grid.returns, dtype=float).reshape(-1, 2)
    hit_pad = np.asarray([pad[i] for i, r in enumerate(scene.rays) if r.status == 'return'], dtype=float)
    if len(rest) and len(hits):
        dist = np.sqrt(((rest[:, None, :]-hits[None, :, :])**2).sum(axis=2))
        surface = np.any(dist <= hit_pad[None, :], axis=1)
    else:
        surface = np.zeros(len(rest), dtype=bool)
    blocked = ~(returned & (surface | (ds > cfg.band_ahead_m)))
    detail.update(region_cells=int(len(region)), possible_cells=int(len(rest)), blocked_cells=int(np.sum(blocked)),
                  grow_m=float(grow), body_cells=int(len(body)))
    # 가려진 영역은 새 반환 근처만 남기는 방식으로 삭제하지 않는다. free가 아닌 영역 전체를
    # 다음 fresh 판단으로 전달한다. '다시 보인 점이 같은 차'라는 식별 전제가 필요하지 않다.
    if len(rest):
        detail['record'] = ThreatRecord(record.side, scene, record.points, record.acquired, rest.copy(),
            old.t_ref_s if record.detected_at_s is None else record.detected_at_s)
    else:
        detail['record'] = None
    detail['ok'] = True   # 영역 갱신 가능. 실제 해제는 아래 blocked/corridor 판정과 별도다.
    if np.any(blocked):
        return {**detail, 'reason': 'threat_region_unobserved'}
    sign = 1. if record.side == 'left' else -1.
    gap = cfg.budget.lateral_clearance(True, False, scene.t_ref_s-scene.source_first_s)[0]
    low = sign*state['d0']-state['half_span_m']
    high = max(sign*state['d0'], 0.)+state['half_span_m']+gap
    corridor = (ds >= -cfg.length_m/2) & (ds <= cfg.length_m/2+stop_distance) & (sign*l >= low) & (sign*l <= high)
    detail.update(ok=True, corridor_cells=int(np.sum(corridor)), stop_distance_m=float(stop_distance))
    if np.any(corridor):
        return {**detail, 'reason': 'threat_possible_in_return_corridor'}
    return {**detail, 'release': True, 'reason': 'threat_region_confirmed'}


class ExecutionLateralTrial:
    """측방 촉발·후보 선택 상태. 촉발·후보 순서는 동결 ObservedLateralTrial과 같고 직진 후보만 추가된다."""

    def __init__(self, cfg, execution, release_speed_mps=1.):
        self.cfg, self.execution = cfg, execution
        self.release_speed_mps = release_speed_mps   # 해제 뒤 기본 G1 속도 상한(정지 거리 계산용)
        self.reset()

    def reset(self):
        self.active = False
        self.clear_fresh = 0
        self.epoch = self.last_ref = self.last_source = None
        self.records = {}   # 측면별 마지막 위협 기록. 해제 선택 옵션에서만 쓴다
        self.release_confirmations = 0
        self.release_history_invalidated = False

    def invalidate_release_history(self):
        """활성 위협의 시간 연결을 잃으면 부재로 해제하지 않는다. 명시 reset 전까지 HOLD한다."""
        self.active = True
        self.release_history_invalidated = True
        self.release_confirmations = 0

    def _release(self, scene, history, frame, state, light, side_masks, present, fresh):
        """위협 기록 갱신과 해제 판단. 출력 상태는 해제 때만 바꾼다."""
        for name, mask in zip(('left', 'right'), side_masks):
            if name in present:
                self.records[name] = threat_record(name, scene, light, mask)
        gates = {}
        if fresh and history is not None:
            stop = envelope_distance(self.release_speed_mps, scene.t_ref_s-scene.source_first_s, .02, self.execution)
            for name, record in list(self.records.items()):
                if name in present or record is None:
                    continue
                gate = release_gate(record, scene, history, frame, state, light, self.cfg, self.execution, stop)
                if gate['ok'] and gate['record'] is not None:
                    self.records[name] = gate['record']
                # 관측 자유로 지워진 기록도 즉시 버리지 않는다. 다음 fresh 스캔에서도 그 영역을
                # 다시 확인해야 두 번의 독립된 양의 관측이 된다. 빈 dict의 all()로 해제하지 않는다.
                gates[name] = {k: v for k, v in gate.items() if k != 'record'}
        pending = [n for n, r in self.records.items() if r is not None and n not in gates]
        confirmed = bool(not present and fresh and history is not None and gates
                         and not pending and all(g['release'] for g in gates.values()))
        if fresh:
            self.release_confirmations = self.release_confirmations+1 if confirmed else 0
        allowed = confirmed and self.release_confirmations >= self.cfg.release_fresh
        if present:
            reason = 'threat_present'
        elif confirmed and not allowed:
            reason = 'debounce'
        else:
            reason = next((g['reason'] for g in gates.values() if not g['release']),
                          'not_fresh_or_no_history' if not (fresh and history is not None) else None)
        summary = {'release': allowed, 'reason': reason, 'sides': gates,
                   'positive_fresh_confirmations': self.release_confirmations,
                   'ok': bool(gates) and all(g['ok'] for g in gates.values())}
        for key in ('blocked_cells', 'possible_cells', 'corridor_cells'):
            summary[key] = int(sum(g.get(key, 0) for g in gates.values()))
        return allowed, summary

    def step(self, scene, frames, path, speed, follow_padding, command_check=None, history=None):
        out = {'runtime_authorized': False, 'hidden_future_risk_unresolved': True,
               'mode': 'OFF', 'selected': None, 'checks': {}, 'path': path}
        cfg = self.cfg
        if not cfg.enabled:
            return out
        if not math.isfinite(speed) or speed < 0 or not source_valid(scene):
            lost = self.execution.release_to_baseline and self.active
            self.reset()
            if lost:
                self.invalidate_release_history()
            return {**out, 'mode': 'HOLD_INPUT', 'path': None}
        if (self.epoch != scene.source_epoch or (self.last_ref is not None and scene.t_ref_s < self.last_ref) or
                (self.last_source is not None and scene.source_first_s < self.last_source)):
            lost = self.execution.release_to_baseline and self.active
            self.reset()
            if lost:
                self.invalidate_release_history()
        fresh = self.last_source is None or scene.source_first_s > self.last_source
        self.epoch, self.last_ref, self.last_source = scene.source_epoch, scene.t_ref_s, scene.source_first_s
        if self.release_history_invalidated:
            return {**out, 'mode': 'HOLD_RELEASE_HISTORY', 'path': None,
                    'release_gate': {'release': False, 'reason': 'release_history_invalidated'}}
        if path is None:
            return {**out, 'mode': 'HOLD_PATH', 'path': None}
        try:
            frame = ExecutionFrame(path)
        except (ValueError, TypeError):
            return {**out, 'mode': 'HOLD_PATH', 'path': None}
        state = ego_state(frame, cfg)
        light = ReturnGrid(scene, cfg)
        _, side_masks = side_return_masks(frame, state, light, cfg)
        if any(np.any(m) for m in side_masks):
            grid = ObservationGrid(scene, cfg)
            s, l = cell_coordinates(grid, frame)
            observed = grid.observed_free()
        else:
            # 측방 반환이 없으면 side_threats는 칸 좌표/상태를 읽지 않는다(같은 결과, 계산 생략).
            grid, s, l, observed = light, None, None, None
        corridor = corridor_reference(frame, grid.returns, state['s0']-cfg.band_behind_m, state['s0']+cfg.horizon_m, cfg)
        threats = side_threats(frame, state, grid, cfg, s, l, observed)
        present = [name for name in ('left', 'right') if threats[name]['present']]
        out.update(state=state, threats=threats)
        if not self.active and not present:
            return {**out, 'mode': 'BASELINE', 'selected': 'baseline'}
        self.active = True
        self.clear_fresh = 0 if present else self.clear_fresh+int(fresh)
        if self.execution.release_to_baseline:
            allowed, out['release_gate'] = self._release(scene, history, frame, state, light, side_masks, present, fresh)
            if allowed:
                self.active = False
                self.clear_fresh = 0
                self.records = {}
                return {**out, 'mode': 'BASELINE', 'selected': 'baseline', 'released_to_baseline': True}
        context = {'frame': frame, 'grid': grid, 'corridor': corridor, 'age': scene.t_ref_s-scene.source_first_s,
                   'follow_padding': follow_padding}
        candidates = {'hold': anchored(offset_candidate(frame, state, state['d0'], cfg, 'hold'))}
        order = []
        if len(present) == 1:
            side = present[0]
            sign = 1. if side == 'left' else -1.
            gap = cfg.budget.lateral_clearance(True, candidates['hold']['curved'], context['age'])[0]
            edge = threats[side]['nearest']['near_edge_l_m']
            target = sign*min(sign*state['d0'], sign*edge-state['half_span_m']-gap)
            limit = cfg.policy_limit_m(candidates['hold']['curved'])['selected_m']
            target = float(np.clip(target, -limit, limit))
            if sign*(target-state['d0']) < -.005:
                candidates['yield'] = anchored(offset_candidate(frame, state, target, cfg, 'yield'))
                order.append('yield')
        elif not present and self.clear_fresh >= cfg.release_fresh:
            candidates['center'] = anchored(offset_candidate(frame, state, 0., cfg, 'center'))
            order.append('center')
            if self.execution.small_center_straight:
                straight = straight_candidate(frame, state, cfg)
                out['small_center'] = small_center_check(straight, cfg)
                if out['small_center']['ok']:
                    candidates['straight'] = straight
                    order.insert(0, 'straight')
        order.append('hold')
        if self.execution.straight_start and 'straight' not in candidates:
            candidates['straight'] = straight_candidate(frame, state, cfg)
            order.append('straight')
        # 계획과 선택형 실제 발행 검사를 모두 통과한 첫 후보에서 멈춘다.
        # command_check는 조향/속도 평가만 하며 재개 카운터·출력 상태를 바꾸지 않는다.
        checks = {}
        out['checks'] = checks
        for key in order:
            checks[key] = plan_check(scene, frames, candidates[key], context, cfg, self.execution, speed)
            if checks[key]['eligible_for_trial']:
                if command_check is not None:
                    checked = command_check(key, candidates[key]['path'])
                    checks[key]['command'] = checked
                    if not checked['accepted']:
                        continue
                if (key == 'center' and abs(state['d0']) < .005
                        and not self.execution.release_to_baseline):
                    self.active = False
                return {**out, 'mode': key.upper()+'_TRIAL', 'selected': key, 'path': candidates[key]['path']}
        return {**out, 'mode': 'HOLD_STOP_TRIAL', 'path': None}


class M3ExecutionFollow:
    """동결 M3ObservedFollow와 같은 G1 관문·0.35 m/s 상한·점유 제동·2 fresh 재개를 유지한 시험판."""

    def __init__(self, config=None, execution=None, *, allow_simulation_trial=False):
        if allow_simulation_trial is not True:
            raise ValueError('M3 requires explicit simulation trial selection')
        self.config = config or FollowConfig(occupancy_timing_mode='per_ray_candidate')
        c = self.config
        if c.max_speed_mps > 1. or c.occupancy_timing_mode != 'per_ray_candidate' or c.recent_observation_enabled:
            raise ValueError('M3 trial requires <=1 m/s, per-ray occupancy, no recent-observation mode')
        self.execution = execution or ExecutionConfig()
        if not isinstance(self.execution, ExecutionConfig):
            raise ValueError('typed execution configuration required')
        self.trial_speed_cap_mps = min(.35, c.max_speed_mps)
        self.lateral_config = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass',
            wheelbase_m=c.wheelbase_m, length_m=c.length_m, width_m=c.width_m, max_steering_rad=c.max_steering_rad)
        if not math.isclose(c.wall_offset_m, self.lateral_config.wall_offset_m, abs_tol=1e-12):
            raise ValueError('M3 requires the registered wall-to-road geometry')
        self.base = LocalFollow(c)
        self.lateral = ExecutionLateralTrial(self.lateral_config, self.execution, c.max_speed_mps)
        self.recent = SceneHistory()
        self.reset()

    def reset(self):
        self.base.reset()
        self.lateral.reset()
        self.recent.reset()
        self.epoch = self.side = self.frame = self.last_ref = self.last_source = None
        self.held = True
        self.release_count = 0

    def _hold(self, reason, detail, steering=0.):
        self.held = True
        self.release_count = 0
        return 0., float(steering), {**detail, 'reason': reason, 'behavior': 'HOLD'}

    def _evaluate_command(self, name, path, scene, frames, history, speed, previous_steering, control_dt,
                          padding, baseline):
        """후보 하나의 실제 조향·중단 외곽을 검사한다. 제어기 상태/재개 횟수는 바꾸지 않는다."""
        cfg = self.config
        age = scene.t_ref_s-scene.source_first_s
        result = {'accepted': False, 'reason': None, 'detail': {}}
        try:
            cap, desired, remaining = pursue_path(path, speed, cfg,
                                                  max(age, baseline[2].get('boundary_age_s') or 0.))
        except MotionUnavailable:
            return {**result, 'reason': 'm3_path_too_short'}
        steering = float(np.clip(desired, previous_steering-3*control_dt, previous_steering+3*control_dt))
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        clearance = min(swept_limit(points, a, cfg.wheelbase_m, horizon=2.5, margin=padding,
                                   length=cfg.length_m, width=cfg.width_m)
                        for a in (steering, previous_steering))
        available = max(0., clearance-cfg.standstill_gap_m)
        target = min(self.trial_speed_cap_mps, cap, braking_speed(available, age), available/cfg.headway_s)
        envelope_speed = max(speed, target)
        distance = envelope_distance(envelope_speed, age, control_dt, self.execution)
        envelope = arc_checks(scene, frames, baseline[2]['path_points'], (steering, previous_steering),
                              distance, self.lateral_config, self.execution)
        detail = dict(m3_selected=True, m3_candidate=name, m3_envelope=envelope,
            path_points=np.asarray(path).tolist(), occupied_points=len(points), unfiltered_occupancy=True,
            swept_clearance_m=clearance, observed_path_ahead_m=remaining, desired_steering_rad=desired,
            envelope_speed_mps=envelope_speed, target_speed_before_hold_mps=target,
            occupancy_padding_min_m=float(np.min(padding)), occupancy_padding_max_m=float(np.max(padding)),
            body_standstill_verified=False)
        result.update(detail=detail, steering=steering, clearance=clearance, target=target, cap=cap)
        if not (envelope['observation_ok'] and envelope['geometry_ok']):
            if (self.execution.straight_start and name == 'straight' and abs(speed) <= STOPPED_SPEED_MPS
                    and steering != previous_steering):
                quiet = presteer_motion_quiet(history, scene.t_ref_s)
                align = presteer_support(scene, frames, (steering, previous_steering), self.lateral_config,
                                         self.execution)
                align['motion_quiet'] = quiet
                detail['m3_presteer'] = align
                if align['supported'] and quiet:
                    return {**result, 'accepted': True, 'reason': 'm3_straight_presteer'}
            reason = 'm3_command_unobserved' if not envelope['observation_ok'] else 'm3_command_geometry_unconfirmed'
            return {**result, 'reason': reason}
        return {**result, 'accepted': True}

    def command(self, scene, history, speed, side, previous_steering=0., control_dt=.02):
        invalid = (side not in ('left', 'right') or not all(math.isfinite(v) for v in
                   (speed, previous_steering, control_dt)) or not 0 < control_dt <= .2)
        changed = (self.epoch != scene.source_epoch or self.side != side or self.frame != scene.source_frame_id or
                   (self.last_ref is not None and scene.t_ref_s < self.last_ref) or
                   (self.last_source is not None and scene.source_first_s is not None and
                    scene.source_first_s < self.last_source))
        if invalid or changed:
            lost = self.execution.release_to_baseline and self.lateral.active
            self.reset()
            if lost:
                self.lateral.invalidate_release_history()
        self.epoch, self.side, self.frame, self.last_ref = scene.source_epoch, side, scene.source_frame_id, scene.t_ref_s
        fresh = self.last_source is None or (scene.source_first_s is not None and scene.source_first_s > self.last_source)
        self.last_source = scene.source_first_s
        # 관측 이력은 기본 G1 관문 결과와 무관하게 매 호출 기록한다(사용은 측방 후보 검사에서만).
        frames = self.recent.update(scene, history) if self.execution.masked_history_support else []
        baseline = self.base.command(scene, history, speed, side, previous_steering, control_dt)
        detail = {**baseline[2], 'mode': 'm3_execution_trial', 'simulation_trial_only': True,
                  'hidden_future_risk_unresolved': True, 'runtime_authorized': False, 'm3_selected': False,
                  'baseline_output': [baseline[0], baseline[1]], 'baseline_reason': baseline[2]['reason']}
        if baseline[2]['reason'] not in ('local_follow', 'follow_gap_hold', 'follow_release_wait',
                                         'local_follow_corridor_candidate'):
            self.held, self.release_count = True, 0
            return baseline[0], baseline[1], detail
        was_active = self.lateral.active
        cfg = self.config
        # 계획 선택은 동결 판본처럼 시험 상한까지의 이동을 가정한다. 점유 팽창은 기존 G1과 같다.
        plan_speed = max(speed, self.trial_speed_cap_mps)
        padding = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
        checked_commands = {}

        def command_check(name, path):
            check = self._evaluate_command(name, path, scene, frames, history, speed, previous_steering,
                                           control_dt, padding, baseline)
            checked_commands[name] = check
            return {'accepted': check['accepted'], 'reason': check['reason']}

        # 운동 이력은 해제 선택 옵션에서만 넘긴다(기본 OFF 호출 형태 유지).
        extra = {'history': history} if self.execution.release_to_baseline else {}
        if self.execution.command_fallback:
            trial = self.lateral.step(scene, frames, baseline[2].get('path_points'), plan_speed, padding, command_check,
                                      **extra)
        else:
            trial = self.lateral.step(scene, frames, baseline[2].get('path_points'), plan_speed, padding, **extra)
        detail['m3_lateral'] = {k: v for k, v in trial.items() if k != 'path'}
        if trial['mode'] == 'BASELINE':
            self.held = baseline[2]['behavior'] == 'HOLD'
            self.release_count = self.base.release_count
            return baseline[0], baseline[1], detail
        if not was_active:
            self.held = baseline[2]['behavior'] == 'HOLD'
            self.release_count = 0
        if trial['path'] is None or trial['selected'] is None:
            if checked_commands:
                first = next(iter(checked_commands.values()))
                detail.update(first['detail'], m3_all_commands_rejected=True)
                return self._hold(first['reason'], detail, previous_steering)
            return self._hold('m3_candidate_unavailable', detail, previous_steering)
        selected = trial['selected']
        checked = checked_commands.get(selected)
        if checked is None:
            checked = self._evaluate_command(selected, trial['path'], scene, frames, history, speed,
                                             previous_steering, control_dt, padding, baseline)
        detail.update(checked['detail'])
        if not checked['accepted']:
            return self._hold(checked['reason'], detail, previous_steering)
        steering, clearance, target, cap = (checked[k] for k in ('steering', 'clearance', 'target', 'cap'))
        if checked['reason'] == 'm3_straight_presteer':
            return self._hold(checked['reason'], detail, steering)
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
        return float(target), steering, {**detail, 'reason': 'm3_execution_trial',
                                         'behavior': 'FOLLOW' if target < cap-1e-6 else 'CRUISE'}
