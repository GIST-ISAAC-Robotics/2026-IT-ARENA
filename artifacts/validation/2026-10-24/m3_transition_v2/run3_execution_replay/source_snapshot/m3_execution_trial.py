"""M3 유한 실행·제동 전이 후보. 명시 선택형 순수 클래스이며 ROS/launch·기본 경로에 연결하지 않는다.

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
   다시 확인한다. 무응답(unknown)·현재 반환 뒤(가림)는 현재 장면만 인정한다. 정지 중에는 이전 장면도
   같은 마스크라 지지하지 못한다. 이동 상계를 넘는 숨은 차량은 미해결이다.
5. **직진 출발(선택):** 현재 진행 방향 직진 후보를 마지막 선택지로 둔다. 정지(단일 모터축 0 측정) 중이고
   직진 후보만 남았을 때 0 목표를 유지한 채 조향만 0으로 정렬한다. 차체 정지 자체는 확인하지 못한다.

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
from arena_autonomy.scene_motion import relative_poses
from arena_autonomy.m3_observed_follow import command_arc

# 경로 좌표 왕복의 수치 차이만 보정한다. 실제 자세 차이를 덮는 허용치가 아니다.
START_TOLERANCE_M = .005
START_TOLERANCE_RAD = .02
# 단일 모터축 측정 0. 바퀴 정지일 뿐 차체 정지 증명이 아니다.
STOPPED_SPEED_MPS = 1e-6


@dataclass(frozen=True)
class ExecutionConfig:
    masked_history_support: bool = True
    straight_start: bool = True
    reaction_s: float = .10          # 기존 보호 반응 상수(local_path.braking_speed)
    deceleration_mps2: float = 2.0   # 기존 보호 감속 상수. 실차 제동 검증값이 아니다
    history_motion_bound_mps: float = .5   # 기존 점유 취득 시차 팽창과 같은 물체 이동 상계

    def __post_init__(self):
        if type(self.masked_history_support) is not bool or type(self.straight_start) is not bool:
            raise ValueError('execution options must be bool')
        for value in (self.reaction_s, self.deceleration_mps2, self.history_motion_bound_mps):
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError('execution constants must be finite and positive')


def envelope_distance(speed, age, control_dt, cfg):
    """이번 주기 이동 + 반응(데이터 나이 포함) + 제동 거리. 기존 braking_speed와 같은 상수."""
    v = abs(speed)
    return v*(control_dt+max(0., age)+cfg.reaction_s)+v*v/(2*cfg.deceleration_mps2)


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
    관계). 여유는 현재 시각 기준 광선 나이(기존 0.5 m/s 이동 상계)로 계산한다.
    """
    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = ray_padding(scene, cfg, execution.history_motion_bound_mps)
    ok = depth_support(scene, q, pad) if len(q) else np.zeros(0, dtype=bool)
    detail = {'queries': int(len(q)), 'current_unsupported': int(np.sum(~ok)), 'masked_candidates': 0,
              'history_supported': 0, 'history_sources': 0}
    if execution.masked_history_support and np.any(~ok) and frames:
        missing = np.flatnonzero(~ok)
        masked = masked_only(scene, q[missing], pad)
        detail['masked_candidates'] = int(masked.sum())
        remaining = missing[masked]
        for old, (dx, dy, yaw) in frames:
            if not len(remaining):
                break
            c, sn = math.cos(yaw), math.sin(yaw)
            local = (q[remaining]-[dx, dy])@np.array([[c, -sn], [sn, c]])
            old_pad = np.array([.025+cfg.budget.range_bias_m+execution.history_motion_bound_mps*(
                scene.t_ref_s-r.acquired_at_s) for r in old.rays])
            accepted = depth_support(old, local, old_pad)
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
    frame = PathFrame(reference_path)
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


def plan_check(scene, frames, candidate, context, cfg, execution, speed):
    """동결 observed_candidate와 같은 계획 조건. 시작 보정·마스크 이력 지지·앞 물체 분담만 다르다."""
    result = {'target_m': candidate['d_target_m'], 'eligible_for_trial': False, 'reasons': [],
              'start_correction_m': candidate.get('start_correction_m'),
              'hidden_future_risk_unresolved': True, 'continuous_sweep_certified': False}
    if not candidate['valid']:
        result['reasons'].append('geometry_invalid')
        result['problems'] = list(candidate['problems'])
        return result
    surface = surface_check(context['frame'], candidate, cfg, current_pose=(0., 0., 0.))
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


def side_points(frame, state, grid, cfg):
    """side_threats와 같은 창·벽·횡위치·compact 조건의 측방 반환 개수(좌, 우)."""
    if not len(grid.returns):
        return 0, 0
    s, l = frame.project(grid.returns)
    ds = s-state['s0']
    window = (ds >= -cfg.band_behind_m) & (ds <= cfg.band_ahead_m)
    wall = np.zeros(len(l), dtype=bool)
    for _, sign in SIDES:
        wall |= np.abs(l-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m
    interior = np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m
    return tuple(int(np.sum(window & interior & ~wall & (sign*(l-state['d0']) >= state['half_span_m'])
                            & grid.compact_returns)) for _, sign in SIDES)


class ExecutionLateralTrial:
    """측방 촉발·후보 선택 상태. 촉발·후보 순서는 동결 ObservedLateralTrial과 같고 직진 후보만 추가된다."""

    def __init__(self, cfg, execution):
        self.cfg, self.execution = cfg, execution
        self.reset()

    def reset(self):
        self.active = False
        self.clear_fresh = 0
        self.epoch = self.last_ref = self.last_source = None

    def step(self, scene, frames, path, speed, follow_padding):
        out = {'runtime_authorized': False, 'hidden_future_risk_unresolved': True,
               'mode': 'OFF', 'selected': None, 'checks': {}, 'path': path}
        cfg = self.cfg
        if not cfg.enabled:
            return out
        if not math.isfinite(speed) or speed < 0 or not source_valid(scene):
            self.reset()
            return {**out, 'mode': 'HOLD_INPUT', 'path': None}
        if (self.epoch != scene.source_epoch or (self.last_ref is not None and scene.t_ref_s < self.last_ref) or
                (self.last_source is not None and scene.source_first_s < self.last_source)):
            self.reset()
        fresh = self.last_source is None or scene.source_first_s > self.last_source
        self.epoch, self.last_ref, self.last_source = scene.source_epoch, scene.t_ref_s, scene.source_first_s
        if path is None:
            return {**out, 'mode': 'HOLD_PATH', 'path': None}
        try:
            frame = PathFrame(path)
        except (ValueError, TypeError):
            return {**out, 'mode': 'HOLD_PATH', 'path': None}
        state = ego_state(frame, cfg)
        light = ReturnGrid(scene, cfg)
        if any(side_points(frame, state, light, cfg)):
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
        order.append('hold')
        if self.execution.straight_start:
            candidates['straight'] = straight_candidate(frame, state, cfg)
            order.append('straight')
        # 우선순위대로 평가하고 첫 통과 후보에서 멈춘다(선택 결과는 전체 평가와 같다).
        checks = {}
        out['checks'] = checks
        for key in order:
            checks[key] = plan_check(scene, frames, candidates[key], context, cfg, self.execution, speed)
            if checks[key]['eligible_for_trial']:
                if key == 'center' and abs(state['d0']) < .005:
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
        self.lateral = ExecutionLateralTrial(self.lateral_config, self.execution)
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

    def command(self, scene, history, speed, side, previous_steering=0., control_dt=.02):
        invalid = (side not in ('left', 'right') or not all(math.isfinite(v) for v in
                   (speed, previous_steering, control_dt)) or not 0 < control_dt <= .2)
        changed = (self.epoch != scene.source_epoch or self.side != side or self.frame != scene.source_frame_id or
                   (self.last_ref is not None and scene.t_ref_s < self.last_ref) or
                   (self.last_source is not None and scene.source_first_s is not None and
                    scene.source_first_s < self.last_source))
        if invalid or changed:
            self.reset()
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
        trial = self.lateral.step(scene, frames, baseline[2].get('path_points'), plan_speed, padding)
        detail['m3_lateral'] = {k: v for k, v in trial.items() if k != 'path'}
        if trial['mode'] == 'BASELINE':
            self.held = baseline[2]['behavior'] == 'HOLD'
            self.release_count = self.base.release_count
            return baseline[0], baseline[1], detail
        if not was_active:
            self.held = baseline[2]['behavior'] == 'HOLD'
            self.release_count = 0
        if trial['path'] is None or trial['selected'] is None:
            return self._hold('m3_candidate_unavailable', detail, previous_steering)
        age = scene.t_ref_s-scene.source_first_s
        try:
            cap, desired, remaining = pursue_path(trial['path'], speed, cfg,
                                                  max(age, baseline[2].get('boundary_age_s') or 0.))
        except MotionUnavailable:
            return self._hold('m3_path_too_short', detail, previous_steering)
        steering = float(np.clip(desired, previous_steering-3*control_dt, previous_steering+3*control_dt))
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        clearance = min(swept_limit(points, a, cfg.wheelbase_m, horizon=2.5, margin=padding,
                                   length=cfg.length_m, width=cfg.width_m)
                        for a in (steering, previous_steering))
        available = max(0., clearance-cfg.standstill_gap_m)
        target = min(self.trial_speed_cap_mps, cap, braking_speed(available, age), available/cfg.headway_s)
        # 이번 명령으로 낼 수 있는 속도 상계: 현재 측정 속도와 이번(재개 시 낼) 목표 중 큰 값.
        envelope_speed = max(speed, target)
        distance = envelope_distance(envelope_speed, age, control_dt, self.execution)
        envelope = arc_checks(scene, frames, baseline[2]['path_points'], (steering, previous_steering),
                              distance, self.lateral_config, self.execution)
        detail.update(m3_selected=True, m3_candidate=trial['selected'], m3_envelope=envelope,
            path_points=np.asarray(trial['path']).tolist(), occupied_points=len(points), unfiltered_occupancy=True,
            swept_clearance_m=clearance, observed_path_ahead_m=remaining, desired_steering_rad=desired,
            envelope_speed_mps=envelope_speed, target_speed_before_hold_mps=target,
            occupancy_padding_min_m=float(np.min(padding)), occupancy_padding_max_m=float(np.max(padding)),
            body_standstill_verified=False)
        if not (envelope['observation_ok'] and envelope['geometry_ok']):
            if (self.execution.straight_start and trial['selected'] == 'straight' and abs(speed) <= STOPPED_SPEED_MPS
                    and steering != previous_steering):
                # 정지 중 직진 출발 준비: 0 목표를 유지하고 조향만 0 쪽으로 옮긴다. 차체는 움직이지 않는다고
                # 가정하므로(단일 모터축 0, 차체 정지 미확인) 현재 자세의 조향 바퀴 외곽 관측만 다시 확인한다.
                align = presteer_support(scene, frames, (steering, previous_steering), self.lateral_config,
                                         self.execution)
                detail['m3_presteer'] = align
                if align['supported']:
                    return self._hold('m3_straight_presteer', detail, steering)
            reason = 'm3_command_unobserved' if not envelope['observation_ok'] else 'm3_command_geometry_unconfirmed'
            return self._hold(reason, detail, previous_steering)
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
