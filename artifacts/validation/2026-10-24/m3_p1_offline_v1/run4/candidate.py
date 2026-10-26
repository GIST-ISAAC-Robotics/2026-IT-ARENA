"""A2 중앙 복귀 보류·P1 법선 오프셋의 오프라인 판별 시제품.

운영 src/launch에 연결하지 않는다. 모든 함수는 명시 인자만 받으며 정답 자세·actor
이름·일정·지도를 받지 않는다. 기존 추종·점유 검사는 src 함수를 그대로 호출한다.
결과는 저장 궤적 위 개루프 판정이며 주행·안전 판정이 아니다.
"""
from dataclasses import dataclass
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import pursue_path
from arena_autonomy.local_path import braking_speed, swept_limit

ROAD_HALF_SPEC = .225   # 공식 본선 0.45 m. 센서 관측이 아니라 트랙 사양이다.


@dataclass(frozen=True)
class A2Config:
    band_behind_m: float = .60
    band_ahead_m: float = .35
    interior_m: float = .345          # wall_offset 0.425 - boundary_agreement 0.08
    min_points: int = 3
    min_offset_m: float = .02
    heading_max_rad: float = math.radians(15)   # 앞 선별의 사후 진단값. 검증 전 설계값 아님
    extension_m: float = .60          # 측방 분류에만 쓰는 경로 후방 선형 연장(P1에는 쓰지 않음)
    p1_check_ahead_m: float = .65
    edge_screen_m: float = .02        # 정적 도로 끝 선별 여유. 채택 값 아님
    release_fresh: int = 2
    lidar_x_m: float = .06
    rear_half_angle_deg: float = 30.
    length_m: float = .20
    width_m: float = .15


# ------------------------------------------------------------------ 경로 좌표
def _frame(path, extension):
    p = np.asarray(path, dtype=float).reshape(-1, 2)
    p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-9]]
    if len(p) < 3:
        return None
    t0 = (p[1]-p[0])/np.linalg.norm(p[1]-p[0])
    t1 = (p[-1]-p[-2])/np.linalg.norm(p[-1]-p[-2])
    q = np.vstack([p[0]-extension*t0, p, p[-1]+extension*t1]) if extension > 0 else p
    seg = np.diff(q, axis=0)
    length = np.linalg.norm(seg, axis=1)
    start = np.r_[0., np.cumsum(length)][:-1]-(extension if extension > 0 else 0.)
    return q, seg, length, start, extension > 0


def project(points, path, extension=0.):
    """(호길이 s, 왼쪽 + 횡좌표 l, 연장 구간 여부, 접선)."""
    frame = _frame(path, extension)
    if frame is None:
        raise ValueError('path too short')
    q, seg, length, start, extended_frame = frame
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    rel = points[:, None, :]-q[None, :-1, :]
    frac = np.clip(np.sum(rel*seg[None], axis=2)/(length**2)[None], 0., 1.)
    foot = q[None, :-1, :]+frac[:, :, None]*seg[None]
    index = np.argmin(np.linalg.norm(points[:, None, :]-foot, axis=2), axis=1)
    rows = np.arange(len(points))
    tangent = seg[index]/length[index][:, None]
    offset = points-foot[rows, index]
    lateral = tangent[:, 0]*offset[:, 1]-tangent[:, 1]*offset[:, 0]
    along = start[index]+frac[rows, index]*length[index]
    extended = ((index == 0) | (index == len(seg)-1)) if extended_frame else np.zeros(len(points), bool)
    return along, lateral, extended, tangent


# ------------------------------------------------------------------ 측후방 상태
def side_state(points, ray_bearings_rad, ray_status, path, cfg=A2Config()):
    """측후방 존재·unknown·창 관측 가능성. 정체·속도를 추정하지 않는다.

    points: base_link 반환 끝점, ray_bearings_rad/ray_status: 센서 방위·상태(전체 광선).
    """
    if path is None:
        return {'status': 'no_path'}
    try:
        along, lateral, extended, _ = project(points, path, cfg.extension_m)
        ego_s, ego_l, _, ego_t = project(np.zeros((1, 2)), path, 0.)
    except ValueError:
        return {'status': 'no_path'}
    ego_s, ego_l = float(ego_s[0]), float(ego_l[0])
    psi = -math.atan2(ego_t[0, 1], ego_t[0, 0])
    half = (cfg.length_m*abs(math.sin(psi))+cfg.width_m*abs(math.cos(psi)))/2
    ds = along-ego_s
    window = (ds >= -cfg.band_behind_m) & (ds <= cfg.band_ahead_m) & (np.abs(lateral) < cfg.interior_m)
    sides = {}
    for name, sign in (('left', 1.), ('right', -1.)):
        outside = window & (sign*(lateral-ego_l) >= half)        # 차체 바깥만 측방으로 센다
        gap = sign*(lateral[outside]-ego_l)-half
        sides[name] = {'points': int(np.sum(outside)),
                       'points_without_extension': int(np.sum(outside & ~extended)),
                       'min_lateral_gap_m': float(np.min(gap)) if len(gap) else None}
    inlane = window & (np.abs(lateral-ego_l) < half)
    bearings = np.degrees(np.asarray(ray_bearings_rad, dtype=float))
    status = np.asarray(ray_status)
    unknown = {'left': int(np.sum((status == 'unknown') & (bearings >= 90) & (bearings <= 150))),
               'right': int(np.sum((status == 'unknown') & (bearings <= -90) & (bearings >= -150)))}
    toward = None if abs(ego_l) < .005 else ('left' if ego_l < 0 else 'right')
    geometry = []
    if abs(psi) > cfg.heading_max_rad:
        geometry.append('heading_exceeds_screen')
    if abs(ego_l)+half > ROAD_HALF_SPEC:
        geometry.append('ego_outside_road_by_wall_offset')
    return {'status': 'path', 'ego_offset_m': ego_l, 'yaw_from_path_rad': psi, 'half_span_m': half,
            'centering_toward': toward, 'sides': sides,
            'inlane_rear_points': int(np.sum(inlane & (ds < 0))), 'inlane_front_points': int(np.sum(inlane & (ds >= 0))),
            'side_rear_unknown': unknown, 'geometry_valid': not geometry, 'geometry_reasons': geometry,
            'window_coverage': window_coverage(cfg, ego_l, half)}


def window_coverage(cfg, ego_l, half):
    """측후방 창이 실제 관측으로 덮이는지. 덮였다고 증명할 수 없으면 unverifiable."""
    reasons = []
    # 창의 뒤쪽 가까운 모서리(차체 바로 옆, 창 뒤끝)가 후방 마스크 안인지 본다(직선 근사).
    for sign in (1., -1.):
        x, y = -cfg.band_behind_m-cfg.lidar_x_m, sign*(half+.01)
        if abs(math.degrees(math.atan2(y, x))) > 180-cfg.rear_half_angle_deg:
            reasons.append('rear_mask_overlap')
            break
    # 최소 물체 폭을 채택하지 않았으므로 광선 사이는 관측되지 않은 공간이다.
    reasons.append('inter_ray_gaps_no_min_object_width')
    return {'status': 'unverifiable', 'reasons': reasons}


# ------------------------------------------------------------------ P1
def _segments_intersect(a, b, c, d):
    def cross(o, p, q):
        return (p[0]-o[0])*(q[1]-o[1])-(p[1]-o[1])*(q[0]-o[0])
    d1, d2, d3, d4 = cross(c, d, a), cross(c, d, b), cross(a, b, c), cross(a, b, d)
    return (d1*d2 < 0) and (d3*d4 < 0)


def p1_path(path, offset_m, half_span_m, cfg=A2Config()):
    """기본 경로의 곡선별 법선 오프셋. 외삽·관측 지지 추가 없음.

    반환의 `valid`는 기하 무결성(역전·접힘·자기교차·길이)과 정적 도로 끝 조건만 뜻한다.
    차체 swept footprint의 도로 포함은 계산하지 않으므로 항상 `unverified`다.
    """
    p = np.asarray(path, dtype=float).reshape(-1, 2)
    p = p[np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-9]]
    out = {'support': 'inherits_base_path_arc_only', 'swept_footprint_in_road': 'unverified'}
    if len(p) < 4:
        return {**out, 'valid': False, 'reason': 'base_path_short', 'path': None}
    tangent = np.gradient(p, axis=0)
    tangent /= np.maximum(1e-12, np.linalg.norm(tangent, axis=1))[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))      # 왼쪽 법선
    shifted = p+offset_m*normal
    step = np.diff(shifted, axis=0)
    reversal = np.flatnonzero(np.sum(step*(tangent[1:]+tangent[:-1]), axis=1) <= 0.)
    # 국소 곡률로 본 이론적 접힘(1-κ·d ≤ 0)
    dt = np.gradient(tangent, axis=0)
    ds = np.maximum(1e-9, np.linalg.norm(np.gradient(p, axis=0), axis=1))
    kappa = (tangent[:, 0]*dt[:, 1]-tangent[:, 1]*dt[:, 0])/ds
    fold_points = np.flatnonzero(1.-kappa*offset_m <= 0.)
    s, _, _, _ = project(np.zeros((1, 2)), shifted, 0.) if len(shifted) >= 3 else (np.array([0.]), 0, 0, 0)
    arc = np.r_[0., np.cumsum(np.linalg.norm(step, axis=1))]
    ego_s = float(s[0])
    check = np.flatnonzero((arc >= ego_s-1e-9) & (arc <= ego_s+cfg.p1_check_ahead_m))
    lo, hi = (int(check[0]), int(check[-1])) if len(check) else (0, -1)
    intersections = [(i, j) for i in range(len(step)) for j in range(i+2, len(step))
                     if _segments_intersect(shifted[i], shifted[i+1], shifted[j], shifted[j+1])]
    in_range = lambda idx: lo <= idx <= hi
    problems = []
    if arc[-1]-ego_s < cfg.p1_check_ahead_m:
        problems.append('short_ahead')
    if any(in_range(i) for i in reversal):
        problems.append('direction_reversal')
    if any(in_range(i) for i in fold_points):
        problems.append('curvature_fold')
    if any(in_range(i) or in_range(j) for i, j in intersections):
        problems.append('self_intersection')
    static_road = abs(offset_m)+half_span_m <= ROAD_HALF_SPEC-cfg.edge_screen_m
    if not static_road:
        problems.append('static_edge_screen')
    return {**out, 'valid': not problems, 'reason': problems[0] if problems else 'ok', 'problems': problems,
            'reversal_segments': len(reversal), 'fold_points': len(fold_points),
            'intersections': len(intersections), 'static_edge_ok': static_road,
            'offset_curve': shifted, 'path': shifted if not problems else None}


# ------------------------------------------------------------------ 기존 검사 적용
def existing_checks(path, points, margin, speed, previous_steering, age, follow_cfg, control_dt=.02):
    """G1 `_baseline_command`의 경로 이후 계산과 같은 함수·순서. 상태(HOLD 이력)는 제외."""
    try:
        cap, desired, remaining = pursue_path(path, speed, follow_cfg, age)
    except MotionUnavailable as error:
        return {'status': str(error)}
    steering = float(np.clip(desired, previous_steering-3*control_dt, previous_steering+3*control_dt))
    clearance = min(swept_limit(points, a, follow_cfg.wheelbase_m, horizon=2.5, margin=margin,
                                length=follow_cfg.length_m, width=follow_cfg.width_m)
                    for a in (steering, previous_steering))
    available = max(0., clearance-follow_cfg.standstill_gap_m)
    target = min(cap, braking_speed(available, age), available/follow_cfg.headway_s)
    return {'status': 'ok', 'cap_mps': cap, 'desired_steering_rad': desired, 'steering_rad': steering,
            'swept_clearance_m': clearance, 'target_mps': target,
            'gap_hold': clearance <= follow_cfg.hold_clearance_m, 'observed_path_ahead_m': remaining}


# ------------------------------------------------------------------ A2 상태기계(기록 비교용)
class A2Machine:
    """P0/P1 선택과 해제·무효 전이 기록. 해제는 증명 불가이므로 release_unverified로만 표시한다."""

    def __init__(self, cfg=A2Config()):
        self.cfg = cfg
        self.reset()

    def reset(self):
        self.mode, self.side, self.absent = 'P0', None, 0
        self.offset = None

    def step(self, state, p1_valid, fresh):
        events = []
        cfg = self.cfg
        if state.get('status') != 'path':
            if self.mode == 'P1':
                events.append('invalid_transition:no_path')
            self.reset()
            return 'NO_PATH', events
        toward = state['centering_toward']
        present = {k: state['sides'][k]['points'] >= cfg.min_points for k in ('left', 'right')}
        if self.mode == 'P0':
            if (state['geometry_valid'] and toward and present[toward]
                    and abs(state['ego_offset_m']) >= cfg.min_offset_m):
                if p1_valid:
                    self.mode, self.side, self.absent = 'P1', toward, 0
                    self.offset = state['ego_offset_m']
                    events.append('trigger')
                else:
                    events.append('trigger_but_p1_invalid')
            return self.mode, events
        # P1 활성 중
        if not state['geometry_valid'] or not p1_valid:
            events.append('invalid_transition:'+('geometry' if not state['geometry_valid'] else 'p1'))
            self.reset()
            return 'P0', events
        if present[self.side]:
            self.absent = 0
            return 'P1', events
        if state['side_rear_unknown'][self.side] > 0:
            events.append('release_blocked_unknown')
            self.absent = 0
            return 'P1', events
        if fresh:
            self.absent += 1
        if self.absent >= cfg.release_fresh:
            # 창 관측은 항상 unverifiable(마스크·광선 사이) → 해제는 증명되지 않은 해제다.
            events.append('release_unverified')
            self.reset()
            return 'P0', events
        return 'P1', events


def toward_threat(steering_rad, side, deadband=.01):
    if side is None:
        return False
    return steering_rad > deadband if side == 'left' else steering_rad < -deadband


class SteeringChain:
    """후보 자신의 직전 조향(P1 활성 중에만).

    리셋·경로 소실·입력 오류·재촉발 뒤에는 그 판단에서 관측된(기록된) 직전 조향부터
    다시 시작한다. 이전 후보 조향을 재사용하지 않는다.
    """

    def __init__(self):
        self.value = None

    def reset(self):
        self.value = None

    def previous(self, mode, events, recorded):
        if mode != 'P1' or 'trigger' in events or self.value is None:
            return recorded
        return self.value

    def update(self, mode, steering):
        self.value = steering if mode == 'P1' else None
