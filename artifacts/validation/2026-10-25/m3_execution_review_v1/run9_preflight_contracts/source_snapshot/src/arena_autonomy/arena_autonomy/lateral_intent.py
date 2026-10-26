"""선택형 측방 의도 제한기(M3 후보 v1). 기본 꺼짐이며 운영 노드·launch에 연결하지 않는다.

10/24 검토: 이 판본의 상대 횡외곽/숨은 물체 도달은 좌표·시간 일관성이 미완료다.
`verified`는 이 시제품의 검사 결과일 뿐 주행 허가가 아니다. 동일 시각 진단은
`lateral_sweep_review`로 분리했으며, 검토를 마치기 전 이 모듈을 구동에 연결하지 않는다.

입력은 순차 시각으로 운동 보정한 `LocalScene`(모든 반환·unknown·후방 마스크 보존), 기존 경계
절차가 확정한 기본 경로 P0(base_link), 자차 속도뿐이다. 지도·scene·actor 정답과 정답 지면
(도로/잔디 다각형)은 받지 않는다.

- 도로 끝은 LiDAR가 직접 보지 못한다. 관측한 벽에서 공식 잔디 폭 0.20 m를 뺀 **사양 추정**이며,
  벽 연속성·양쪽 폭 일치를 확인한 구간에서만 잔디 쪽 바퀴 진입을 허용한다. 개구부·갈림길처럼
  벽이 끊기거나 어긋나면 그쪽 잔디 사용은 검증되지 않은 것으로 남긴다.
- 횡이동은 새로 차지할 영역이 실제 광선으로 비어 있음을 확인하고(unknown·마스크는 자유 공간이
  아니다), 보이지 않는 영역에 숨을 수 있는 차량이 그 전에 도달할 수 있는지를 명시한 상계로 본다.
- 측방 간격은 종방향 제동 간격과 별개다. 거리 오차·상대 횡운동·추종 오차를 `LateralBudget`에
  분리해 적는다. 15 cm 과거 비교선은 여기서 쓰지 않으며 판정을 대신하지도 않는다.
- 출력은 경로 선택·근거 기록이다. 종방향 속도·HOLD 이력·독립 보호층·STOP은 기존대로 적용한다.
"""
from dataclasses import dataclass, asdict, field, replace
import math

import numpy as np

UNOBS, FREE, OCC, EGO = 0, 1, 2, 3
SIDES = (('left', 1.), ('right', -1.))
# 닫힌 차체 경계에 있는 격자 중심의 부동소수점 오차만 처리한다(0.1 nm).
# 1 cm 격자 해상도/센서 오차/차량 간격을 완화하는 값이 아니다.
GEOMETRY_EPS_M = 1e-10


@dataclass(frozen=True)
class LateralBudget:
    """측방 오차 예산. 각 값의 근거는 설계 문서에 둔다. 실측이 아닌 값은 실물 자료로 교체한다."""
    range_bias_m: float = .030             # C1 공식 거리 정확도 ±30 mm. 분포 미확정 → 치우침으로 선형 합산
    grass_width_tolerance_m: float = .010  # 공식 잔디 0.20 m의 시공 편차 가정(시뮬 상자 근사 0.194–0.208)
    tracking_straight_m: float = .022      # 관측된 0.019–0.021 m 최대치를 올림. 실차 오차 보장이 아님
    tracking_curve_m: float = .040         # 같은 기록 곡선 최대 0.036–0.038 m
    timing_m: float = 0.                   # 시뮬은 점별 시각 검증. 실물 C1 시각 계약 해결 전 재설정 필요
    slip_m: float = .010                   # 잔디·도로 마찰 차이 횡미끄럼 가정(실측 없음)
    heading_lever_m: float = .002          # 벽 방향 추정 약 1.7° × 바퀴 종위치 0.0675 m
    other_lateral_speed_mps: float = .15   # 상대차 횡속도 상계 가정(1–2 m/s에서 진행각 ±5–8°)
    observation_age_s: float = .12         # 10 Hz 스캔 + 처리
    reaction_s: float = .10                # 기존 보호층 반응 상수

    def __post_init__(self):
        if any(isinstance(v, bool) or not math.isfinite(v) or v < 0 for v in asdict(self).values()):
            raise ValueError('lateral budget values must be finite and nonnegative')

    def dynamic_m(self, curved=False):
        tracking = self.tracking_curve_m if curved else self.tracking_straight_m
        # 최대 편차와 가정 상계의 분포/독립성이 없으므로 RSS로 줄이지 않는다.
        return tracking+self.timing_m+self.slip_m+self.heading_lever_m

    def edge_margin(self, curved=False):
        """추정 도로 끝과 바퀴/차체 사이 여유. 모든 오차항을 선형 합산한다."""
        bias = self.range_bias_m+self.grass_width_tolerance_m
        dynamic = self.dynamic_m(curved)
        return bias+dynamic, {'range_bias_m': self.range_bias_m, 'grass_width_tolerance_m': self.grass_width_tolerance_m,
                              'dynamic_bound_sum_m': dynamic, 'curved': curved}

    def lateral_clearance(self, moving=True, curved=False, age_s=None):
        """관측한 다른 물체와의 측방 간격 요구. moving=False는 정지가 확인된 물체에만 쓴다."""
        age = self.observation_age_s if age_s is None else age_s
        if isinstance(age, bool) or not math.isfinite(age) or age < 0:
            raise ValueError('observation age must be finite and nonnegative')
        motion = self.other_lateral_speed_mps*(age+self.reaction_s) if moving else 0.
        dynamic = self.dynamic_m(curved)
        return self.range_bias_m+motion+dynamic, {'range_bias_m': self.range_bias_m, 'other_motion_m': motion,
                                                  'dynamic_bound_sum_m': dynamic, 'moving': moving, 'curved': curved}


@dataclass(frozen=True)
class LateralIntentConfig:
    enabled: bool = False
    surface_policy: str = 'road_only'            # 'road_only' | 'two_wheel_grass'(사용자 10/24 재승인, 선택형)
    budget: LateralBudget = field(default_factory=LateralBudget)
    length_m: float = .20
    width_m: float = .15
    wheelbase_m: float = .135
    track_m: float = .135
    wheel_radius_m: float = .0325
    wheel_width_m: float = .012
    contact_patch_length_m: float = .010         # 평가기와 같은 미실측 접지 길이 가정
    road_half_m: float = .225                    # 공식 본선 0.45 m(사양, 관측 아님)
    grass_width_m: float = .20                   # 공식 잔디 완충 0.20 m(사양, 관측 아님)
    blend_min_m: float = .60
    blend_per_offset: float = 6.0                # 최대 기울기 1.5/6 → 약 14°. yaw에 의한 반폭 증가를 줄인다(§7.3)
    max_steering_rad: float = .37                # G1 명령 상한. 후보의 필요 조향이 넘으면 무효
    horizon_m: float = 1.2
    min_ahead_m: float = .65
    curved_curvature_m_inv: float = .5
    band_behind_m: float = .60
    band_ahead_m: float = .35
    min_threat_points: int = 3
    wall_tolerance_m: float = .03
    wall_bin_m: float = .05
    wall_min_coverage: float = .8
    wall_max_gap_m: float = .10
    opposite_min_coverage: float = .3
    grid_resolution_m: float = .01
    grid_x_m: tuple = (-1.0, 2.0)
    grid_y_m: tuple = (-.8, .8)
    hidden_vehicle_width_m: float = .10          # 숨을 수 있는 상대차 최소 폭 가정(규정 최대 0.15)
    hidden_vehicle_length_m: float = .15         # 숨을 수 있는 상대차 최소 길이 가정(규정 최대 0.20)
    hidden_relative_speed_mps: float = .5        # 숨은 차량의 자차 대비 종방향 상대속도 상계(기존 시험 가정)
    hidden_lateral_speed_mps: float = .15        # 숨은 차량 횡속도 상계(budget.other_lateral_speed와 같은 가정)
    commit_horizon_s: float = .50                # 관측 나이 + 반응 0.10 + 횡운동 정지 0.30
    planned_speed_floor_mps: float = .20
    release_fresh: int = 2
    grass_max_continuous_s: float = 3.0          # 팀 전략 매개변수(공식 '장시간' 기준 아님)
    # 이전 soft_after_quiet 시제품은 검토에서 반려했다. 경과 시간으로 미관측을 해제하지 않는다.
    recenter_hidden_policy: str = 'hard'
    max_vehicle_width_m: float = .15             # 공식 차량 규격 최대 폭(보이는 바깥 끝에서 가까운 면 한계)
    max_vehicle_length_m: float = .20
    compact_point_motion_bound_mps: float = .5   # 반환 묶음 분류용 이동 가정. 차량 의미 인식의 보장이 아님

    def __post_init__(self):
        if self.surface_policy not in ('road_only', 'two_wheel_grass'):
            raise ValueError('invalid surface policy')
        if self.recenter_hidden_policy != 'hard':
            raise ValueError('invalid recenter hidden policy')
        if type(self.enabled) is not bool:
            raise ValueError('enabled must be bool')
        numbers = {k: v for k, v in asdict(self).items()
                   if k not in ('enabled', 'surface_policy', 'budget', 'grid_x_m', 'grid_y_m', 'recenter_hidden_policy')}
        if any(isinstance(v, bool) or not math.isfinite(v) or v <= 0 for v in numbers.values()):
            raise ValueError('lateral intent configuration must be finite and positive')
        if self.track_m+self.wheel_width_m > self.width_m or self.wheelbase_m+2*self.wheel_radius_m > self.length_m+1e-9:
            raise ValueError('wheels must fit inside the registered body footprint')
        if not isinstance(self.budget, LateralBudget):
            raise ValueError('typed lateral budget required')
        for extent in (self.grid_x_m, self.grid_y_m):
            if len(extent) != 2 or not all(math.isfinite(v) for v in extent) or extent[0] >= extent[1]:
                raise ValueError('ordered finite grid bounds required')
        if any(type(v) is not int for v in (self.min_threat_points, self.release_fresh)):
            raise ValueError('integer observation counts required')
        cells = math.ceil((self.grid_x_m[1]-self.grid_x_m[0])/self.grid_resolution_m)*math.ceil(
            (self.grid_y_m[1]-self.grid_y_m[0])/self.grid_resolution_m)
        if cells > 1000000:
            raise ValueError('observation grid exceeds bounded capacity')

    @property
    def wall_offset_m(self):
        return self.road_half_m+self.grass_width_m

    def policy_limit_m(self, curved=False):
        """직진 자세(yaw 0) 기준 중심 횡위치 한계. 정책별 공간 예산 비교용."""
        margin = self.budget.edge_margin(curved)[0]
        body = self.road_half_m-self.width_m/2-margin
        grass = self.road_half_m+self.track_m/2-self.wheel_width_m/2-margin
        return {'road_only_m': body, 'two_wheel_grass_m': grass, 'edge_margin_m': margin,
                'selected_m': grass if self.surface_policy == 'two_wheel_grass' else body}


# ------------------------------------------------------------------ 기본 경로 좌표
class PathFrame:
    """기본 경로의 호길이 s·왼쪽 + 횡좌표 l. 투영은 양 끝 접선으로 연장하지만 경로 생성은 외삽하지 않는다."""

    def __init__(self, path):
        p = np.asarray(path, dtype=float).reshape(-1, 2)
        if len(p) and not np.all(np.isfinite(p)):
            raise ValueError('nonfinite path')
        keep = np.r_[True, np.linalg.norm(np.diff(p, axis=0), axis=1) > 1e-9] if len(p) else []
        p = p[keep]
        if len(p) < 4:
            raise ValueError('path too short')
        self.p = p
        seg = np.diff(p, axis=0)
        self.length = np.linalg.norm(seg, axis=1)
        self.tangent = seg/self.length[:, None]
        self.s = np.r_[0., np.cumsum(self.length)]
        heading = np.unwrap(np.arctan2(self.tangent[:, 1], self.tangent[:, 0]))
        mids = (self.s[:-1]+self.s[1:])/2
        self._heading_s, self._heading = mids, heading
        self._curv = np.gradient(heading, mids) if len(mids) > 1 else np.zeros(1)

    @property
    def end_s(self):
        return float(self.s[-1])

    def project(self, points):
        q = np.asarray(points, dtype=float).reshape(-1, 2)
        a = self.p[:-1]
        rel = q[:, None, :]-a[None]
        t = np.sum(rel*self.tangent[None], axis=2)
        n = len(self.length)
        lo = np.where(np.arange(n) == 0, -np.inf, 0.)
        hi = np.where(np.arange(n) == n-1, np.inf, self.length)
        tc = np.clip(t, lo[None], hi[None])
        foot = a[None]+tc[:, :, None]*self.tangent[None]
        dist = np.linalg.norm(q[:, None, :]-foot, axis=2)
        i = np.argmin(dist, axis=1)
        rows = np.arange(len(q))
        tan = self.tangent[i]
        off = q-foot[rows, i]
        return self.s[i]+tc[rows, i], tan[:, 0]*off[:, 1]-tan[:, 1]*off[:, 0]

    def heading(self, s):
        return np.interp(s, self._heading_s, self._heading)

    def curvature(self, s):
        return np.interp(s, self._heading_s, self._curv)

    def pose(self, s, l):
        """(x, y, 경로 접선 방향). 경로 밖 s는 끝 접선으로 연장(투영·검사용)."""
        s = np.atleast_1d(np.asarray(s, dtype=float))
        l = np.broadcast_to(np.asarray(l, dtype=float), s.shape)
        i = np.clip(np.searchsorted(self.s, s, side='right')-1, 0, len(self.length)-1)
        base = self.p[i]+(s-self.s[i])[:, None]*self.tangent[i]
        h = self.heading(s)
        normal = np.column_stack((-np.sin(h), np.cos(h)))
        return np.column_stack((base+l[:, None]*normal, h))


def rectangle_corners(poses, length, width):
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    local = np.array([[-length/2, -width/2], [length/2, -width/2], [length/2, width/2], [-length/2, width/2]])
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    x = poses[:, None, 0]+local[None, :, 0]*c[:, None]-local[None, :, 1]*s[:, None]
    y = poses[:, None, 1]+local[None, :, 0]*s[:, None]+local[None, :, 1]*c[:, None]
    return np.stack((x, y), axis=2)


def wheel_points(poses, cfg):
    """앞왼·앞오·뒤왼·뒤오 바퀴 중심(n,4,2)."""
    a, b = cfg.wheelbase_m/2, cfg.track_m/2
    local = np.array([[a, b], [a, -b], [-a, b], [-a, -b]])
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    x = poses[:, None, 0]+local[None, :, 0]*c[:, None]-local[None, :, 1]*s[:, None]
    y = poses[:, None, 1]+local[None, :, 0]*s[:, None]+local[None, :, 1]*c[:, None]
    return np.stack((x, y), axis=2)


def wheel_contact_corners(poses, steering, cfg):
    """네 바퀴 접지 사각형. 중앙 등가 조향을 Ackermann 좌우 바퀴 각도로 변환한다."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    delta = np.broadcast_to(np.asarray(steering, dtype=float), (len(poses),))
    k = np.tan(delta)/cfg.wheelbase_m
    angles = np.zeros((len(poses), 4))
    for i, side in enumerate((1., -1.)):
        angles[:, i] = np.arctan2(cfg.wheelbase_m*k, 1.-side*cfg.track_m*k/2)
    # 뒤 바퀴는 차체와 같은 방향. 접지 길이 10 mm는 평면 시뮬 모델용 가정이다.
    headings = poses[:, 2, None]+angles
    half_x, half_y = cfg.contact_patch_length_m/2, cfg.wheel_width_m/2
    local = np.array([[-half_x, -half_y], [half_x, -half_y], [half_x, half_y], [-half_x, half_y]])
    c, s = np.cos(headings), np.sin(headings)
    centers = wheel_points(poses, cfg)
    x = centers[:, :, 0, None]+c[:, :, None]*local[:, 0]-s[:, :, None]*local[:, 1]
    y = centers[:, :, 1, None]+s[:, :, None]*local[:, 0]+c[:, :, None]*local[:, 1]
    return np.stack((x, y), axis=-1)


def steering_protrusion_m(steering_rad, cfg):
    """조향한 앞바퀴 외곽이 15 cm 차체 밖으로 나오는 양(바퀴 중심 조향축 가정)."""
    d = abs(float(steering_rad))
    if d == 0.:
        return max(0., (cfg.track_m+cfg.wheel_width_m-cfg.width_m)/2)
    # 중앙 등가 조향을 그대로 양 바퀴에 적용하면 안쪽 Ackermann 바퀴의 돌출을 작게 센다.
    radius = cfg.wheelbase_m/math.tan(d)
    angles = [math.atan2(cfg.wheelbase_m, radius-sign*cfg.track_m/2) for sign in (-1., 1.)]
    # 조향 전이 도중 가장 넓어지는 각도까지 포함한다.
    maximum = math.atan2(2*cfg.wheel_radius_m, cfg.wheel_width_m)
    half_spans = [cfg.wheel_radius_m*math.sin(min(a, maximum))+
                  cfg.wheel_width_m/2*math.cos(min(a, maximum)) for a in angles]
    return max(0., cfg.track_m/2+max(half_spans)-cfg.width_m/2)


def ego_state(frame, cfg):
    """자차 원점·후륜축의 기본 경로 좌표. Pure Pursuit는 후륜축이 경로를 따르므로 전이는 후륜축 기준이다."""
    s, l = frame.project(np.array([[0., 0.], [-cfg.wheelbase_m/2, 0.]]))
    psi = -float(frame.heading(s[:1])[0])
    psi = math.remainder(psi, math.tau)
    half = (cfg.length_m*abs(math.sin(psi))+cfg.width_m*abs(math.cos(psi)))/2
    return {'s0': float(s[0]), 'd0': float(l[0]), 's_rear0': float(s[1]), 'd_rear0': float(l[1]),
            'psi': psi, 'half_span_m': half}


# ------------------------------------------------------------------ 후보 경로
def _segments_cross(p):
    a, b = p[:-1], p[1:]
    hits = 0
    for i in range(len(a)):
        j = np.arange(i+2, len(a))
        if not len(j):
            continue
        def cross(o, u, v):
            return (u[..., 0]-o[..., 0])*(v[..., 1]-o[..., 1])-(u[..., 1]-o[..., 1])*(v[..., 0]-o[..., 0])
        d1, d2 = cross(a[j], b[j], a[i]), cross(a[j], b[j], b[i])
        d3, d4 = cross(a[i], b[i], a[j]), cross(a[i], b[i], b[j])
        hits += int(np.sum((d1*d2 < 0) & (d3*d4 < 0)))
    return hits


def offset_candidate(frame, state, d_target, cfg, name='candidate'):
    """현재 후륜축 위치와 접선에서 목표 법선 오프셋까지 Hermite 전이하는 경로.

    `path`는 후륜축이 따르는 경로(기존 `pursue_path` 입력과 같은 의미), `poses`는 그때의 차체 원점
    자세(후륜축 + 축거/2 앞, 방향 = 실제 오프셋 경로 접선)다. 경로와 무관한 yaw 감쇠를 더하지 않는다.
    """
    s0, d0 = state['s_rear0'], state['d_rear0']
    delta = d_target-d0
    blend = max(cfg.blend_min_m, cfg.blend_per_offset*abs(delta))
    end = min(frame.end_s, s0+cfg.horizon_m)
    problems = []
    if end-state['s0'] < cfg.min_ahead_m:
        problems.append('short_ahead')
    s = np.arange(s0, max(end, s0+.05)+1e-9, .025)
    u = np.clip((s-s0)/blend, 0., 1.)
    kappa = frame.curvature(s)
    rear_heading = float(frame.heading(np.array([s0]))[0])
    initial_psi = math.remainder(-rear_heading, math.tau)
    if abs(initial_psi) >= math.pi/3:
        problems.append('initial_heading_infeasible')
    initial_slope = (1.-kappa[0]*d0)*math.tan(initial_psi)
    d = d0+delta*(3*u*u-2*u**3)+blend*initial_slope*u*(1-u)**2
    slope = delta*(6*u-6*u*u)/blend+initial_slope*(1-4*u+3*u*u)
    rear = frame.pose(s, d)
    path_heading = rear[:, 2].copy()
    heading = path_heading+np.arctan2(slope, 1.-kappa*d)
    lr = cfg.wheelbase_m/2
    poses = np.column_stack((rear[:, 0]+lr*np.cos(heading), rear[:, 1]+lr*np.sin(heading), heading))
    if np.any(1.-kappa*d <= .2):
        problems.append('curvature_fold')
    step = np.diff(rear[:, :2], axis=0)
    tangent = np.column_stack((np.cos(path_heading), np.sin(path_heading)))
    if np.any(np.sum(step*tangent[:-1], axis=1) <= 0.):
        problems.append('direction_reversal')
    if len(rear) > 3 and _segments_cross(rear[:, :2]):
        problems.append('self_intersection')
    # 실제 후보 접선의 변화율로 필요 조향을 구한다. 기본 경로의 조향 포화도 면제하지 않는다.
    path_heading_total = np.unwrap(heading)
    ds = np.maximum(1e-6, np.linalg.norm(np.gradient(rear[:, :2], axis=0), axis=1))
    curvature = np.gradient(path_heading_total)/ds
    steering = np.arctan(cfg.wheelbase_m*curvature)
    if np.max(np.abs(steering)) > cfg.max_steering_rad+1e-9:
        problems.append('steering_infeasible')
    return {'name': name, 'd_target_m': float(d_target), 'delta_m': float(delta), 'blend_m': blend,
            'progress': np.r_[0., np.cumsum(np.linalg.norm(np.diff(rear[:, :2], axis=0), axis=1))],
            'offset': d, 'poses': poses, 'path': rear[:, :2], 'path_heading': path_heading,
            'max_abs_curvature_m_inv': float(np.max(np.abs(curvature))),
            'max_abs_steering_rad': float(np.max(np.abs(steering))),
            'steering_rad': steering,
            'curved': bool(np.max(np.abs(kappa)) > cfg.curved_curvature_m_inv),
            'valid': not problems, 'problems': problems}


# ------------------------------------------------------------------ 노면(추정) 정책
def surface_check(frame, candidate, cfg, current_pose=None):
    """추정 도로 끝 대비 바퀴·차체 분류. 정답 지면을 쓰지 않는다.

    도로만 정책: 차체 외형이 추정 도로 끝-여유 안. 현재 자세가 이미 여유를 못 지키면 그보다
    나빠지지 않는 후보만 허용한다(단조 비악화). 두 바퀴 잔디 정책은 도로만 정책의 상위 집합이다.
    도로만 정책으로 허용되지 않을 때만 한쪽 바퀴 쌍의 미확인(잔디 가능)을 허용하며, 다른 쪽 쌍은
    확인된 도로여야 하고, 그 쪽 도로 끝 근거(관측 벽)가 호출자에서 별도로 확인되어야 한다.
    """
    margin, margin_detail = cfg.budget.edge_margin(candidate['curved'])
    poses = candidate['poses']
    steering = np.asarray(candidate.get('steering_rad', np.zeros(len(poses))))
    if current_pose is not None:
        poses = np.vstack((np.asarray(current_pose, dtype=float).reshape(1, 3), poses))
        steering = np.r_[steering[0], steering]
    wheels = wheel_points(poses, cfg).reshape(-1, 2)
    ws, wl = frame.project(wheels)
    wl = wl.reshape(-1, 4)
    ws = ws.reshape(-1, 4)
    contact = wheel_contact_corners(poses, steering, cfg)
    _, contact_l = frame.project(contact.reshape(-1, 2))
    contact_l = contact_l.reshape(len(poses), 4, 4)
    edge = cfg.road_half_m
    inner_ok = np.max(np.abs(contact_l), axis=2) <= edge-margin+GEOMETRY_EPS_M
    beyond = ((contact_l >= edge+margin).all(axis=2) | (contact_l <= -edge-margin).all(axis=2))
    corners = rectangle_corners(poses, cfg.length_m, cfg.width_m).reshape(-1, 2)
    _, cl = frame.project(corners)
    body_excess = np.max(np.abs(cl.reshape(-1, 4)), axis=1)+margin-edge   # >0이면 도로만 정책 여유 부족
    sides = np.sign(wl)
    rows = slice(1, None) if current_pose is not None else slice(None)
    base = body_excess[0] if current_pose is not None else 0.
    road_ok = (base <= margin+1e-9 and
               bool(np.all(body_excess[rows] <= max(0., base)+1e-9)))
    unconfirmed = ~inner_ok
    pair_left = (unconfirmed[:, [0, 2]].any(axis=1) & (sides[:, [0, 2]] > 0).all(axis=1))[rows]
    pair_right = (unconfirmed[:, [1, 3]].any(axis=1) & (sides[:, [1, 3]] < 0).all(axis=1))[rows]
    left_inner_ok = inner_ok[rows][:, [0, 2]].all(axis=1)
    right_inner_ok = inner_ok[rows][:, [1, 3]].all(axis=1)
    ok_rows = (~unconfirmed[rows].any(axis=1)) | (pair_left & right_inner_ok) | (pair_right & left_inner_ok)
    grass_sides = [n for n, flag in (('left', pair_left.any()), ('right', pair_right.any())) if flag]
    two_wheel_ok = bool(np.all(ok_rows)) and len(grass_sides) <= 1
    s_unconfirmed = ws[rows][unconfirmed[rows]]
    result = {'edge_margin_m': margin, 'edge_margin_detail': margin_detail, 'wheel_s': ws, 'wheel_l': wl,
              'policy': cfg.surface_policy, 'road_only_ok': road_ok, 'two_wheel_ok': two_wheel_ok,
              'max_body_excess_m': float(np.max(body_excess[rows])), 'current_body_excess_m': float(base),
              'outer_unconfirmed_samples': int(np.sum(pair_left | pair_right)),
              'outer_beyond_samples': int(np.sum(beyond[rows].any(axis=1))),
              's_range_needing_edge': (float(np.min(s_unconfirmed)), float(np.max(s_unconfirmed)))
              if len(s_unconfirmed) else None}
    if cfg.surface_policy == 'road_only' or road_ok:
        result.update(ok=road_ok, grass_side=None,
                      reason=None if road_ok else 'body_beyond_inferred_road_margin')
        return result
    result.update(ok=two_wheel_ok, grass_side=grass_sides[0] if len(grass_sides) == 1 else None,
                  grass_sides=grass_sides,
                  reason=None if two_wheel_ok else ('both_sides_unconfirmed' if len(grass_sides) > 1
                                                    else 'inner_wheel_not_confirmed_on_road'))
    return result


def corridor_reference(frame, points, s_lo, s_hi, cfg):
    """관측 벽이 사양 위치(±0.425 m)에 연속으로 있는지. 잔디 쪽 도로 끝 추정의 근거."""
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    s, l = frame.project(pts) if len(pts) else (np.zeros(0), np.zeros(0))
    bins = np.arange(s_lo, s_hi+cfg.wall_bin_m*.5, cfg.wall_bin_m)
    out = {}
    for name, sign in SIDES:
        expected = sign*cfg.wall_offset_m
        near = np.abs(l-expected) <= cfg.wall_tolerance_m
        sel = near & (s >= s_lo-1e-9) & (s <= s_hi+1e-9)
        hit = np.zeros(max(1, len(bins)-1), dtype=bool)
        if len(bins) > 1 and np.any(sel):
            idx = np.clip(np.searchsorted(bins, s[sel], side='right')-1, 0, len(bins)-2)
            hit[idx] = True
        coverage = float(np.mean(hit)) if len(bins) > 1 else 0.
        run = gap = 0
        for flag in hit:
            run = 0 if flag else run+1
            gap = max(gap, run)
        residual = float(np.median(l[sel]-expected)) if np.any(sel) else None
        out[name] = {'coverage': coverage, 'max_gap_m': gap*cfg.wall_bin_m, 'points': int(np.sum(sel)),
                     'median_residual_m': residual}
    for name, _ in SIDES:
        other = out['right' if name == 'left' else 'left']
        mine = out[name]
        out[name]['edge_verified'] = bool(
            mine['coverage'] >= cfg.wall_min_coverage and mine['max_gap_m'] <= cfg.wall_max_gap_m
            and mine['median_residual_m'] is not None and abs(mine['median_residual_m']) <= cfg.wall_tolerance_m/2
            and other['coverage'] >= cfg.opposite_min_coverage and other['median_residual_m'] is not None
            and abs(other['median_residual_m']) <= cfg.wall_tolerance_m)
    return out


# ------------------------------------------------------------------ 관측 격자
class ObservationGrid:
    """base_link(t_ref) 격자. 반환 광선이 지난 칸만 FREE, 끝점은 OCC, 나머지는 UNOBS다.

    광선 사이 구멍·후방 마스크·unknown 방향·가림 뒤는 UNOBS로 남는다. 자유 공간을 만들어 채우지 않는다.
    """

    def __init__(self, scene, cfg):
        self.cfg = cfg
        r = cfg.grid_resolution_m
        self.x0, self.x1 = cfg.grid_x_m
        self.y0, self.y1 = cfg.grid_y_m
        self.nx = int(round((self.x1-self.x0)/r))
        self.ny = int(round((self.y1-self.y0)/r))
        self.r = r
        state = np.zeros((self.nx, self.ny), dtype=np.int8)
        origins, hits = [], []
        for ray in scene.rays:
            if ray.status == 'return':
                origins.append(ray.ray_origin_xy_m)
                hits.append(ray.hit_xy_m)
        self.returns = np.asarray(hits, dtype=float).reshape(-1, 2)
        self.ages = np.asarray([scene.t_ref_s-ray.acquired_at_s for ray in scene.rays if ray.status == 'return'], dtype=float)
        self.compact_returns, self.return_components = compact_return_mask(scene, cfg)
        if len(hits):
            o = np.asarray(origins, dtype=float)
            h = self.returns
            vec = h-o
            dist = np.linalg.norm(vec, axis=1)
            free = np.maximum(0., dist-cfg.budget.range_bias_m)
            step = r/2
            counts = np.ceil(free/step).astype(int)
            total = int(np.sum(counts))
            if total:
                ray_index = np.repeat(np.arange(len(o)), counts)
                offsets = np.arange(total)-np.repeat(np.cumsum(counts)-counts, counts)
                t = np.minimum(offsets*step, np.repeat(free, counts))
                pts = o[ray_index]+(vec[ray_index]/np.maximum(dist[ray_index], 1e-9)[:, None])*t[:, None]
                ix, iy, inside = self.index(pts)
                state[ix[inside], iy[inside]] = FREE
            ix, iy, inside = self.index(h)
            state[ix[inside], iy[inside]] = OCC
        ex, ey = self.centers()
        ego = (np.abs(ex) <= cfg.length_m/2+GEOMETRY_EPS_M) & (np.abs(ey) <= cfg.width_m/2+GEOMETRY_EPS_M)
        state[ego] = EGO
        self.state = state

    def index(self, xy):
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        ix = np.floor((xy[:, 0]-self.x0)/self.r).astype(int)
        iy = np.floor((xy[:, 1]-self.y0)/self.r).astype(int)
        inside = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, inside

    def centers(self):
        xs = self.x0+(np.arange(self.nx)+.5)*self.r
        ys = self.y0+(np.arange(self.ny)+.5)*self.r
        return np.meshgrid(xs, ys, indexing='ij')

    def observed_free(self):
        """반환 광선이 실제 지난 칸만. 인접 UNOBS를 팽창으로 채우지 않는다.

        광선이 칸 일부를 지난 것이 칸 전체의 연속 공간 증명은 아니다. 이 격자는 오프라인 선별
        근사이며 독립 보호층의 실제 광선/연속 외곽 검사를 대신할 수 없다.
        """
        free = self.state == FREE
        near_occ = _dilate(self.state == OCC, max(1, int(round(self.cfg.budget.range_bias_m/self.r))))
        return free & ~near_occ

    def hidden_seeds(self, corridor_mask):
        """숨은 차량(폭·길이 최소 가정)의 중심이 있을 수 있는 칸. 사각형 전체가 UNOBS·통로 안이어야 한다."""
        hx = int(math.ceil(self.cfg.hidden_vehicle_length_m/2/self.r))
        hy = int(math.ceil(self.cfg.hidden_vehicle_width_m/2/self.r))
        blocked = (self.state != UNOBS) | ~corridor_mask
        # 격자 밖은 관측되지 않은 공간으로 둔다(숨은 차량이 격자 경계 너머에 있을 수 있다).
        padded = np.pad(blocked, ((hx, hx), (hy, hy)), constant_values=False)
        integral = np.pad(np.cumsum(np.cumsum(padded, axis=0), axis=1), ((1, 0), (1, 0)))
        a = integral[2*hx+1:, 2*hy+1:]-integral[:-2*hx-1, 2*hy+1:]-integral[2*hx+1:, :-2*hy-1]+integral[:-2*hx-1, :-2*hy-1]
        return (a == 0) & (self.state == UNOBS) & corridor_mask


def _dilate(mask, k):
    out = mask.copy()
    for _ in range(k):
        grown = out.copy()
        grown[1:] |= out[:-1]
        grown[:-1] |= out[1:]
        grown[:, 1:] |= out[:, :-1]
        grown[:, :-1] |= out[:, 1:]
        out = grown
    return out


def corridor_mask(cell_l, corridor, cfg):
    """숨은 차량이 있을 수 있는 통로. 벽이 확인된 쪽은 벽 바깥을 제외하고, 확인되지 않은 쪽(개구부)은 남긴다."""
    mask = np.ones(cell_l.shape, dtype=bool)
    for name, sign in SIDES:
        if corridor[name]['coverage'] >= cfg.opposite_min_coverage and corridor[name]['median_residual_m'] is not None                 and abs(corridor[name]['median_residual_m']) <= cfg.wall_tolerance_m:
            mask &= ~(sign*cell_l > cfg.wall_offset_m-cfg.wall_tolerance_m)
    return mask


def cell_coordinates(grid, frame, stride=4):
    """격자 칸 중심의 (s, l). 성긴 격자(stride 칸 간격)에서 투영한 뒤 쌍선형 보간한다.

    경로 좌표는 경로 근처에서 매끄러워 2차 오차가 (간격²/8)·곡률 정도다(4 cm·반경 0.3 m에서 약 0.7 mm).
    곡률 중심 근처나 경로가 크게 감겨 양 끝 연장선 투영이 겹치는 경로 범위 밖에서는 투영 자체가 불연속이라
    보간 오차가 cm 단위가 될 수 있다(합성 반경 0.5 m·212° 경로에서 최대 1.9 cm, 통로 판정 경계 밖).
    통로 안(|l| <= 0.42 m)에서도 곡선 안쪽의 s는 기울기가 R/(R-l)배라 수 cm 오차가 날 수 있다(같은 경로 3.8 cm).
    """
    ex, ey = grid.centers()
    if stride <= 1:
        s, l = frame.project(np.column_stack((ex.ravel(), ey.ravel())))
        return s.reshape(ex.shape), l.reshape(ex.shape)
    xs, ys = ex[:, 0], ey[0]
    cx = np.r_[xs[::stride], xs[-1]] if (len(xs)-1) % stride else xs[::stride]
    cy = np.r_[ys[::stride], ys[-1]] if (len(ys)-1) % stride else ys[::stride]
    gx, gy = np.meshgrid(cx, cy, indexing='ij')
    cs, cl = frame.project(np.column_stack((gx.ravel(), gy.ravel())))
    cs, cl = cs.reshape(gx.shape), cl.reshape(gx.shape)
    ix = np.clip(np.searchsorted(cx, xs, side='right')-1, 0, len(cx)-2)
    iy = np.clip(np.searchsorted(cy, ys, side='right')-1, 0, len(cy)-2)
    fx = ((xs-cx[ix])/(cx[ix+1]-cx[ix]))[:, None]
    fy = ((ys-cy[iy])/(cy[iy+1]-cy[iy]))[None, :]
    def lerp(field):
        a = field[ix][:, iy]
        b = field[ix+1][:, iy]
        c = field[ix][:, iy+1]
        d = field[ix+1][:, iy+1]
        return a*(1-fx)*(1-fy)+b*fx*(1-fy)+c*(1-fx)*fy+d*fx*fy
    return lerp(cs), lerp(cl)


# ------------------------------------------------------------------ 측방 위협·간격
def compact_return_mask(scene, cfg):
    """차량 크기 정도의 반환 묶음만 측방 양보를 촉발한다.

    긴 반환열은 '벽/정지물'로 확정하지 않는다. 차 여러 대가 붙어 있어도 길 수 있으므로 모든 점유와
    동일한 동적 간격 검사는 그대로 남긴다. 이 분류는 의미 인식이나 충돌 검사 면제가 아니다.
    크기 비교값은 최대 차체 대각선+거리 오차 양끝+취득 중 물체 이동의 설계 상계다.
    이동 상계 초과·가림·서로 붙은 차는 오분류될 수 있으므로 점유 검사를 대신하지 않는다.
    """
    rays = [r for r in scene.rays if r.status == 'return']
    if not rays:
        return np.zeros(0, dtype=bool), []
    points = np.asarray([r.hit_xy_m for r in rays])
    labels = np.zeros(len(points), dtype=int)
    group = 0
    for i in range(1, len(rays)):
        a, b = rays[i-1:i+1]
        theta = abs(math.atan2(a.direction_xy[0]*b.direction_xy[1]-a.direction_xy[1]*b.direction_xy[0],
                              a.direction_xy[0]*b.direction_xy[0]+a.direction_xy[1]*b.direction_xy[1]))
        depth = min(math.dist(a.ray_origin_xy_m, a.hit_xy_m), math.dist(b.ray_origin_xy_m, b.hit_xy_m))
        join = (2*cfg.budget.range_bias_m+1.5*depth*theta+
                cfg.budget.other_lateral_speed_mps*abs(b.acquired_at_s-a.acquired_at_s))
        if b.source_index-a.source_index != 1 or math.dist(points[i], points[i-1]) > join:
            group += 1
        labels[i] = group
    compact = np.zeros(len(points), dtype=bool)
    details = []
    for k in range(group+1):
        take = labels == k
        p = points[take]
        # AABB 대각선은 회전한 차량의 실제 지름보다 길 수 있다. 방향별 투영 폭의 최댓값은
        # 실제 점 집합 지름의 하한이므로 회전만으로 큰 물체라고 오판하지 않는다.
        angles = np.arange(8)*math.pi/8
        projections = p@np.array([np.cos(angles), np.sin(angles)])
        span = float(np.max(np.ptp(projections, axis=0)))
        ids = np.flatnonzero(take)
        span_s = rays[ids[-1]].acquired_at_s-rays[ids[0]].acquired_at_s
        limit = (math.hypot(cfg.max_vehicle_length_m, cfg.max_vehicle_width_m)+2*cfg.budget.range_bias_m+
                 cfg.compact_point_motion_bound_mps*span_s)
        compact[take] = span <= limit
        details.append({'points': int(take.sum()), 'span_m': span, 'compact_limit_m': limit,
                        'compact': bool(span <= limit)})
    return compact, details


def side_threats(frame, state, grid, cfg, cell_s, cell_l, observed):
    """측후방 창의 벽이 아닌 반환. 정체·속도를 추정하지 않는다.

    보이는 반환이 물체의 가까운 면이라는 보장은 없다(후방 마스크·가림). 가까운 쪽 한계는 (1) 보이는 바깥
    끝에서 규정 최대 차폭만큼, (2) 자차와 반환 사이 칸 가운데 관측된 빈 칸이 아닌 첫 칸 중 더 바깥쪽이다.
    """
    pts = grid.returns
    ages = grid.ages
    if not len(pts):
        return {name: {'present': False, 'points': 0, 'nearest': None} for name, _ in SIDES}
    s, l = frame.project(pts)
    ds = s-state['s0']
    window = (ds >= -cfg.band_behind_m) & (ds <= cfg.band_ahead_m)
    wall = np.zeros(len(pts), dtype=bool)
    for name, sign in SIDES:
        wall |= np.abs(l-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m
    interior = np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m
    out = {}
    for name, sign in SIDES:
        raw_side = window & interior & ~wall & (sign*(l-state['d0']) >= state['half_span_m'])
        side = raw_side & grid.compact_returns
        nearest = None
        if np.any(side):
            gap = sign*(l[side]-state['d0'])-state['half_span_m']
            k = int(np.argmin(gap))
            visible_near = float(sign*l[side][k])
            far = float(np.max(sign*l[side]))
            width_bound = far-cfg.max_vehicle_width_m
            lo, hi = float(np.min(s[side]))-.05, float(np.max(s[side]))+.05
            ego_edge = sign*state['d0']+state['half_span_m']
            between = ((cell_s >= lo) & (cell_s <= hi) & (sign*cell_l >= ego_edge) & (sign*cell_l < visible_near)
                       & ~observed & (grid.state != EGO))
            observed_bound = float(np.min(sign*cell_l[between])) if np.any(between) else visible_near
            near = max(width_bound, observed_bound)
            nearest = {'l_m': float(l[side][k]), 'visible_gap_m': float(gap[k]), 'far_l_m': float(sign*far),
                       'near_edge_l_m': float(sign*near), 'gap_m': float(near-ego_edge),
                       'near_edge_source': 'max_vehicle_width' if width_bound >= observed_bound else 'observed_cells',
                       'age_s': float(ages[side][k]) if len(ages) else None}
        out[name] = {'present': bool(np.sum(side) >= cfg.min_threat_points), 'points': int(np.sum(side)),
                     'extended_surface_points': int(np.sum(raw_side & ~grid.compact_returns)), 'nearest': nearest}
    return out


def lateral_clearance_check(candidate, points, ages, frame, corridor, cfg, max_progress_m=None):
    """후보 경로를 따라 차체 옆(종방향 겹침) 반환의 측방 간격. 앞뒤 물체는 기존 종방향 검사가 맡는다.

    max_progress_m까지(확약 시간 동안의 이동 + 정지거리)만 본다. 그 너머 자세에서 옆에 있게 될 물체는
    그때까지 움직일 수 있으므로 지금 정지물로 확정하지 않고 다음 판단 주기에서 다시 검사한다.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(pts):
        return {'ok': True, 'min_margin_m': None}
    s, l = frame.project(pts)
    wall = np.zeros(len(pts), dtype=bool)
    for name, sign in SIDES:
        wall |= (np.abs(l-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m) & corridor[name]['edge_verified']
    budget = cfg.budget
    curved = candidate['curved']
    dynamic = budget.dynamic_m(curved)
    required = np.where(wall, budget.range_bias_m+dynamic,
                        budget.range_bias_m+budget.other_lateral_speed_mps*(ages+budget.reaction_s)+dynamic)
    protrude = steering_protrusion_m(candidate['max_abs_steering_rad'], cfg)
    worst, worst_info = np.inf, None
    progress = candidate['progress']
    for index in range(0, len(candidate['poses']), 2):
        if max_progress_m is not None and progress[index] > max_progress_m:
            break
        pose = candidate['poses'][index]
        c, sn = math.cos(pose[2]), math.sin(pose[2])
        dx, dy = pts[:, 0]-pose[0], pts[:, 1]-pose[1]
        lx, ly = c*dx+sn*dy, -sn*dx+c*dy
        # 차체 횡폭 안의 점은 경로 위 물체(앞차 등)이며 기존 종방향 swept·제동 검사가 맡는다.
        beside = (np.abs(lx) <= cfg.length_m/2) & (np.abs(ly) >= cfg.width_m/2)
        if not np.any(beside):
            continue
        lateral = np.abs(ly[beside])-(cfg.width_m/2+protrude)
        margin = lateral-required[beside]
        k = int(np.argmin(margin))
        if margin[k] < worst:
            worst = float(margin[k])
            worst_info = {'lateral_m': float(lateral[k]), 'required_m': float(required[beside][k]),
                          'wall': bool(wall[beside][k]), 'pose_progress_m': float(progress[index])}
    return {'ok': bool(worst >= 0.), 'min_margin_m': None if worst is np.inf else worst, 'worst': worst_info,
            'steering_protrusion_m': protrude, 'checked_progress_m': max_progress_m}


def committed_progress_m(speed, age, cfg):
    """확약 시간 동안의 계획 이동 + 기존 보호 상수(반응 0.10 s·감속 2 m/s²)의 정지거리."""
    v = max(cfg.planned_speed_floor_mps, abs(speed))
    stop = v*(age+cfg.budget.reaction_s)+v*v/(2*2.)
    return v*cfg.commit_horizon_s+stop


def lateral_sweep(state, candidate, frame, cfg, horizon_s, speed):
    """자차 이동 좌표(상대 운동)에서의 횡이동 외곽. 종방향 전진은 기존 검사가 맡으므로 제외한다.

    후륜축이 경로 법선으로 (d - d_rear0)만큼 옮겨지고 차체는 후륜축을 중심으로 돈다. 도로 곡률에 의한
    회전은 같은 도로를 달리는 다른 차와 공통이므로 뺀다.
    """
    v = max(cfg.planned_speed_floor_mps, abs(speed))
    progress = candidate['progress']
    times = progress/v
    keep = times <= horizon_s
    road_heading = float(frame.heading(np.array([state['s_rear0']]))[0])
    normal = np.array([-math.sin(road_heading), math.cos(road_heading)])
    delta = candidate['offset'][keep]-state['d_rear0']
    heading = candidate['poses'][keep, 2]-(candidate['path_heading'][keep]-road_heading)
    lr = cfg.wheelbase_m/2
    x = -lr+delta*normal[0]+lr*np.cos(heading)
    y = delta*normal[1]+lr*np.sin(heading)
    return np.column_stack((x, y, heading)), times[keep]


def sweep_cells(grid, poses, times, cfg, protrusion=0.):
    """자세열 외곽이 처음 덮는 시각. 덮지 않는 칸은 inf."""
    ex, ey = grid.centers()
    first = np.full(ex.shape, np.inf)
    half_l, half_w = cfg.length_m/2, cfg.width_m/2+protrusion
    for (x, y, h), t in zip(poses, times):
        c, s = math.cos(h), math.sin(h)
        dx, dy = ex-x, ey-y
        inside = (np.abs(c*dx+s*dy) <= half_l+GEOMETRY_EPS_M) & (np.abs(-s*dx+c*dy) <= half_w+GEOMETRY_EPS_M)
        first = np.where(inside & (first == np.inf), t, first)
    return first


def hidden_reach(seeds_xy, cells_xy, tau, cfg, ego_half_w):
    """숨은 차량이 각 칸에 τ 안에 닿을 수 있는지. 자차 뒤에서 차로가 겹친 차는 옆으로 빠진 뒤에만 앞으로 온다."""
    if not len(seeds_xy) or not len(cells_xy):
        return np.zeros(len(cells_xy), dtype=bool), None
    v_long, v_lat = cfg.hidden_relative_speed_mps, cfg.hidden_lateral_speed_mps
    half_l, half_w = cfg.hidden_vehicle_length_m/2, cfg.hidden_vehicle_width_m/2
    rear_plane = -cfg.length_m/2
    reach = np.zeros(len(cells_xy), dtype=bool)
    best = np.full(len(cells_xy), np.inf)
    # 최대 τ 안에 닿을 수 없는 먼 seed는 미리 뺀다(자차 차단은 도달 시간을 늘리기만 한다).
    t_max = float(np.max(tau))
    lo = np.min(cells_xy, axis=0)-[v_long*t_max+half_l, v_lat*t_max+half_w]
    hi = np.max(cells_xy, axis=0)+[v_long*t_max+half_l, v_lat*t_max+half_w]
    seeds_xy = seeds_xy[np.all((seeds_xy >= lo) & (seeds_xy <= hi), axis=1)]
    if not len(seeds_xy):
        return reach, best
    for start in range(0, len(cells_xy), 64):
        p = cells_xy[start:start+64]
        q = seeds_xy
        dx = p[:, None, 0]-q[None, :, 0]
        dy = p[:, None, 1]-q[None, :, 1]
        need_x = np.maximum(0., np.abs(dx)-half_l)
        need_y = np.maximum(0., np.abs(dy)-half_w)
        t_free = np.maximum(need_x/v_long, need_y/v_lat)
        behind = (q[None, :, 0]+half_l <= rear_plane) & (p[:, None, 0] > rear_plane)
        overlap = np.abs(q[None, :, 1]) < ego_half_w+half_w
        side = np.sign(p[:, None, 1])
        clear_y = np.maximum(0., ego_half_w+half_w-side*q[None, :, 1])
        to_plane = np.maximum(0., (rear_plane-half_l)-q[None, :, 0])
        after = np.maximum(0., need_x-to_plane)
        t_block = np.maximum(t_free, np.maximum(clear_y/v_lat, to_plane/v_long)+after/v_long)
        t_need = np.where(behind & overlap, t_block, t_free)
        m = np.min(t_need, axis=1)
        best[start:start+64] = m
        reach[start:start+64] = m <= tau[start:start+64]
    return reach, best


def bypass_candidate(frame, state, points, side, cfg, *, stationary, ahead_m=1.2):
    """자차 횡폭 안의 앞 물체를 side 쪽으로 지나는 오프셋 후보(G2 공간 예산 비교용).

    정지 여부는 이 함수가 판정하지 않는다. stationary=True는 호출자가 별도 근거(자차 정지 중 반복
    스캔의 위치 불변 등)를 확보했다는 가정이며, 그때만 상대 횡운동 항을 뺀 간격을 쓴다.
    """
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(pts):
        return None
    s, l = frame.project(pts)
    ds = s-state['s0']
    wall = np.zeros(len(pts), dtype=bool)
    for _, sign in SIDES:
        wall |= np.abs(l-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m
    ahead = (ds > cfg.length_m/2) & (ds <= ahead_m) & ~wall & (np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m)
    in_path = ahead & (np.abs(l-state['d0']) < state['half_span_m'])
    if not np.any(in_path):
        return None
    lo, hi = float(np.min(l[in_path])), float(np.max(l[in_path]))
    cluster = ahead & (l >= lo-.2) & (l <= hi+.2)
    lo, hi = float(np.min(l[cluster])), float(np.max(l[cluster]))
    clearance, detail = cfg.budget.lateral_clearance(moving=not stationary)
    sign = 1. if side == 'left' else -1.
    target = hi+clearance+state['half_span_m'] if sign > 0 else lo-clearance-state['half_span_m']
    candidate = offset_candidate(frame, state, target, cfg, 'bypass_'+side)
    candidate.update(obstacle_l_range_m=(lo, hi), bypass_clearance_m=clearance, bypass_clearance_detail=detail,
                     stationary_assumed=bool(stationary))
    return candidate


# ------------------------------------------------------------------ 후보 검증
def verify_candidate(candidate, hold, context, cfg, *, discretionary):
    """후보 하나의 기하·노면 정책·측방 간격·관측·숨은 차량 도달 검사.

    숨은 차량 도달은 중앙 복귀·양보·유지 모두에 적용한다. discretionary는 기록용 인자다.
    """
    frame, state, grid = context['frame'], context['state'], context['grid']
    result = {'name': candidate['name'], 'd_target_m': candidate['d_target_m'], 'valid': candidate['valid'],
              'problems': list(candidate['problems']), 'reasons': []}
    if not candidate['valid']:
        result['reasons'].append('geometry_invalid')
        result['verified'] = False
        return result
    surface = surface_check(frame, candidate, cfg, current_pose=(0., 0., 0.))
    result['surface'] = {k: surface.get(k) for k in ('policy', 'ok', 'grass_side', 'outer_unconfirmed_samples',
                                                     'outer_beyond_samples', 'max_body_excess_m', 'edge_margin_m', 'reason',
                                                     's_range_needing_edge')}
    if not surface['ok']:
        result['reasons'].append(surface['reason'])
    if surface.get('grass_side'):
        lo, hi = surface['s_range_needing_edge']
        ref = corridor_reference(frame, grid.returns, lo-.05, hi+.05, cfg)
        result['edge_reference'] = ref[surface['grass_side']]
        if not ref[surface['grass_side']]['edge_verified']:
            result['reasons'].append('grass_edge_reference_unverified')
    clearance = lateral_clearance_check(candidate, grid.returns, grid.ages, frame, context['corridor'], cfg,
                                        committed_progress_m(context['speed'], context['age_s'], cfg))
    result['lateral_clearance'] = clearance
    if not clearance['ok']:
        result['reasons'].append('lateral_clearance_short')
    # 새로 차지할 측방 영역(자차 이동 좌표)
    horizon = cfg.commit_horizon_s
    poses, times = lateral_sweep(state, candidate, frame, cfg, horizon, context['speed'])
    protrude = steering_protrusion_m(candidate['max_abs_steering_rad'], cfg)
    first = sweep_cells(grid, poses, times, cfg, protrude)
    hold_poses, hold_times = lateral_sweep(state, hold, frame, cfg, horizon, context['speed']) if hold is not None else (poses[:1], times[:1])
    hold_first = sweep_cells(grid, hold_poses, hold_times, cfg, steering_protrusion_m(hold['max_abs_steering_rad'], cfg) if hold else 0.)
    new = np.isfinite(first) & ~np.isfinite(hold_first) & (grid.state != EGO)
    # 회전으로 뒷면이 물러나는 구역도 검사한다. 상대차의 책임을 관측 증거로 대신하지 않는다.
    ex, ey = grid.centers()
    result['rear_zone_cells_excluded'] = 0
    result['new_cells'] = int(np.sum(new))
    if np.any(new):
        observed = context['observed_free']
        bad = new & ~observed
        result['new_cells_unobserved'] = int(np.sum(bad))
        if np.any(bad):
            result['reasons'].append('lateral_intent_unverified')
        cells = np.column_stack((ex[new], ey[new]))
        tau = first[new]+context['age_s']
        seeds = context['hidden_seeds_xy']
        reach, best = hidden_reach(seeds, cells, tau, cfg, cfg.width_m/2)
        result['hidden_reach_cells'] = int(np.sum(reach))
        result['hidden_min_margin_s'] = float(np.min(best-tau)) if best is not None and len(best) else None
        if np.any(reach):
            result['reasons'].append('hidden_vehicle_reach')
    else:
        result['new_cells_unobserved'] = 0
        result['hidden_reach_cells'] = 0
    result['verified'] = not result['reasons']
    # 숨은 차량 도달만 남은 경우를 따로 표시한다(선택 이동에서는 불합격, 필요한 이동에서는 잔여 위험).
    result['verified_except_hidden'] = not [r for r in result['reasons'] if r != 'hidden_vehicle_reach']
    return result


def stopping_steering(options, points, cfg, speed, age, threat_sign):
    """정지 중 조향 선택. 정지거리 동안 위협 쪽으로 가장 덜 이동하고 반환과 겹치지 않는 조향."""
    distance = max(0., speed)*(age+cfg.budget.reaction_s)+max(0., speed)**2/(2*2.)
    pts = np.asarray(points, dtype=float).reshape(-1, 2)
    best = None
    for name, angle in options:
        if angle is None or not math.isfinite(angle):
            continue
        kappa = math.tan(angle)/cfg.wheelbase_m
        travel = np.linspace(0., distance, max(2, int(distance/.01)+1))
        if abs(kappa) < 1e-6:
            x, y, h = travel, 0*travel, 0*travel
        else:
            h = kappa*travel
            x, y = np.sin(h)/kappa, (1-np.cos(h))/kappa
        approach = float(np.max(threat_sign*y)) if threat_sign else 0.
        hit = False
        for xi, yi, hi in zip(x, y, h):
            c, s = math.cos(hi), math.sin(hi)
            dx, dy = pts[:, 0]-xi, pts[:, 1]-yi
            if np.any((np.abs(c*dx+s*dy) <= cfg.length_m/2) & (np.abs(-s*dx+c*dy) <= cfg.width_m/2)):
                hit = True
                break
        key = (hit, approach)
        if best is None or key < best[0]:
            best = (key, name, angle, approach, hit, distance)
    if best is None:
        return {'name': 'zero', 'steering_rad': 0., 'approach_m': None, 'collides': None}
    return {'name': best[1], 'steering_rad': best[2], 'approach_m': best[3], 'collides': best[4],
            'stopping_distance_m': best[5]}


def build_context(scene, frame, speed, cfg):
    """한 판단의 관측 문맥(격자·벽 근거·숨은 차량 seed·측방 위협). 지도·정답을 받지 않는다."""
    state = ego_state(frame, cfg)
    grid = ObservationGrid(scene, cfg)
    corridor = corridor_reference(frame, grid.returns, state['s0']-cfg.band_behind_m, state['s0']+cfg.horizon_m, cfg)
    cell_s, cell_l = cell_coordinates(grid, frame)
    seeds = grid.hidden_seeds(corridor_mask(cell_l, corridor, cfg))
    observed = grid.observed_free()
    ex, ey = grid.centers()
    age = scene.t_ref_s-scene.source_first_s if scene.source_first_s is not None else cfg.budget.observation_age_s
    return {'frame': frame, 'state': state, 'grid': grid, 'corridor': corridor, 'speed': speed,
            'observed_free': observed, 'hidden_seed_mask': seeds,
            'hidden_seeds_xy': np.column_stack((ex[seeds], ey[seeds])), 'age_s': age,
            'threats': side_threats(frame, state, grid, cfg, cell_s, cell_l, observed)}


# ------------------------------------------------------------------ 상태기계
class LateralIntentLimiter:
    """횡이동 의도와 관측 근거를 분리한다. 선택 경로·이유를 반환하며 종방향 판단은 하지 않는다."""

    def __init__(self, cfg=None):
        self.cfg = cfg or LateralIntentConfig()
        self.reset()

    def reset(self):
        self.mode = 'CENTER'
        self.intent_offset = None
        self.absent_fresh = 0
        self.grass_time = 0.
        self.last_ref = None
        self.last_threat_s = None

    def step(self, scene, base_path, speed, *, fresh=True, previous_steering=0.):
        cfg = self.cfg
        if not cfg.enabled:
            return {'enabled': False, 'policy': cfg.surface_policy, 'mode': 'DISABLED',
                    'path': base_path, 'speed_cap_mps': None, 'reasons': [], 'control_authorization': False}
        now = scene.t_ref_s
        dt = 0. if self.last_ref is None else max(0., now-self.last_ref)
        self.last_ref = now
        out = {'enabled': cfg.enabled, 'policy': cfg.surface_policy, 'mode_before': self.mode,
               'control_authorization': False, 'validation_scope': 'offline_v1_temporal_contract_unfinished'}
        if base_path is None:
            # 기본 경로가 없으면 기존 G1이 HOLD한다. 의도(목표 오프셋)는 지우지 않고 P0로 바꾸지 않는다.
            out.update(mode='NO_BASE_PATH', path=None, speed_cap_mps=0., reasons=['base_path_missing'])
            return out
        try:
            frame = PathFrame(base_path)
        except ValueError:
            out.update(mode='NO_BASE_PATH', path=None, speed_cap_mps=0., reasons=['base_path_invalid'])
            return out
        context = build_context(scene, frame, speed, cfg)
        state, grid, corridor = context['state'], context['grid'], context['corridor']
        threats, seeds, age = context['threats'], context['hidden_seed_mask'], context['age_s']
        present = {name: threats[name]['present'] for name, _ in SIDES}
        hold = offset_candidate(frame, state, state['d0'], cfg, 'hold')
        center = offset_candidate(frame, state, 0., cfg, 'center')
        moving_clearance, clearance_detail = cfg.budget.lateral_clearance(True, hold['curved'])
        limit = cfg.policy_limit_m(hold['curved'])['selected_m']
        candidates = {'hold': hold, 'center': center}
        threat_sides = [name for name, _ in SIDES if present[name]]
        if len(threat_sides) == 1:
            name = threat_sides[0]
            sign = 1. if name == 'left' else -1.
            nearest = threats[name]['nearest']
            # 위협 반대쪽으로 측방 간격 요구를 회복하는 최소 오프셋(정책 한계로 자름)
            needed = nearest['near_edge_l_m']-sign*(state['half_span_m']+moving_clearance)
            target = sign*min(sign*needed, sign*state['d0'])
            target = float(np.clip(target, -limit, limit))
            if sign*(target-state['d0']) < -.005:
                candidates['yield'] = offset_candidate(frame, state, target, cfg, 'yield')
        checks = {}
        for key, cand in candidates.items():
            discretionary = key == 'center'
            if key == 'hold':
                checks[key] = verify_candidate(cand, None, context, cfg, discretionary=False)
            else:
                checks[key] = verify_candidate(cand, hold, context, cfg, discretionary=discretionary)
        selected, mode, reasons = None, None, []
        def toward(cand, side):
            # 원점 기준 목표 변화. 측방 간격 자체는 후보 검증의 확약 구간 검사가 따로 보장한다.
            change = cand['d_target_m']-state['d0']
            return (side == 'left' and change > .005) or (side == 'right' and change < -.005)
        if len(threat_sides) == 2:
            self.absent_fresh = 0
            self.last_threat_s = now
            if checks['hold']['verified']:
                selected, mode = 'hold', 'SQUEEZE_HOLD_OFFSET'
            else:
                mode, reasons = 'HOLD_STOP', ['squeezed_hold_unverified']
        elif len(threat_sides) == 1:
            side = threat_sides[0]
            self.absent_fresh = 0
            self.last_threat_s = now
            if 'yield' in checks and checks['yield']['verified']:
                selected, mode = 'yield', 'YIELD'
            elif checks['hold']['verified']:
                selected, mode = 'hold', 'HOLD_OFFSET'
                if 'yield' in checks:
                    reasons.append('yield_unverified')
            else:
                mode, reasons = 'HOLD_STOP', ['hold_offset_unverified']
            if selected and toward(candidates[selected], side):
                selected, mode, reasons = None, 'HOLD_STOP', reasons+['would_move_toward_threat']
        else:
            if fresh:
                self.absent_fresh += 1
            offset_now = abs(state['d0']) > .005
            release_ready = self.mode == 'CENTER' or self.absent_fresh >= cfg.release_fresh
            center_check = checks['center']
            if not offset_now and center_check['verified']:
                selected, mode = 'center', 'CENTER'
            elif release_ready and center_check['verified']:
                selected, mode = 'center', 'CENTER'
            elif checks['hold']['verified']:
                selected, mode = 'hold', 'HOLD_OFFSET'
                if not release_ready:
                    reasons.append('release_wait')
                elif 'hidden_vehicle_reach' in center_check['reasons']:
                    reasons.append('recenter_hidden_reach')
                else:
                    reasons.append('recenter_unverified')
            else:
                mode, reasons = 'HOLD_STOP', ['hold_offset_unverified']
        surface_now = surface_check(frame, hold, cfg)
        on_grass_now = bool(surface_now.get('outer_unconfirmed_samples', 0)) if cfg.surface_policy == 'two_wheel_grass' else False
        self.grass_time = self.grass_time+dt if on_grass_now else 0.
        if self.grass_time > cfg.grass_max_continuous_s:
            reasons.append('grass_duration_exceeded')
        braking = None
        if mode == 'HOLD_STOP':
            sign = 0.
            if len(threat_sides) == 1:
                sign = 1. if threat_sides[0] == 'left' else -1.
            options = [('previous', previous_steering), ('zero', 0.)]
            braking = stopping_steering(options, grid.returns, cfg, speed, age, sign)
        self.mode = mode
        if selected is not None:
            self.intent_offset = candidates[selected]['d_target_m']
        out.update(mode=mode, selected=selected, reasons=reasons,
                   path=None if selected is None else candidates[selected]['path'],
                   speed_cap_mps=0. if mode == 'HOLD_STOP' else None, braking_steering=braking,
                   state={k: v for k, v in state.items()}, threats=threats, corridor=corridor,
                   checks=checks, lateral_clearance_required_m=moving_clearance,
                   clearance_detail=clearance_detail, policy_limit=cfg.policy_limit_m(hold['curved']),
                   grass_time_s=self.grass_time, hidden_seed_cells=int(np.sum(seeds)))
        return out
