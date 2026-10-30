"""G2 중앙 통로 명시 시험 후보. 기본 경로와 경기 기본값은 이 모듈을 쓰지 않는다.

세 요소를 묶는다.
- 표면 정렬: 같은 표면의 광선 산포를 표면 대표값 기준으로 정렬한다. 광선별 30 mm 이하의 탐색 가정이며
  결정론적 여유 보존이 아니다. 대표값이 치우치거나 작은 돌출이 있으면 원시 반환 기준 여유는 25 mm까지 줄어든다.
  원시 반환은 독립 보호가 따로 본다.
- 통로 계획: 순차 스캔 시각표로 옆면 광선 나이와 통로 폭을 계산해 넓은 쪽과 측정 중앙을 고른다.
- 시작 가속 항: 안쪽 경계 면이 외벽 기준 최근 창에서 다가오지 않았을 때만, 그 면의 광선에 한해
  0.5 m/s*age 대신 관측 조건 항*age + 시험 가속*age^2/2를 쓴다. 관측 조건 항은 평균 변화량의 오차를 고려한
  값이고, 순간 속도나 미래 속도의 물리 상계가 아니다. 3 m/s^2는 시험 가정이며 타차 기구 보장이 아니다.

근거의 적용 범위는 광선으로 이어진 경계 면 구간이다. 그 면과 이어지지 않은 반환은 어떤 깊이에 있어도
원래 운동 여유를 유지한다. 물체 ID를 판정하지 않고 미관측을 자유 공간으로 채우지 않는다.
추종, 관측 지지, 독립 보호, STOP, 차량/센서 모델은 건드리지 않는다.
"""
from dataclasses import dataclass, replace
import math

import numpy as np

from arena_autonomy.bypass_entry_trial import (EntryGuide, EntryIntent, current_outer_wall_padding,
    straight_reference)
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points


@dataclass(frozen=True)
class CorridorPolicy:
    noise_budget_m: float = .03       # 기존 C1 ±30 mm 값. 새 측정 주장이 아님
    snap_budget_m: float = .03        # 광선별 정렬 상한. 원시 반환의 남는 고정 여유는 55 mm-이 값
    approach_budget_m: float = .03    # 창 안의 허용 접근량. 관측 조건이다
    surface_error_budget_m: float = .06  # 양 끝 표면값 오차 예산. 평균 변화량 기준이며 물리 상계가 아니다
    wall_fit_error_m: float = .01     # 첫 선택 벽선의 잔차 상한(기존 observed_straight_wall)
    face_minimum_returns: int = 8
    face_bin_m: float = .05
    face_minimum_bins: int = 3
    face_gap_rays: int = 2            # 같은 면 안에서 건너뛸 수 있는 연속 광선 수(이상점 허용)
    face_association_gap_m: float = .06  # 스캔 사이 같은 면으로 잇는 벽 방향 간격
    quantile_rank: int = 0            # 뒷면만 보일 때 가장 안쪽 반환. 이상점은 통로를 좁히는 쪽이다
    along_min_m: float = -.45
    along_max_m: float = 2.50         # 기존 원호 검사 지평. 먼 표적도 첫 계획에서 본다
    normal_min_m: float = .06
    normal_max_m: float = .78
    window_s: float = .40             # 근거 창, 10 Hz 서로 다른 5스캔
    minimum_scans: int = 5
    onset_acceleration_mps2: float = 3.  # 시험 가정. 실측도 타차 기구 보장도 아니다
    scan_reuse_s: float = .08         # 50 Hz 제어의 같은 10 Hz 스캔 재사용 상한(시뮬레이션 일정)
    transitions_m: tuple = (.6, .8, .9, 1.2)  # 등록 전이. 실제 선택은 관측 전방 거리로 정한다
    transition_effective_fraction: float = .93  # 5차 전이의 잔여 편이가 0.4% 미만이 되는 비율
    start_slack_m: float = .10        # 가장 긴 전이보다 이만큼 멀면 아직 직진한다


REGISTERED_POLICIES = {'corridor_v1': CorridorPolicy()}


def wall_frame(points, config, side, wall):
    """외벽에서 안쪽(+)으로 잰 법선 거리와 벽 방향 좌표. 같은 스캔 안의 상대량이다."""
    sign = 1 if side == 'left' else -1
    slope, intercept = wall['slope'], wall['intercept_m']
    norm = math.sqrt(1+slope*slope)
    physical = intercept+sign*config.wall_offset_m*norm
    return (points[:, 0]+slope*points[:, 1])/norm, sign*(slope*points[:, 0]+physical-points[:, 1])/norm


def boundary_face(ids, points, along, normal, candidates, policy):
    """외벽에 가장 가까운 비벽 반환과 광선으로 이어진 면 구간.

    법선 띠 안에서 광선 순서로 이어진 반환만 모은다. 띠 밖이거나 끊긴 반환은 깊이와
    무관하게 이 면에 넣지 않는다. 같은 띠에서 12 cm 안쪽으로 이어진 두 물체는 구별하지 못한다.
    """
    p = policy
    window = candidates&(along >= p.along_min_m)&(along <= p.along_max_m)&(normal >= p.normal_min_m)&(normal <= p.normal_max_m)
    index = np.flatnonzero(window)
    if len(index) <= p.quantile_rank:
        return None
    nearest = int(index[np.argsort(normal[index])[p.quantile_rank]])
    band_high = normal[nearest]+2*p.noise_budget_m
    order = np.argsort(ids)
    rank = np.empty(len(ids), dtype=int)
    rank[order] = np.arange(len(ids))
    ranges = np.linalg.norm(points, axis=1)

    def walk(step):
        taken, last, missed, position = [], nearest, 0, rank[nearest]+step
        while 0 <= position < len(order) and missed <= p.face_gap_rays:
            k = int(order[position])
            limit = .10+.035*min(ranges[k], ranges[last])
            if (window[k] and normal[k] <= band_high and abs(int(ids[k])-int(ids[last])) <= p.face_gap_rays+1
                    and np.linalg.norm(points[k]-points[last]) <= limit):
                taken.append(k)
                last, missed = k, 0
            else:
                missed += 1
            position += step
        return taken
    members = np.asarray(sorted(walk(-1)+[nearest]+walk(1)), dtype=int)
    low = float(normal[nearest])
    bins = np.floor(along[members]/p.face_bin_m).astype(int)
    labels = np.unique(bins)
    if len(members) >= p.face_minimum_returns and len(labels) >= p.face_minimum_bins:
        # 벽에 수직인 뒷면은 한 벽 방향 구간에 몰린다. 구간별 대표값으로 면 표본을 동등하게 센다.
        value = float(np.median([np.median(normal[members][bins == label]) for label in labels]))
        method = 'parallel_face_median'
    else:
        value, method = low, 'low_order'
    return dict(normal_m=value, method=method, low_order_m=low, members=members, bins=len(labels),
                along_range_m=(float(along[members].min()), float(along[members].max())))


def connected_wall_returns(ids, points, along, normal, policy):
    """잡음 예산 안의 연속 벽 반환. 첫 선택의 15 mm 분류가 ±30 mm 산포에서 끊기는 것을 대신한다.

    누락 광선/다른 면을 건너 잇지 않고 65 cm 이상 연속인 구간만 받는다.
    """
    near = (np.abs(normal) <= policy.noise_budget_m+policy.wall_fit_error_m)&(along >= -.4)&(along <= 1.5)
    indices = np.flatnonzero(near)
    indices = indices[np.argsort(ids[indices])]
    mask = np.zeros(len(points), dtype=bool)
    if len(indices) < 2:
        return mask
    gaps = np.linalg.norm(np.diff(points[indices], axis=0), axis=1)
    limits = .10+.035*np.minimum(np.linalg.norm(points[indices[:-1]], axis=1), np.linalg.norm(points[indices[1:]], axis=1))
    breaks = (gaps > limits)|(np.abs(np.diff(ids[indices])) != 1)
    for piece in np.split(indices, np.flatnonzero(breaks)+1):
        if len(piece) >= 12 and np.ptp(points[piece, 0]) >= .65:
            mask[piece] = True
    return mask


class BoundaryMonitor:
    """경계 면의 스캔별 위치. 새 스캔에서만 누적하고 이어지는 같은 면 구간만 근거로 잇는다."""
    def __init__(self, policy):
        self.policy = policy
        self.reset()

    def reset(self):
        self.rows, self.key = [], None
        self.state = dict(stable=False, reason='no_observation')

    def update(self, scene, face, to_current=None):
        """to_current(이전 행)은 이전 면 끝점의 현재 벽 방향 구간. 불가능하면 None을 돌려준다."""
        key = (scene.source_epoch, scene.source_first_s)
        if key == self.key:
            return self.state
        if self.key is not None and (key[0] != self.key[0] or key[1] < self.key[1] or key[1]-self.key[1] > .25):
            self.rows = []
        self.key = key
        p = self.policy
        if face is None:
            self.rows = []
            self.state = dict(stable=False, reason='inner_boundary_not_observed')
            return self.state
        if self.rows and to_current is not None:
            previous = to_current(self.rows[-1])
            lo, hi = face['along_range_m']
            if previous is None or min(hi, previous[1])-max(lo, previous[0]) < -p.face_association_gap_m:
                # 다른 면이 가장 가까워졌다. 이전 면의 안정 근거를 넘기지 않는다.
                self.rows = []
        current = face['normal_m']
        self.rows.append(dict(source=scene.source_first_s, reference=scene.t_ref_s, normal=current,
                              endpoints=face.get('endpoints_xy_m')))
        latest = self.rows[-1]['source']
        while len(self.rows) > p.minimum_scans and latest-self.rows[1]['source'] >= p.window_s-1e-9:
            self.rows.pop(0)
        span = latest-self.rows[0]['source']
        approach = max(r['normal'] for r in self.rows)-current
        stable = len(self.rows) >= p.minimum_scans and span >= p.window_s-1e-9 and approach <= p.approach_budget_m
        self.state = dict(stable=stable, inner_normal_m=current, approach_m=approach, span_s=span, scans=len(self.rows),
            observed_rate_term_mps=((p.approach_budget_m+p.surface_error_budget_m)/span if span > 0 else None),
            rate_term_is_physical_bound=False,
            reason='low_approach_inner_boundary_candidate' if stable else 'inner_boundary_window_or_approach',
            object_identity_or_stationarity_proven=False, free_space_assumed=False)
        return self.state


def abeam_age(scene, side, reuse_s):
    """통과 쪽 옆면 방위 광선의 최악 나이. 표본 시각표만 쓰며 물체를 가정하지 않는다."""
    goal = -1. if side == 'left' else 1.   # 왼쪽 통과면 표적은 오른쪽 옆면
    rays = [ray for ray in scene.rays if ray.status != 'masked']
    best = max(rays, key=lambda ray: goal*ray.direction_xy[1])
    return float(scene.source_last_s-best.acquired_at_s+reuse_s)


def choose_side(rows):
    """예측 통로 폭이 넓은 쪽. 비슷해도 고른다. 비교값이며 그쪽 통과 허가가 아니다."""
    return 'left' if rows['left']['width_m'] > rows['right']['width_m'] else 'right'


def straight_blocked(rows, config):
    """직진 차체 띠에 양쪽 벽 기준의 안쪽 경계가 겹치는가. 현재 반환에 근거한다."""
    fixed = .025+config.measurement_padding_m
    limit = config.wall_offset_m+config.width_m/2+fixed
    return rows['left']['inner_normal_m'] <= limit and rows['right']['inner_normal_m'] <= limit


class PlannedIntent(EntryIntent):
    """관측 통로 계획이 고른 편이를 첫 후보로 쓴다. 후보의 현재 명령 검사는 그대로다."""
    def reset(self):
        super().reset()
        self.forced_offset = None

    def select(self, scene, comparison):
        result = super().select(scene, comparison)
        if self.forced_offset is None or self.sign is not None:
            return result
        row = next((r for r in result['candidates'] if r.get('eligible') and r['offset_m'] == self.forced_offset), None)
        if row is None:
            return result
        self.sign = math.copysign(1., row['offset_m'])
        return {**result, 'intent_direction':self.sign, 'selected':dict(offset_m=row['offset_m'],
            transition_m=row['transition_m'], speed_mps=row['target_speed_mps'], steering_rad=row['steering_rad'],
            ranking='observed_corridor_plan_not_full_path_free_proof')}


class CorridorGuide(EntryGuide):
    def __init__(self, config=None, entry=None, *, corridor_policy=None, **kwargs):
        self.corridor_policy = corridor_policy or REGISTERED_POLICIES['corridor_v1']
        super().__init__(config, entry, **kwargs)
        self.registered_entry = replace(self.entry, transitions_m=self.corridor_policy.transitions_m)
        self.entry = self.registered_entry
        self.padding_provider = self._padding

    def reset(self):
        super().reset()
        self.intent = PlannedIntent()
        self.monitors = dict(left=BoundaryMonitor(self.corridor_policy), right=BoundaryMonitor(self.corridor_policy))
        self.corridor_use, self.plan_detail, self.motion_history = {}, None, None

    def _padding(self, scene, config, side, wall, growth_s=0.):
        padding, evidence = current_outer_wall_padding(scene, config, side, wall, growth_s)
        if growth_s != 0.:
            raise ValueError('corridor trial has no growth comparison')
        p = self.corridor_policy
        padding = np.asarray(padding, dtype=float).copy()
        before = padding.copy()
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        rays = [ray for ray in scene.rays if ray.status == 'return']
        ids = np.asarray([ray.source_index for ray in rays])
        fixed = .025+config.measurement_padding_m
        static = padding <= fixed+1e-12
        along, normal = wall_frame(points, config, side, wall)
        if wall.get('reason') != 'current_connected_straight_wall_fit':
            static = static|connected_wall_returns(ids, points, along, normal, p)
            padding[static] = np.minimum(padding[static], fixed)
        face = boundary_face(ids, points, along, normal, ~static, p)

        def to_current(row):
            if self.motion_history is None or row['endpoints'] is None:
                return None
            try:
                moved = transport_points(row['endpoints'], self.motion_history, row['reference'], scene.t_ref_s)
            except (ValueError, MotionUnavailable):
                return None
            values = wall_frame(moved, config, side, wall)[0]
            return float(values.min()), float(values.max())
        if face is not None:
            members = face['members']
            face['endpoints_xy_m'] = points[[members[int(np.argmin(along[members]))], members[int(np.argmax(along[members]))]]]
        state = self.monitors[side].update(scene, face, to_current)
        use = dict(side=side, surface=None if face is None else dict(normal_m=face['normal_m'], method=face['method'],
                low_order_m=face['low_order_m'], bins=face['bins'], along_range_m=list(face['along_range_m'])),
            state=state, wall_rays_aligned=0, inner_rays_aligned=0, onset_rays=0, supported_rays=0, supported_along_m=None)
        supported = np.zeros(len(points), dtype=bool)
        if face is not None:
            # 근거는 이 면 구간에만 적용한다. 표면 띠 밖 반환은 면 구간 안이어도 그대로 둔다.
            supported[face['members']] = True
            supported &= np.abs(normal-face['normal_m']) <= p.snap_budget_m
            if supported.any():
                use.update(supported_rays=int(supported.sum()),
                           supported_along_m=[float(along[supported].min()), float(along[supported].max())])
        if state.get('stable') and supported.any():
            ages = np.asarray([scene.t_ref_s-ray.acquired_at_s for ray in rays], dtype=float)
            reduced = fixed+state['observed_rate_term_mps']*ages+.5*p.onset_acceleration_mps2*ages*ages
            padding[supported] = np.minimum(padding[supported], reduced[supported])
            use['onset_rays'] = int(supported.sum())
        wall_snap = np.clip(normal, 0., p.snap_budget_m)*static
        padding -= wall_snap
        use['wall_rays_aligned'] = int(np.count_nonzero(wall_snap))
        if face is not None:
            inner_snap = np.clip(face['normal_m']-normal, 0., p.snap_budget_m)*supported
            padding -= inner_snap
            use['inner_rays_aligned'] = int(np.count_nonzero(inner_snap))
        use['maximum_inner_padding_m'] = float(np.max(padding[supported])) if supported.any() else None
        use['padding_never_increased'] = bool(np.all(padding <= before+1e-12))
        self.corridor_use[side] = use
        return padding, dict(evidence, corridor=use)

    def _plan(self, scene):
        """첫 진입의 쪽/편이 비교. 결과는 후보 제안일 뿐 허가가 아니다."""
        p = self.corridor_policy
        if not source_valid(scene):
            return None
        reference, evidence = straight_reference(scene, self.config)
        if reference is None:
            return None
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        ids = np.asarray([ray.source_index for ray in scene.rays if ray.status == 'return'])
        rows = {}
        for side in ('left', 'right'):
            wall = evidence['walls'][side]
            along, normal = wall_frame(points, self.config, side, wall)
            face = boundary_face(ids, points, along, normal, along > .05, p)
            if face is None:
                return None
            # 뒷면 근처 반환의 중앙값. 최솟값은 거리 잡음만큼 앞당겨진다.
            near_face = along[face['members']]
            blocker_along = float(np.median(near_face[near_face <= near_face.min()+2*p.noise_budget_m]))
            age = abeam_age(scene, side, p.scan_reuse_s)
            motion = .5*p.onset_acceleration_mps2*age*age+(p.approach_budget_m+p.surface_error_budget_m)/p.window_s*age
            fixed = .025+self.config.measurement_padding_m
            low = self.config.width_m/2+fixed
            high = face['normal_m']-self.config.width_m/2-fixed-motion
            center = (low+high)/2
            sign = 1. if side == 'left' else -1.
            want = sign*(self.config.wall_offset_m-center)
            maximum = max(abs(o) for o in self.registered_entry.offsets_m)
            # 전이는 후륜축에서 시작한다. 끝날 때 앞면이 표적 뒷면의 팽창 경계 앞에 있어야 한다.
            limit = (blocker_along-fixed-motion-self.config.length_m/2)/p.transition_effective_fraction
            fitting = [t for t in self.registered_entry.transitions_m if t <= limit]
            rows[side] = dict(side=side, age_s=age, inner_normal_m=face['normal_m'], low_m=low, high_m=high,
                blocker_along_m=blocker_along, transition_limit_m=limit,
                transition_m=max(fitting) if fitting else min(self.registered_entry.transitions_m),
                width_m=high-low, want_offset_m=want, offset_m=sign*min(maximum, abs(want)), surface=face['method'])
        blocked = straight_blocked(rows, self.config)
        return dict(rows=rows, blocked=blocked, chosen=choose_side(rows) if blocked else None,
                    side_comparison_is_permission=False)

    def evaluate(self, scene, history, speed, previous, dt):
        self.corridor_use, self.motion_history = {}, history
        if self.path is None:
            result = self._first_selection(scene, history, speed, previous, dt)
        else:
            result = super().evaluate(scene, history, speed, previous, dt)
        return {**result, 'corridor':dict(self.corridor_use), 'corridor_plan':self.plan_detail}

    def _first_selection(self, scene, history, speed, previous, dt):
        plan = self._plan(scene)
        self.plan_detail = plan
        if plan is None or plan['chosen'] is None:
            return super().evaluate(scene, history, speed, previous, dt)
        row = plan['rows'][plan['chosen']]
        if row['transition_limit_m'] > max(self.registered_entry.transitions_m)+self.corridor_policy.start_slack_m:
            # 일찍 편이를 끝내면 옆면 띠가 비어 표적 앞에서 복귀가 시작된다. 그때까지 직진 후보만 본다.
            self.entry = replace(self.registered_entry, offsets_m=(0.,))
            try:
                return super().evaluate(scene, history, speed, previous, dt)
            finally:
                self.entry = self.registered_entry
        self.entry = replace(self.registered_entry, offsets_m=(0., row['offset_m']), transitions_m=(row['transition_m'],))
        self.intent.forced_offset = row['offset_m']
        try:
            result = super().evaluate(scene, history, speed, previous, dt)
        finally:
            self.entry = self.registered_entry
            self.intent.forced_offset = None
        if self.path is None:
            return result
        if self.corridor_use.get(self.side, {}).get('state', {}).get('stable'):
            return result
        # 저속 근거 없이 경로를 확정하지 않는다. 다음 호출에서 같은 근거를 다시 본다.
        self.path = self.wall_line = self.reference_s = None
        self.offset = self.transition = self.side = None
        self.wall_confirmations = 0
        self.intent.sign = None
        return self._hold('entry_inner_boundary_evidence_wait', reference=result.get('reference'),
                          guide_mode='new_observed_entry')
