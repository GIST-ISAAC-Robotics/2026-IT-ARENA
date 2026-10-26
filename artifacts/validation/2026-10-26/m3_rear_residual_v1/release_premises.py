"""M3 해제 관문의 전제별 미연결 시제품. 노드·launch·기본값에 연결하지 않는다.

`premise_release_gate(..., premise=ReleasePremise())`는 현행 `m3_execution_trial.release_gate`와 같은 계산을 한다
(모든 전제 False일 때 출력 동일을 저장 입력에서 확인한다). 전제를 켜면 일부 미확인 칸을 **차단/회랑 판정에서만**
제외하고, 영역 자체는 지우지 않고 다음 시각으로 계속 전파한다. 어느 전제도 관측으로 입증된 사실이 아니다.

- `rear_follow` (P-follow): 표본 중심이 현재 차체 후면 평면 뒤인 칸. 전진하는 자차 차체의 모든 점은 앞으로 움직이므로
  그 칸의 물체는 자차보다 빠르게 다가올 때만 접촉한다(운동학). 그 접근을 뒤차 책임 또는 기본 G1과 같은 후방 노출로 보는
  **결정**이다. 대신 가능 영역이 뒤 모서리 근처에 있으면 모서리 옆 영역 Z(후면 평면 앞·옆면 바깥)를 현재 장면에서
  직접 판정한다. 점 물체 모형에서는 Z에 미관측 점이 하나라도 있으면 차단한다(표본 격자 정렬에 의존하지 않음).
- `tail_min_width_m` (P-size, P-follow와 함께만 의미): Z에 지름 w 원판이 완전히 들어갈 수 없으면 Z 안 마스크 칸과 모서리
  판정을 차단에서 제외한다. w 미만 물체는 계약 밖이다. 근거(규정·실물 가림 지도) 없이 채택하지 않는다.
- `mask_equivalence` (P-base): 양옆 광선에 차체 가림 마스크가 있는(무응답 없는) 칸 전체를 기본 G1이 이미 검사하지 않는
  후방 노출과 같은 등급으로 제외한다. 가장 약한 전제다.
- `no_expiry_diagnostic`: 1 s 기록 만료를 끈 **진단 전용** 판본. 해결 후보가 아니다.
"""
from dataclasses import dataclass
import math

import numpy as np

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.observation_support import depth_support
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable


@dataclass(frozen=True)
class ReleasePremise:
    rear_follow: bool = False
    tail_min_width_m: object = None
    mask_equivalence: bool = False
    no_expiry_diagnostic: bool = False

    def __post_init__(self):
        if any(type(v) is not bool for v in (self.rear_follow, self.mask_equivalence, self.no_expiry_diagnostic)):
            raise ValueError('premise switches must be bool')
        if self.tail_min_width_m is not None:
            w = self.tail_min_width_m
            if isinstance(w, bool) or not isinstance(w, (int, float)) or not math.isfinite(w) or w <= 0:
                raise ValueError('tail width must be finite and positive')

    @property
    def active(self):
        return self.rear_follow or self.tail_min_width_m is not None or self.mask_equivalence


STRICT = ReleasePremise()
SAMPLE_RADIUS_M = mx.RELEASE_GRID_M/math.sqrt(2.)
DEBUG_HOOK = None   # 오프라인 진단 전용 관찰 함수. 판정에 영향을 주지 않는다.


def bracket_status(scene, points):
    """depth_support와 같은 양옆 광선 선택의 (유효, 마스크 포함, 무응답 포함)."""
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    directions = np.array([r.direction_xy for r in scene.rays])
    status = np.array([r.status for r in scene.rays])
    indices = np.array([r.source_index for r in scene.rays])
    valid = np.zeros(len(points), dtype=bool)
    masked = np.zeros(len(points), dtype=bool)
    unknown = np.zeros(len(points), dtype=bool)
    for k in range(0, len(points), 128):
        d = points[k:k+128, None, :]-origins[None, :, :]
        angle = np.arctan2(directions[None, :, 0]*d[:, :, 1]-directions[None, :, 1]*d[:, :, 0],
                           directions[None, :, 0]*d[:, :, 0]+directions[None, :, 1]*d[:, :, 1])
        low = np.where(angle >= -1e-10, np.maximum(angle, 0.), np.inf)
        high = np.where(angle <= 1e-10, np.maximum(-angle, 0.), np.inf)
        lo, hi = np.argmin(low, axis=1), np.argmin(high, axis=1)
        rows = np.arange(len(d))
        valid[k:k+128] = (np.isfinite(low[rows, lo]) & np.isfinite(high[rows, hi])
                          & (low[rows, lo]+high[rows, hi] <= .025) & (np.abs(indices[lo]-indices[hi]) <= 1))
        masked[k:k+128] = (status[lo] == 'masked') | (status[hi] == 'masked')
        unknown[k:k+128] = (status[lo] == 'unknown') | (status[hi] == 'unknown')
    return valid, masked, unknown


CORNER_REACH_M = .05     # 가능 영역 표본이 뒤 모서리에서 이 거리 안에 있으면 모서리 옆 영역을 직접 판정한다
CORNER_ZONE_DEPTH_M = .08   # 후면 평면 앞쪽 판정 범위(현행 60° 마스크의 꼬리 띠 길이 약 3 cm를 포함)
CORNER_ZONE_WIDTH_M = .06   # 차체 옆면 바깥 판정 범위
CORNER_FINE_M = .001


def behind_rear_plane(points, cfg):
    """표본 중심이 현재 차체 후면 평면 뒤인 칸(P-follow 범위). 평면 근처 표본의 표현 반경은 아래 모서리 판정이 맡는다."""
    return points[:, 0] < -cfg.length_m/2


def corner_zone_admits(scene, cfg, pad, sign, width, fine=CORNER_FINE_M, cache=None):
    """뒤 모서리 옆 영역 Z(후면 평면 앞·차체 옆면 바깥)에 물체가 들어갈 수 있는지.

    width=None은 점 물체 모형: Z 안 관측 자유가 아닌 미세 격자점이 하나라도 있으면 True. width=w는 Z 안에 완전히 들어가는
    지름 w 원판(관측 자유 점과 겹치지 않음)이 있으면 True. 후면 평면 뒤에서 걸친 물체는 P-follow 범위로 본다.
    미세 격자 중심만 보므로 자유를 놓치는 쪽(=가능하다고 보는 쪽)으로 근사하지만, 1 mm보다 얇은 미관측 틈은 놓칠 수 있다.
    """
    # 캐시는 장면 객체를 함께 붙잡아 id 재사용을 막고, 같은 객체일 때만 재사용한다(run2–4 판본의 id 단독 키 결함 수정).
    key = (id(scene), sign, width)
    if cache is not None and key in cache and cache[key][0] is scene:
        return cache[key][1]
    rear, half = -cfg.length_m/2, cfg.width_m/2
    xs = np.arange(rear, rear+CORNER_ZONE_DEPTH_M+fine/2, fine)
    ys = sign*np.arange(half+fine/2, half+CORNER_ZONE_WIDTH_M+fine/2, fine)
    grid = np.stack(np.meshgrid(xs, ys, indexing='ij'), axis=-1).reshape(-1, 2)
    free = depth_support(scene, grid, pad)
    if width is None:
        result = bool(np.any(~free))
    else:
        r = width/2
        lateral = sign*grid[:, 1]
        centers = grid[(grid[:, 0] >= rear+r) & (grid[:, 0] <= rear+CORNER_ZONE_DEPTH_M-r)
                       & (lateral >= half+r) & (lateral <= half+CORNER_ZONE_WIDTH_M-r)]
        blocked = grid[free]
        result = False
        if len(centers) and not len(blocked):
            result = True
        elif len(centers):
            for k in range(0, len(centers), 256):
                d = centers[k:k+256, None, :]-blocked[None, :, :]
                if np.any(np.min(d[..., 0]**2+d[..., 1]**2, axis=1) > r*r):
                    result = True
                    break
    if cache is not None:
        cache[key] = (scene, result)
    return result


_ZONE_CACHE = {}


def premise_release_gate(record, scene, history, frame, state, grid, cfg, execution, stop_distance, premise=STRICT,
                         confirmation_expiry=True):
    """`mx.release_gate`와 같은 계산 + 선택 전제의 차단/회랑 제외. 영역 보존 규칙은 바꾸지 않는다.

    `confirmation_expiry=False`는 아래 합집합 상태기 전용이다. 직전 갱신과의 간격(1 s)·세대·시각 검사는 유지한다.
    """
    detail = {'side': record.side, 'ok': False, 'release': False, 'reason': None, 'record': record}
    old = record.scene
    age = scene.t_ref_s-old.t_ref_s
    detection_age = scene.t_ref_s-(old.t_ref_s if record.detected_at_s is None else record.detected_at_s)
    confirmation_age = (detection_age if record.last_confirmed_at_s is None
                        else scene.t_ref_s-record.last_confirmed_at_s)
    detail['record_age_s'] = float(age)
    detail['detection_age_s'] = float(detection_age)
    detail['confirmation_age_s'] = float(confirmation_age)
    stale = (not 0 < age <= mx.RELEASE_MAX_AGE_S or not 0 <= confirmation_age <= mx.RELEASE_MAX_AGE_S)
    if premise.no_expiry_diagnostic or not confirmation_expiry:
        stale = not (0 < age <= mx.RELEASE_MAX_AGE_S and 0 <= confirmation_age)
    if (old.source_epoch != scene.source_epoch or not scene.source_first_s > old.source_first_s
            or stale or not len(record.points)):
        return {**detail, 'reason': 'record_stale_or_inconsistent'}
    bound = execution.history_motion_bound_mps
    if record.possible_xy_m is None:
        reach = math.hypot(cfg.max_vehicle_length_m, cfg.max_vehicle_width_m)
        body = mx._cells(record.points.min(axis=0)-reach, record.points.max(axis=0)+reach)
        body = body[mx._within(body, record.points, reach) & ~mx._inside_body(body, cfg)]
        body = body[~depth_support(old, body, mx.ray_padding(old, cfg, bound))]
        grow = bound*(scene.t_ref_s-float(np.min(record.acquired)))+.025+cfg.budget.range_bias_m
    else:
        body = record.possible_xy_m
        grow = bound*age+math.sqrt(2.)*mx.RELEASE_GRID_M
    if not len(body):
        return {**detail, 'reason': 'record_stale_or_inconsistent'}
    try:
        current = transport_points(body, history, old.t_ref_s, scene.t_ref_s)
    except MotionUnavailable:
        return {**detail, 'reason': 'motion_unavailable'}
    region = mx._cells(current.min(axis=0)-grow, current.max(axis=0)+grow)
    region = region[mx._within(region, current, grow+1e-9)]
    _, l = frame.project(region)
    region = region[(np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m) & ~mx._inside_body(region, cfg)]
    pad = mx.ray_padding(scene, cfg, bound)
    rest = region[~depth_support(scene, region, pad)] if len(region) else region
    s, l = frame.project(rest) if len(rest) else (np.zeros(0), np.zeros(0))
    ds = s-state['s0']
    returned = mx._both_sides_returned(scene, rest)
    hits = np.asarray(grid.returns, dtype=float).reshape(-1, 2)
    hit_pad = np.asarray([pad[i] for i, r in enumerate(scene.rays) if r.status == 'return'], dtype=float)
    if len(rest) and len(hits):
        dist = np.sqrt(((rest[:, None, :]-hits[None, :, :])**2).sum(axis=2))
        surface = np.any(dist <= hit_pad[None, :], axis=1)
    else:
        surface = np.zeros(len(rest), dtype=bool)
    blocked = ~(returned & (surface | (ds > cfg.band_ahead_m)))
    exempt = np.zeros(len(rest), dtype=bool)
    counts = {'exempt_rear': 0, 'exempt_tail_size': 0, 'exempt_mask': 0, 'corner_blocked': 0}
    corner_block = False
    if premise.active and len(rest):
        if len(_ZONE_CACHE) > 64:
            _ZONE_CACHE.clear()
        valid, masked, unknown = bracket_status(scene, rest)
        mask_only = valid & masked & ~unknown
        behind = behind_rear_plane(rest, cfg)
        if premise.mask_equivalence:
            counts['exempt_mask'] = int(np.sum(mask_only & blocked))
            exempt |= mask_only
        if premise.rear_follow:
            counts['exempt_rear'] = int(np.sum(behind & blocked & ~exempt))
            exempt |= behind
            if not premise.mask_equivalence:
                width = premise.tail_min_width_m
                alongside = mask_only & ~behind & ~exempt
                rear, half = -cfg.length_m/2, cfg.width_m/2
                for sign in (1., -1.):
                    corner = np.array([rear, sign*half])
                    near = np.hypot(rest[:, 0]-corner[0], rest[:, 1]-corner[1]) <= CORNER_REACH_M
                    if not np.any(near) and not np.any(alongside & (sign*rest[:, 1] > 0)):
                        continue
                    admits = corner_zone_admits(scene, cfg, pad, sign, width, cache=_ZONE_CACHE)
                    side = alongside & (sign*rest[:, 1] > 0)
                    if admits:
                        corner_block = True
                        counts['corner_blocked'] += 1
                    elif width is not None:
                        counts['exempt_tail_size'] += int(np.sum(side & blocked))
                        exempt |= side
    effective = blocked & ~exempt
    # 모서리 옆 직접 판정은 표본 위치와 무관한 차단 사유다. 칸 수에는 1로 더한다.
    blocked_count = int(np.sum(effective))+int(corner_block)
    if DEBUG_HOOK is not None:
        DEBUG_HOOK(scene=scene, rest=rest, blocked=blocked, exempt=exempt, effective=effective, ds=ds, l=l,
                   returned=returned, surface=surface, record=record, cfg=cfg, pad=pad, corner_block=corner_block)
    detail.update(region_cells=int(len(region)), possible_cells=int(len(rest)), blocked_cells=blocked_count,
                  grow_m=float(grow), body_cells=int(len(body)), raw_blocked_cells=int(np.sum(blocked)), **counts)
    if len(region):
        retained = rest if len(rest) else region
        confirmed_at = scene.t_ref_s if not blocked_count else record.last_confirmed_at_s
        detail['record'] = mx.ThreatRecord(record.side, scene, record.points, record.acquired, retained.copy(),
            old.t_ref_s if record.detected_at_s is None else record.detected_at_s, confirmed_at)
    else:
        detail['record'] = None
    detail['ok'] = True
    if blocked_count:
        return {**detail, 'reason': 'threat_region_unobserved'}
    sign = 1. if record.side == 'left' else -1.
    gap = cfg.budget.lateral_clearance(True, False, scene.t_ref_s-scene.source_first_s)[0]
    low = sign*state['d0']-state['half_span_m']
    high = max(sign*state['d0'], 0.)+state['half_span_m']+gap
    corridor = ((ds >= -cfg.length_m/2) & (ds <= cfg.length_m/2+stop_distance) & (sign*l >= low)
                & (sign*l <= high) & ~exempt)
    detail.update(ok=True, corridor_cells=int(np.sum(corridor)), stop_distance_m=float(stop_distance))
    if np.any(corridor):
        return {**detail, 'reason': 'threat_possible_in_return_corridor'}
    return {**detail, 'release': True, 'reason': 'threat_region_confirmed'}


# 합집합 상태기의 보수적 상한. 넘으면 해제하지 않고 세션 복귀를 보류한다(공간을 자유로 바꾸는 기준이 아니다).
UNION_MAX_LIFETIME_S = 20.0
UNION_MAX_CELLS_PER_SIDE = mx.RELEASE_MAX_CELLS_PER_SIDE


class UnionReleaseTrial(mx.ExecutionLateralTrial):
    """측면별 가능 영역 합집합 상태기(미연결 시제품).

    현행 장부는 fresh 위협 스캔마다 별도 기록을 만들고 각 기록에 '최초 탐지 후 1 s 안 전체 확인'을 요구한다.
    같은 차의 그림자가 측방 창에 1 s 넘게 남는 느린 통과에서는 확인 전에 만료·16기록 상한으로 세션이 잠긴다.
    여기서는 새 기록의 초기 가능 몸체를 다음 fresh 시각에 합집합으로 병합하고, 합집합 전체를 같은 이동 상계로
    전파한다. 어떤 기록도 새 반환으로 교체·삭제하지 않으며 영역은 양의 자유 관측으로만 줄어든다. 확인 만료 대신
    직전 갱신 간격 1 s·수명 20 s·칸 상한을 넘으면 보류한다. 전제는 `premise`로만 바뀐다(기본 엄격).
    """

    def __init__(self, cfg, execution, release_speed_mps=1., premise=STRICT):
        self.premise = premise
        super().__init__(cfg, execution, release_speed_mps)

    def reset(self):
        super().reset()
        self.union = {}      # side -> ThreatRecord(합집합 영역) 또는 None
        self.pending = {}    # side -> [아직 초기 몸체를 만들지 않은 새 기록]

    def _release(self, scene, history, frame, state, light, side_masks, present, fresh):
        gates = {}
        if fresh and history is not None:
            stop = mx.envelope_distance(self.release_speed_mps, scene.t_ref_s-scene.source_first_s, .02, self.execution)
            for name in sorted(set(self.union) | set(self.pending)):
                if name in self.release_memory_blocked:
                    continue
                records = ([self.union[name]] if self.union.get(name) is not None else [])+self.pending.get(name, [])
                if not records:
                    continue
                checks, cells, latched = [], [], None
                for record in records:
                    gate = premise_release_gate(record, scene, history, frame, state, light, self.cfg, self.execution,
                                                stop, self.premise, confirmation_expiry=False)
                    if not gate['ok']:
                        latched = gate['reason']
                        break
                    checks.append(gate)
                    if gate['record'] is not None:
                        cells.append(gate['record'].possible_xy_m)
                if latched is None:
                    detected = min(r.detected_at_s if r.detected_at_s is not None else r.scene.t_ref_s for r in records)
                    if scene.t_ref_s-detected > UNION_MAX_LIFETIME_S:
                        latched = 'union_lifetime_exceeded'
                    elif sum(len(c) for c in cells) > UNION_MAX_CELLS_PER_SIDE:
                        latched = 'release_memory_capacity_exceeded'
                if latched is not None:
                    self.release_memory_blocked[name] = latched
                    continue
                confirmed = all(c['blocked_cells'] == 0 for c in checks)
                previous = self.union.get(name)
                last = scene.t_ref_s if confirmed else (None if previous is None else previous.last_confirmed_at_s)
                if cells:
                    merged = np.vstack(cells)
                    # 반환점은 초기 몸체 생성과 비어 있지 않음 검사에만 쓰인다. 병합 뒤에는 영역이 기준이다.
                    first = records[0]
                    self.union[name] = mx.ThreatRecord(name, scene, first.points, first.acquired, merged, detected, last)
                    self.records[name] = self.union[name]
                else:
                    self.union[name] = None
                self.pending[name] = []
                side = {'side': name, 'ok': True, 'release': all(c['release'] for c in checks),
                        'reason': next((c['reason'] for c in checks if not c['release']), 'threat_region_confirmed'),
                        'records': len(checks), 'stop_distance_m': stop}
                for key in ('blocked_cells', 'possible_cells', 'corridor_cells', 'raw_blocked_cells', 'exempt_rear',
                            'exempt_tail_size', 'exempt_mask'):
                    side[key] = int(sum(c.get(key, 0) for c in checks))
                gates[name] = side
        for name, mask in zip(('left', 'right'), side_masks):
            if fresh and name in present and name not in self.release_memory_blocked:
                record = mx.threat_record(name, scene, light, mask)
                self.pending.setdefault(name, []).append(record)
                if self.union.get(name) is None:
                    self.records[name] = record
        for name, reason in self.release_memory_blocked.items():
            if fresh and history is not None:
                gates[name] = {'side': name, 'ok': False, 'release': False, 'reason': reason, 'records': 0}
        tracked = [n for n in set(self.union) | set(self.pending)
                   if self.union.get(n) is not None or self.pending.get(n)]
        pending = [n for n in tracked if n not in gates]
        confirmed = bool(not present and fresh and history is not None and gates and not pending
                         and not self.release_memory_blocked and all(g['release'] for g in gates.values()))
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
                   'ok': bool(gates) and all(g['ok'] for g in gates.values()),
                   'memory_records': sum((self.union.get(n) is not None)+len(self.pending.get(n, [])) for n in tracked),
                   'memory_blocked': dict(self.release_memory_blocked), 'union_premise': self.premise.__dict__.copy()}
        for key in ('blocked_cells', 'possible_cells', 'corridor_cells'):
            summary[key] = int(sum(g.get(key, 0) for g in gates.values()))
        if allowed:
            self.union, self.pending = {}, {}
        return allowed, summary


def install_union(controller, premise=STRICT):
    """M3ExecutionFollow의 측방 상태기를 합집합 시제품으로 바꾼다(오프라인 재생 전용)."""
    old = controller.lateral
    controller.lateral = UnionReleaseTrial(old.cfg, old.execution, old.release_speed_mps, premise)
    return controller


class use_premise:
    """`mx.release_gate`를 전제 판본으로 잠시 바꾸는 문맥. 오프라인 비교 전용."""

    def __init__(self, premise):
        self.premise = premise

    def __enter__(self):
        self.original = mx.release_gate
        premise = self.premise

        def gate(*args):
            return premise_release_gate(*args, premise=premise)
        mx.release_gate = gate
        return self

    def __exit__(self, *exc):
        mx.release_gate = self.original
        return False
