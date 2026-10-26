"""release_to_baseline 관측 기반 관문 편집(1회용). 문자열이 정확히 한 번 있을 때만 바꾼다."""
from pathlib import Path

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
p = ROOT/'src/arena_autonomy/arena_autonomy/m3_execution_trial.py'
t = p.read_bytes().decode('utf-8')
assert '\r\n' not in t


def rep(old, new):
    global t
    assert t.count(old) == 1, old[:70]
    t = t.replace(old, new)


rep('''연속 외곽·숨은 차량 미래 진입·실차 제동/슬립은 미해결이며 `runtime_authorized`는 항상 false다.
"""''', '''6. **기본 G1 복귀(선택):** 측방 위협이 사라진 뒤, 마지막으로 위협이 보인 장면의 가능 몸체 영역을 운동 이력으로
   현재 좌표에 옮기고 이동 상계·거리 오차만큼 넓힌다. 그 영역을 지금 스캔의 양의 free ray로 확인하고(관측 표면과
   측방 창보다 앞에서 반환 뒤에 가려진 칸만 예외), 위협이 있을 수 있는 칸이 자차의 복귀 회랑·정지 거리 안에 없을
   때만 시험을 끝낸다. 반환 부재·점 개수 미달·fresh 2회만으로는 해제하지 않는다. fresh 2회는 깜박임 방지일 뿐이다.

연속 외곽·숨은 차량 미래 진입·실차 제동/슬립은 미해결이며 `runtime_authorized`는 항상 false다.
"""''')
rep('''from arena_autonomy.scene_motion import relative_poses
''', '''from arena_autonomy.scene_motion import relative_poses, transport_points
''')
rep('''    # 옆차가 자차 앞쪽으로 완전히 지나간 뒤 서로 다른 fresh 스캔 release_fresh회 동안 측방 위협이 없으면
    # 시험을 끝내고 활성 전과 같은 기본 G1 출력으로 돌려주는 비교 옵션. 중앙 복귀 완료를 해제 조건으로 쓰지 않는다.
    release_to_baseline: bool = False
''', '''    # 마지막 위협의 가능 영역을 지금 스캔으로 확인한 뒤에만 기본 G1 출력으로 돌려주는 비교 옵션(모듈 설명 6).
    # 해제 뒤에는 측방 시험의 관측 계약 밖이며, 기본 G1은 후방 마스크·측방 예산을 검사하지 않는다.
    release_to_baseline: bool = False
''')
rep('''def side_return_ds(frame, state, grid, cfg):
    """side_threats와 같은 창·벽·횡위치·compact 조건의 측방 반환별 종방향 위치 ds(좌, 우)."""
    if not len(grid.returns):
        return np.zeros(0), np.zeros(0)
    s, l = frame.project(grid.returns)
    ds = s-state['s0']
    window = (ds >= -cfg.band_behind_m) & (ds <= cfg.band_ahead_m)
    wall = np.zeros(len(l), dtype=bool)
    for _, sign in SIDES:
        wall |= np.abs(l-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m
    interior = np.abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m
    return tuple(ds[window & interior & ~wall & (sign*(l-state['d0']) >= state['half_span_m'])
                    & grid.compact_returns] for _, sign in SIDES)
''', '''def side_return_masks(frame, state, grid, cfg):
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
''')
rep('''class ExecutionLateralTrial:
    """측방 촉발·후보 선택 상태. 촉발·후보 순서는 동결 ObservedLateralTrial과 같고 직진 후보만 추가된다."""

    def __init__(self, cfg, execution):
        self.cfg, self.execution = cfg, execution
        self.reset()

    def reset(self):
        self.active = False
        self.clear_fresh = 0
        self.epoch = self.last_ref = self.last_source = None
        # 마지막으로 위협이 보인 판단에서 그 측방 반환 중 가장 뒤쪽의 ds. 해제 선택 옵션에서만 쓴다.
        self.last_threat_rear_ds = None

    def step(self, scene, frames, path, speed, follow_padding, command_check=None):
''', '''# 위협 가능 영역의 이산 표본 간격. 2 cm 외곽 표본과 같은 해상도이며 연속 영역 증명이 아니다.
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
    detail['record_age_s'] = float(age)
    if (old.source_epoch != scene.source_epoch or not scene.source_first_s > old.source_first_s
            or not 0 < age <= RELEASE_MAX_AGE_S or not len(record.points)):
        return {**detail, 'reason': 'record_stale_or_inconsistent'}
    bound = execution.history_motion_bound_mps
    reach = math.hypot(cfg.length_m, cfg.max_vehicle_width_m)
    body = _cells(record.points.min(axis=0)-reach, record.points.max(axis=0)+reach)
    body = body[_within(body, record.points, reach) & ~_inside_body(body, cfg)]
    body = body[~depth_support(old, body, ray_padding(old, cfg, bound))]
    try:
        current = transport_points(body, history, old.t_ref_s, scene.t_ref_s)
    except MotionUnavailable:
        return {**detail, 'reason': 'motion_unavailable'}
    grow = bound*(scene.t_ref_s-float(np.min(record.acquired)))+.025+cfg.budget.range_bias_m
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
    if np.any(blocked):
        return {**detail, 'reason': 'threat_region_unobserved'}
    # 영역 안에서 다시 보인 반환(벽 제외)으로 기록을 갱신한다. 그 반환이 같은 차라는 보장은 없다(보수적 추적).
    if len(hits) and len(region):
        _, hl = frame.project(hits)
        wall = np.zeros(len(hits), dtype=bool)
        for _, sign in SIDES:
            wall |= np.abs(hl-sign*cfg.wall_offset_m) <= cfg.wall_tolerance_m
        seen = _within(hits, region, RELEASE_GRID_M) & ~wall
    else:
        seen = np.zeros(len(hits), dtype=bool)
    rays = [r for r in scene.rays if r.status == 'return']
    acquired = np.asarray([r.acquired_at_s for r in rays], dtype=float)
    detail['record'] = (ThreatRecord(record.side, scene, hits[seen].copy(), acquired[seen].copy())
                        if np.any(seen) else None)
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
                if gate['ok']:
                    self.records[name] = gate['record']
                gates[name] = {k: v for k, v in gate.items() if k != 'record'}
        pending = [n for n, r in self.records.items() if r is not None and n not in gates]
        allowed = bool(not present and fresh and history is not None and self.clear_fresh >= self.cfg.release_fresh
                       and not pending and all(g['release'] for g in gates.values()))
        if present:
            reason = 'threat_present'
        elif self.clear_fresh < self.cfg.release_fresh:
            reason = 'debounce'
        else:
            reason = next((g['reason'] for g in gates.values() if not g['release']),
                          'not_fresh_or_no_history' if not (fresh and history is not None) else None)
        summary = {'release': allowed, 'reason': reason, 'sides': gates,
                   'ok': bool(gates) and all(g['ok'] for g in gates.values())}
        for key in ('blocked_cells', 'possible_cells', 'corridor_cells'):
            summary[key] = int(sum(g.get(key, 0) for g in gates.values()))
        return allowed, summary

    def step(self, scene, frames, path, speed, follow_padding, command_check=None, history=None):
''')
rep('''        light = ReturnGrid(scene, cfg)
        side_ds = side_return_ds(frame, state, light, cfg)
        if any(len(ds) for ds in side_ds):''', '''        light = ReturnGrid(scene, cfg)
        _, side_masks = side_return_masks(frame, state, light, cfg)
        if any(np.any(m) for m in side_masks):''')
rep('''        if self.execution.release_to_baseline:
            if present:
                rear = [float(np.min(ds)) for (name, _), ds in zip(SIDES, side_ds) if name in present and len(ds)]
                self.last_threat_rear_ds = min(rear) if rear else None
            out['last_threat_rear_ds_m'] = self.last_threat_rear_ds
            # 마지막에 보인 옆차 반환이 모두 자차 앞면보다 앞이면(앞으로 지나감) 해제한다. 뒤·옆에서 사라진 차는
            # 후방 마스크에 숨었을 수 있으므로 기존 시험을 유지한다. 관측·간격 기준은 바꾸지 않는다.
            if (not present and self.clear_fresh >= cfg.release_fresh and self.last_threat_rear_ds is not None
                    and self.last_threat_rear_ds >= cfg.length_m/2):
                self.active = False
                self.clear_fresh = 0
                self.last_threat_rear_ds = None
                return {**out, 'mode': 'BASELINE', 'selected': 'baseline', 'released_to_baseline': True}
''', '''        if self.execution.release_to_baseline:
            allowed, out['release_gate'] = self._release(scene, history, frame, state, light, side_masks, present, fresh)
            if allowed:
                self.active = False
                self.clear_fresh = 0
                self.records = {}
                return {**out, 'mode': 'BASELINE', 'selected': 'baseline', 'released_to_baseline': True}
''')
rep('''        self.lateral = ExecutionLateralTrial(self.lateral_config, self.execution)
''', '''        self.lateral = ExecutionLateralTrial(self.lateral_config, self.execution, c.max_speed_mps)
''')
rep('''            trial = self.lateral.step(scene, frames, baseline[2].get('path_points'), plan_speed, padding, command_check)
        else:
            trial = self.lateral.step(scene, frames, baseline[2].get('path_points'), plan_speed, padding)''',
    '''            trial = self.lateral.step(scene, frames, baseline[2].get('path_points'), plan_speed, padding, command_check,
                                      history=history)
        else:
            trial = self.lateral.step(scene, frames, baseline[2].get('path_points'), plan_speed, padding,
                                      history=history)''')
p.write_bytes(t.encode('utf-8'))
print('ok')
