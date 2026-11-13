"""알려진 차체 고정 구조(장착 지주·뒤 마커)의 스캔 가림 선택 정책 structure_aware_v1.

입력은 센서 프로필에서 계산한 구조 각도(lidar_blind_sectors_json)뿐이다. 지도·정답 자세·
상대차 상태·심판 신호를 받지 않는다. 구조 각도 광선은 mask_scan에서 이미 NaN(unknown)이며,
이 정책도 거리를 만들거나 자유 공간으로 채우지 않는다. 바꾸는 것은 세 가지다.

1. 구분: 구조 각도 광선은 양쪽 바로 옆 광선이 모두 유효 반환일 때만 '설명된 구조 가림'으로
   일반 결측 예산(bounded_local_v4)에서 뺀다. 구조 밖 결측과 옆이 끊긴 구조 각도는 기존 v4가
   그대로 판정한다(속도 1.2 m/s 초과 불허·6°/3° 예산·이전 점 래치 포함).
2. 그림자 기하: 구조 광선의 실제 취득 방향(운동 보정)으로 그림자를 표본화한다. 물체가 완전히
   숨을 수 있는 폭이 accepted_hidden_width_m를 넘는 먼 그림자가 현재/직전 조향의 이동 외곽에
   들어오면, 그 그림자 앞에서 멈출 수 있는 속도로만 제한한다(점군·경로에는 넣지 않는다).
3. 숨은 기억: 직전 스캔에 보였던 반환이 이번 스캔의 구조 그림자로 들어가면 정지 물체로 가정해
   자차 운동으로만 옮겨 기억한다. 그림자를 벗어나면 현재 스캔이 판단한다. 기억 점이 이동 외곽에
   들어오면 그 앞에서 멈출 수 있는 속도로 제한한다.

남는 위험(측정 대상, 안전 보증 아님): 그림자 안에서 accepted_hidden_width_m보다 좁고 이전에
보이지 않은 물체, 그림자 안에서 움직이는 물체(기억은 정지 가정), 부분적으로 보이는 물체의
숨은 부분(보이는 옆면에서 최대 accepted_hidden_width_m). 5 cm는 v4 측방 결측의 기존 시험
선례(maximum_chord_m)이며 제조사/규정값이 아니다.
"""
import json
import math

import numpy as np
from arena_autonomy.scan_input import scan_first_stamp

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_path import braking_speed, swept_limit
from arena_autonomy.scene_motion import relative_poses, transport_points

POLICIES = ('off', 'structure_aware_v1')
ACCEPTED_HIDDEN_WIDTH_M = .05
MEMORY_LIMIT = 512
MEMORY_RANGE_M = 3.
SAMPLE_STEP_M = .03


def _current_mask_query_batches(queries, origins, directions):
    """전체 광선 bracket을 원 연산/동률 규칙으로 최대 32질의씩 계산한다."""
    for start in range(0, len(queries), 32):
        dx = queries[start:start+32, None, 0]-origins[None, :, 0]
        dy = queries[start:start+32, None, 1]-origins[None, :, 1]
        # 2원소 norm의 multiply/add/sqrt 순서를 유지한다. hypot나 제곱 임계값으로
        # 바꾸지 않으며 dot의 +0은 np.sum([-0., -0.])의 양의 0도 보존한다.
        radius = np.sqrt(dx*dx + dy*dy)
        cross = directions[None, :, 0]*dy-directions[None, :, 1]*dx
        dot = directions[None, :, 0]*dx + directions[None, :, 1]*dy + 0.
        angles = np.arctan2(cross, dot)
        lows = np.argmin(np.where(angles >= -1e-10, np.maximum(angles, 0.), np.inf), axis=1)
        highs = np.argmin(np.where(angles <= 1e-10, np.maximum(-angles, 0.), np.inf), axis=1)
        yield radius, angles, lows, highs


def current_mask_shadow(scene, scan, queries, padding, sectors):
    """현재 선언 구조 그림자의 폭/양옆 깊이로 위험을 제한한다. free 증거가 아니다.

    차체의 횡이동량과 숨을 수 있는 물체 폭은 서로 다른 양이다. 여기서는
    기존 structure_aware_v1의 5 cm 숨은 폭을 재사용한다. 거리 방향 패딩은
    양옆 반환 깊이에서 빼며, 가림 폭을 계산할 질의 위치를 이동시키지 않는다.
    실제 점유 sweep과 숨은 기억은 호출 전 기하 검사/독립 보호층에 남는다.
    """
    from arena_autonomy.observation_support import source_valid
    result = dict(accepted=False, policy='current_declared_structure_shadow_v1',
        unknown_filled=False, residual_hidden_object_risk=True,
        accepted_hidden_width_m=ACCEPTED_HIDDEN_WIDTH_M, memory_rechecked_here=False,
        independent_guard_required=True)

    def reject(reason):
        return {**result, 'reason': reason}

    q = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = np.asarray(padding, dtype=float)
    rays = scene.rays
    if (not source_valid(scene) or not len(q) or not np.all(np.isfinite(q))
            or pad.shape != (len(rays),) or not np.all(np.isfinite(pad)) or np.any(pad < 0)):
        return reject('invalid_current_mask_queries')
    _, _, rows = classify(scan, sectors)
    membership = np.zeros(len(rays), dtype=int)
    for row in rows:
        if row['explained']:
            membership[row['rays']] += 1
    origins = np.asarray([r.ray_origin_xy_m for r in rays])
    directions = np.asarray([r.direction_xy for r in rays])
    status = np.asarray([r.status for r in rays])
    hits = np.asarray([r.hit_xy_m for r in rays if r.status == 'return']).reshape(-1, 2)
    valid = status == 'return'
    # 일부만 보이는 물체의 실제 옆면을 구조 그림자라는 이유로 제외하지 않는다.
    for start in range(0, len(q), 64):
        if len(hits) and np.any(np.linalg.norm(q[start:start+64, None]-hits[None], axis=2) <= pad[valid]):
            return reject('current_known_occupied_padding')
    depths = np.asarray([math.dist(r.ray_origin_xy_m, r.hit_xy_m) if r.status == 'return'
                         else -1. for r in rays])
    groups = {}
    components = {}
    # 같은 구간을 모든 질의마다 다시 탐색하지 않는다. 안쪽 취득 광선의
    # 방향/원점 변화도 포함해 끝 두 광선만으로 폭을 축소하지 않는다.
    i = 0
    while i < len(rays):
        if status[i] != 'masked':
            i += 1
            continue
        a = i
        while i+1 < len(rays) and status[i+1] == 'masked': i += 1
        b = i
        explained = (a > 0 and b+1 < len(rays) and status[a-1] == 'return'
            and status[b+1] == 'return' and np.all(membership[a:b+1] == 1)
            and any(row['explained'] and set(range(a,b+1)).issubset(set(row['rays'])) for row in rows))
        component = None
        if explained:
            local = directions[a-1:b+2]
            bearings = np.unwrap(np.arctan2(local[:,1], local[:,0]))
            angle = float(np.ptp(bearings))
            positions = origins[a-1:b+2]
            span = float(np.max(np.linalg.norm(positions[:,None]-positions[None],axis=2)))
            component = (a,b,angle,span)
        for j in range(a,b+1): components[j] = component
        i += 1
    # 광선 전체와 각 취득 원점은 유지한다. 최대 32질의의 계산만 배열화한다.
    # 첫 실패 질의와 그 질의의 기존 거절 우선순위는 아래 masks 순서로 보존한다.
    by_id = list(dict.fromkeys(c for c in components.values() if c is not None))
    ids = {c: i for i, c in enumerate(by_id)}
    component_ids = np.full(len(rays), -1, dtype=int)
    for ray, component in components.items():
        if component is not None:
            component_ids[ray] = ids[component]
    available = depths-pad
    for radius, angles, lo, hi in _current_mask_query_batches(q, origins, directions):
        row = np.arange(len(radius))
        low_angles, high_angles = angles[row, lo], angles[row, hi]
        bracket_bad = (np.any(radius <= 1e-9, axis=1) | (low_angles < -1e-10)
            | (high_angles > 1e-10) | (np.abs(lo-hi) > 1)
            | (np.abs(low_angles)+np.abs(high_angles) > .025))
        low_masked, high_masked = status[lo] == 'masked', status[hi] == 'masked'
        low_return, high_return = valid[lo], valid[hi]
        status_bad = (~(low_masked | high_masked) | ~(low_masked | low_return)
            | ~(high_masked | high_return))
        pair_near = ((low_return & (available[lo] <= radius[row, lo]))
            | (high_return & (available[hi] <= radius[row, hi])))
        selected = component_ids[np.where(low_masked, lo, hi)]
        unexplained = selected < 0
        wide, flank_near = np.zeros(len(row), dtype=bool), np.zeros(len(row), dtype=bool)
        reaches, widths = np.zeros(len(row)), np.zeros(len(row))
        # 인접한 두 masked 광선은 반드시 같은 연속 component다. 인접하지 않은
        # 질의는 bracket_bad로 먼저 거절하므로 서로 다른 성분을 합치지 않는다.
        ordered = list(dict.fromkeys(selected.tolist()))
        for index in ordered:
            if index < 0:
                continue
            a, b, angle, origin_span = by_id[index]
            chosen = np.flatnonzero(selected == index)
            reach = np.max(radius[chosen, a-1:b+2], axis=1)
            width = 2*reach*math.sin(angle/2) + origin_span
            reaches[chosen], widths[chosen] = reach, width
            wide[chosen] = (angle >= math.pi/2) | (width > ACCEPTED_HIDDEN_WIDTH_M)
            flank_near[chosen] = ((available[a-1] <= radius[chosen, a-1])
                | (available[b+1] <= radius[chosen, b+1]))
        masks = (bracket_bad, status_bad, pair_near, unexplained, wide, flank_near)
        reasons = ('unbracketed_current_mask_query', 'not_declared_mask_only',
            'near_current_mask_flank', 'unexplained_current_mask_component',
            'wide_current_structure_shadow', 'near_current_mask_flank')
        failed = np.flatnonzero(np.logical_or.reduce(masks))
        if len(failed):
            first_bad = failed[0]
            return reject(next(reason for reason, mask in zip(reasons, masks) if mask[first_bad]))
        for index in ordered:
            a, b, _, _ = by_id[index]
            chosen = np.flatnonzero(selected == index)
            # argmax의 첫 동률 선택과 batch 간 strict > 갱신으로 원 질의 순서 유지.
            best = chosen[np.argmax(widths[chosen])]
            width, reach = float(widths[best]), float(reaches[best])
            key = (a, b)
            old = groups.get(key)
            if old is None or width > old['hidden_width_m']:
                groups[key] = dict(first=a, last=b, hidden_width_m=width,
                    flank_ranges_m=depths[[a-1, b+1]].tolist(), query_reach_m=reach)
    return {**result, 'accepted': True, 'reason': 'bounded_current_structure_shadow_risk',
            'queries': len(q), 'components': list(groups.values())}


def parse_sectors(text):
    """lidar_blind_sectors_json → ((중심°, 반폭°), ...). 비어 있으면 거절한다."""
    from arena_autonomy.local_scene import blind_spec,blind_parameter
    _, sectors = blind_spec(blind_parameter(0.,text))
    if not sectors:
        raise ValueError('structure policy requires known structure sectors')
    return sectors


def validate(policy, sectors_json):
    """두 노드가 같은 규칙으로 검사한다. off에서는 각도 문자열을 받지 않는다."""
    if policy not in POLICIES:
        raise ValueError('unknown known structure policy')
    if policy == 'off':
        if sectors_json:
            raise ValueError('structure sectors require an explicit structure policy')
        return ()
    return parse_sectors(sectors_json)


def sector_rays(angle_min, angle_increment, count, sectors, *, source_angles=None):
    """mask_scan과 같은 센서축 각도 규칙으로 구조 각도별 광선 번호를 찾는다."""
    angles = angle_min + np.arange(count)*angle_increment if source_angles is None else source_angles
    rows = []
    for center, width in sectors:
        offset = np.remainder(angles - math.radians(center) + math.pi, math.tau) - math.pi
        rows.append(np.flatnonzero(np.abs(offset) <= math.radians(width) + 1e-12))
    return rows


def classify(scan, sectors):
    """구조 광선과 '양옆이 유효 반환인' 설명 가능 여부. 거리 배열은 바꾸지 않는다."""
    values = np.asarray(scan.ranges, dtype=float)
    count = len(values)
    valid = np.isfinite(values) & (values >= scan.range_min) & (values <= scan.range_max)
    excused = np.zeros(count, dtype=bool)
    rows = []
    from arena_autonomy.scan_input import scan_angles
    from arena_autonomy.measurement_scan import MeasurementScan
    native=isinstance(scan,MeasurementScan)
    selected=sector_rays(0.,0.,count,sectors,source_angles=scan_angles(scan))
    for (center, width), rays in zip(sectors, selected):
        if not len(rays):
            continue
        members = set(rays.tolist())
        flanks = sorted({(index + step) % count for index in members for step in (-1, 1)} - members)
        explained = bool(flanks) and bool(np.all(valid[flanks]))
        if native and (0 in members or count-1 in members):
            # 원천 회전 경계의 양끝을 자동으로 인접 반환으로 인정하지 않는다.
            explained=False
        if explained:
            excused[rays] = True
        rows.append(dict(center_deg=center, half_deg=width, rays=rays, flanks=flanks, explained=explained))
    return valid, excused, rows


def structure_policy(base, sectors_json):
    """기존 부분 관측 정책 클래스(v4 또는 교통 회복 하위 클래스)에 구조 정책을 얹는다."""
    sectors = parse_sectors(sectors_json)
    return type('StructureAware' + base.__name__, (StructureAwareMixin, base),
                {'structure_sectors': sectors, 'structure_sectors_json': sectors_json})


class StructureAwareMixin:
    structure_sectors = ()
    accepted_hidden_width_m = ACCEPTED_HIDDEN_WIDTH_M

    def reset(self):
        super().reset()
        # 기반 정책이 읽는 설명 가능 구조 광선(같은 호출 안에서만 유효).
        self.structure_excused = None
        self.structure_excused_indices = frozenset()
        self.structure_previous = None
        self.structure_hidden = None
        self.structure_last_source = None
        self.structure_active_call = None
        self.structure_precomputed = None

    def _refine_threat(self, threatening, carried, *args):
        """이미 구조 기억에 보존한 뒤쪽 점은 그 기억의 이동 외곽 검사에 맡긴다.

        설명된 차체 사각 안·차체 뒤끝 밖의 실제 기억만 다룬다.
        2cm 안 기존 기억과 대응하거나, 유효한 옛 반환 자체를 추가해 재검사한다.
        전방/일반 결측은 바꾸지 않고, 숨은 기억의 정지 가정 한계는 유지한다.
        """
        threat = super()._refine_threat(threatening, carried, *args)
        active = self.structure_active_call
        if active is None or not np.any(threat):
            return threat
        length = active[-2]
        proposed = np.any(threat, axis=0) & (carried[:, 0] < -length/2-.025)
        if not np.any(proposed):
            return threat
        scan, _, rows, _, history, now, source = active[:7]
        try:
            shadows = self._shadow(scan, rows, history, now, source)
        except (MotionUnavailable, ValueError, FloatingPointError):
            # 조기 계산 실패는 위협을 면제하지 않는다. evaluate의 기존 구조 오류
            # 처리가 최종 거절을 기록하도록 원 위협을 유지한다.
            return threat
        proposed &= self._inside(carried, [s for s in shadows if s['explained']])
        if not np.any(proposed):
            return threat
        # 같은 호출의 구조 판정은 여기서 한 번만 수행하고 evaluate에서 재사용한다.
        try:
            self.structure_precomputed = self._structure(*active, shadows=shadows)
        except (MotionUnavailable, ValueError, FloatingPointError):
            return threat
        hidden, reference = self.structure_hidden
        if abs(reference-now)>1e-9:
            return threat
        ids = np.flatnonzero(proposed)
        distances = (np.min(np.linalg.norm(carried[ids, None, :]-hidden[None, :, :], axis=2), axis=1)
                     if len(hidden) else np.full(len(ids), np.inf))
        # 일반 소실 기억은 중간 스캔의 속도/관측 거절 때문에 구조 기억보다 한
        # 회전 오래될 수 있다. 거리 허용치를 늘려 다른 점과 같다고 하지 않고,
        # 실제 보관 중인 옛 반환 자체를 같은 후방 그림자의 장애물 기억에 더한다.
        previous = args[0] if len(args) > 1 else None
        retained = (previous is not None and
                    (previous is self.previous or previous is self.previous_distinct))
        extra = np.zeros(0, dtype=int)
        if (retained and len(previous[0]) == len(carried) and
                0 <= now-previous[1] <= .3 and previous[2] < source):
            extra = ids[(distances > .02) & (np.linalg.norm(carried[ids], axis=1) <= MEMORY_RANGE_M)]
        if len(extra) and len(hidden)+len(extra) <= MEMORY_LIMIT:
            saved = self.structure_precomputed
            self.structure_hidden = (np.vstack((hidden, carried[extra])), now)
            try:
                checked = self._structure(*active, shadows=shadows)
            except (MotionUnavailable, ValueError, FloatingPointError):
                self.structure_hidden = (hidden, reference)
                self.structure_precomputed = saved
                return threat
            # 추가한 실제 점도 현재/직전 조향과 제동 외곽으로 반드시 다시 검사한다.
            # 기억을 버리거나 새 점으로 기존 512개를 밀어내지는 않는다.
            self.structure_precomputed = {**checked, 'fresh': saved['fresh'],
                'hidden_memory_added': saved['hidden_memory_added']+len(extra),
                'hidden_memory_released': saved['hidden_memory_released'],
                'retained_previous_shadow_points': len(extra)}
            hidden, reference = self.structure_hidden
            distances = np.min(np.linalg.norm(carried[ids, None, :]-hidden[None, :, :], axis=2), axis=1)
        delegated = ids[distances <= .02]
        if not len(delegated):
            return threat
        threat = threat.copy()
        threat[:, delegated] = False
        report = args[-1]
        report['structure_rear_memory_transfer'] = dict(points=len(delegated),
            maximum_memory_match_m=float(np.max(distances[distances <= .02])),
            scope='rear_known_structure_memory_not_free_space',
            stationary_memory_assumption=True, unknown_filled=False)
        return threat

    def evaluate(self, scan, points, history, now, speed, steering=0., previous_steering=0.,
                 wheelbase=.135, length=.20, width=.15):
        # 제한 없는 수치로 그림자 표본을 할당하지 않는다. 정상 주행 범위 밖 입력은 허가하지 않는다.
        if not math.isfinite(speed) or speed < 0 or speed > 10.:
            return dict(allowed=False, reason='structure_invalid_speed', speed_cap_mps=0.,
                        unknown_filled=False)
        valid, excused, rows = classify(scan, self.structure_sectors)
        self.structure_excused = excused
        self.structure_excused_indices = frozenset(np.flatnonzero(excused).tolist())
        source = scan_first_stamp(scan)
        self.structure_precomputed = None
        self.structure_active_call = (scan, valid, rows, points, history, now, source, speed,
                                      steering, previous_steering, wheelbase, length, width)
        try:
            result = super().evaluate(scan, points, history, now, speed, steering, previous_steering,
                                      wheelbase, length, width)
        finally:
            self.structure_excused = None
            self.structure_active_call = None
        # 기반 정책의 시간·스캔 무결성 거절(또는 되감기 초기화)은 그대로 둔다.
        if result.get('reason') in ('partial_invalid', 'partial_time_reset'):
            return {**result, 'known_structure': dict(policy='structure_aware_v1', evaluated=False)}
        try:
            structure = self.structure_precomputed
            if structure is None:
                structure = self._structure(scan, valid, rows, points, history, now, source, speed,
                                            steering, previous_steering, wheelbase, length, width)
        except (MotionUnavailable, ValueError, FloatingPointError) as error:
            return {**result, 'allowed': False, 'reason': 'structure_motion_unavailable',
                    'known_structure': dict(policy='structure_aware_v1', evaluated=False, error=str(error))}
        report = {**result, 'known_structure': structure}
        cap = structure['speed_cap_mps']
        if not result.get('allowed') or cap is None:
            return report
        previous = result.get('speed_cap_mps')
        report['speed_cap_mps'] = cap if previous is None else min(previous, cap)
        if cap < speed:
            report['reason'] = 'structure_shadow_limited'
            report['underlying_reason'] = result.get('reason')
        return report

    def _shadow(self, scan, rows, history, now, source):
        """구조 각도별 실제 취득 방향의 그림자(원점, 시작/끝 방향). 반환을 만들지 않는다."""
        rays = np.concatenate([row['rays'] for row in rows]) if rows else np.zeros(0, dtype=int)
        if not len(rays):
            return []
        sensor = history.config
        from arena_autonomy.scan_input import scan_angles, scan_times, scan_max_gap
        from arena_autonomy.measurement_scan import MeasurementScan
        stamps = scan_times(scan)[rays]
        poses = relative_poses(history, stamps, now)
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        origins = poses[:, :2] + np.column_stack((c*sensor.lidar_x_m - s*sensor.lidar_y_m,
                                                  s*sensor.lidar_x_m + c*sensor.lidar_y_m))
        directions = scan_angles(scan)[rays] + sensor.lidar_yaw_rad + poses[:, 2]
        step = scan_max_gap(scan)
        shadows, start = [], 0
        for row in rows:
            stop = start + len(row['rays'])
            # 센서축 구조 각도의 연속 광선. 시작 방향을 기준으로 펴서 ±π 경계를 다룬다.
            local = directions[start:stop]
            unwrapped = local[0] + np.remainder(local - local[0] + math.pi, math.tau) - math.pi
            low, high = float(np.min(unwrapped)) - step/2, float(np.max(unwrapped)) + step/2
            # 숨을 수 있는 폭 = 2 r sin(각폭/2). 이 거리보다 먼 그림자만 경로 제한에 쓴다.
            half = (high - low)/2
            shadows.append(dict(origin=np.mean(origins[start:stop], axis=0), low=low, high=high,
                                explained=row['explained'], center_deg=row['center_deg'],
                                hidden_width_range_m=self.accepted_hidden_width_m/(2*math.sin(half))))
            start = stop
        return shadows

    @staticmethod
    def _inside(points, shadows):
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        inside = np.zeros(len(points), dtype=bool)
        for shadow in shadows:
            delta = points - shadow['origin']
            bearing = np.arctan2(delta[:, 1], delta[:, 0])
            offset = np.remainder(bearing - shadow['low'] + math.pi, math.tau) - math.pi
            inside |= (offset >= 0) & (offset <= shadow['high'] - shadow['low'])
        return inside

    def _structure(self, scan, valid, rows, points, history, now, source, speed, steering,
                   previous_steering, wheelbase, length, width, *, shadows=None):
        fresh = source != self.structure_last_source
        if shadows is None:
            shadows = self._shadow(scan, rows, history, now, source)
        current = np.asarray(points, dtype=float).reshape(-1, 2)
        current = current[np.all(np.isfinite(current), axis=1)]
        added = released = 0
        hidden = np.zeros((0, 2))
        if self.structure_hidden is not None:
            stored, reference = self.structure_hidden
            hidden = transport_points(stored, history, reference, now) if len(stored) else stored
        if fresh:
            if len(hidden):
                # 그림자를 벗어난 기억은 현재 스캔이 다시 판단한다.
                keep = self._inside(hidden, shadows) & (np.linalg.norm(hidden, axis=1) <= MEMORY_RANGE_M)
                released = int(np.count_nonzero(~keep))
                hidden = hidden[keep]
            if self.structure_previous is not None and len(self.structure_previous[0]):
                old, reference = self.structure_previous
                carried = transport_points(old, history, reference, now)
                entered = self._inside(carried, shadows)
                if len(current) and np.any(entered):
                    nearest = np.min(np.linalg.norm(carried[entered][:, None, :] - current[None, :, :], axis=2), axis=1)
                    # 2 cm 안에서 다시 관측된 점은 숨은 점이 아니다(v4 재관측 대응 거리와 같다).
                    entered[np.flatnonzero(entered)[nearest <= .02]] = False
                added = int(np.count_nonzero(entered))
                hidden = np.vstack((hidden, carried[entered]))[-MEMORY_LIMIT:]
            self.structure_previous = (current[np.linalg.norm(current, axis=1) <= MEMORY_RANGE_M].copy(), now)
            self.structure_last_source = source
        self.structure_hidden = (hidden, now)
        age = max(0., now - source)
        horizon = speed*(age + .10) + speed*speed/4 + .06 + .04
        steerings = sorted({float(steering), float(previous_steering)})
        far = []
        for shadow in shadows:
            radius = shadow['hidden_width_range_m']
            # 먼 그림자 표본. 거리 상한은 이번 속도의 제동 지평과 차체 대각선.
            reach = horizon + math.hypot(length/2, width/2) + .05
            if radius >= reach:
                continue
            ranges = np.arange(radius, reach + 1e-9, SAMPLE_STEP_M)
            # 넓은 뒤 마커 그림자도 내부를 빠뜨리지 않게 최대 3 cm 각방향 간격으로 표본화한다.
            angles = np.linspace(shadow['low'], shadow['high'],
                                 max(3, math.ceil((shadow['high']-shadow['low'])*reach/SAMPLE_STEP_M)+1))
            for angle in angles:
                far.append(shadow['origin'] + ranges[:, None]*np.array([math.cos(angle), math.sin(angle)]))
        far = np.vstack(far) if far else np.zeros((0, 2))
        # 차체 안에는 외부 물체가 놓일 수 없다. 뒤 마커의 넓은 그림자는 5 cm 폭에
        # 도달하는 지점 자체가 차체 안일 수 있으므로 현재 차체+충돌 여유 밖만 검사한다.
        # 실제 관측했던 장애물 기억은 이 제거 대상이 아니다.
        if len(far):
            far = far[(np.abs(far[:, 0]) > length/2 + .025) |
                      (np.abs(far[:, 1]) > width/2 + .025)]
        def clearance(cloud):
            if not len(cloud):
                return None
            return min(swept_limit(cloud, angle, wheelbase, horizon=horizon, length=length, width=width)
                       for angle in steerings)
        far_clearance, memory_clearance = clearance(far), clearance(hidden)
        caps = {}
        for name, value in (('shadow', far_clearance), ('memory', memory_clearance)):
            if value is not None and value < horizon - 1e-9:
                caps[name] = braking_speed(value, age)
        cap = min(caps.values()) if caps else None
        return dict(policy='structure_aware_v1', evaluated=True, fresh=fresh,
                    sectors=len(rows), explained_sectors=sum(row['explained'] for row in rows),
                    unexplained_sector_deg=[row['center_deg'] for row in rows if not row['explained']],
                    hidden_memory_points=len(hidden), hidden_memory_added=added, hidden_memory_released=released,
                    shadow_clearance_m=far_clearance, memory_clearance_m=memory_clearance,
                    shadow_cap_mps=caps.get('shadow'), memory_cap_mps=caps.get('memory'),
                    speed_cap_mps=cap, horizon_m=horizon, accepted_hidden_width_m=self.accepted_hidden_width_m,
                    unknown_filled=False,
                    scope='known_structure_shadow_bounded_not_free_space')
