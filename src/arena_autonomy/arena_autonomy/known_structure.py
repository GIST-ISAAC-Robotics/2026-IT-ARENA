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

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_path import braking_speed, swept_limit
from arena_autonomy.scene_motion import relative_poses, transport_points

POLICIES = ('off', 'structure_aware_v1')
ACCEPTED_HIDDEN_WIDTH_M = .05
MEMORY_LIMIT = 512
MEMORY_RANGE_M = 3.
SAMPLE_STEP_M = .03


def parse_sectors(text):
    """lidar_blind_sectors_json → ((중심°, 반폭°), ...). 비어 있으면 거절한다."""
    from arena_autonomy.local_scene import blind_spec
    _, sectors = blind_spec({'rear_half_angle_deg': 0., 'sectors_deg': json.loads(text)})
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


def sector_rays(angle_min, angle_increment, count, sectors):
    """mask_scan과 같은 센서축 각도 규칙으로 구조 각도별 광선 번호를 찾는다."""
    angles = angle_min + np.arange(count)*angle_increment
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
    for (center, width), rays in zip(sectors, sector_rays(scan.angle_min, scan.angle_increment, count, sectors)):
        if not len(rays):
            continue
        members = set(rays.tolist())
        flanks = sorted({(index + step) % count for index in members for step in (-1, 1)} - members)
        explained = bool(flanks) and bool(np.all(valid[flanks]))
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

    def evaluate(self, scan, points, history, now, speed, steering=0., previous_steering=0.,
                 wheelbase=.135, length=.20, width=.15):
        # 제한 없는 수치로 그림자 표본을 할당하지 않는다. 정상 주행 범위 밖 입력은 허가하지 않는다.
        if not math.isfinite(speed) or speed < 0 or speed > 10.:
            return dict(allowed=False, reason='structure_invalid_speed', speed_cap_mps=0.,
                        unknown_filled=False)
        valid, excused, rows = classify(scan, self.structure_sectors)
        self.structure_excused = excused
        self.structure_excused_indices = frozenset(np.flatnonzero(excused).tolist())
        try:
            result = super().evaluate(scan, points, history, now, speed, steering, previous_steering,
                                      wheelbase, length, width)
        finally:
            self.structure_excused = None
        source = scan.header.stamp.sec + scan.header.stamp.nanosec*1e-9
        # 기반 정책의 시간·스캔 무결성 거절(또는 되감기 초기화)은 그대로 둔다.
        if result.get('reason') in ('partial_invalid', 'partial_time_reset'):
            return {**result, 'known_structure': dict(policy='structure_aware_v1', evaluated=False)}
        try:
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
        stamps = source + rays*float(scan.time_increment)
        poses = relative_poses(history, stamps, now)
        c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
        origins = poses[:, :2] + np.column_stack((c*sensor.lidar_x_m - s*sensor.lidar_y_m,
                                                  s*sensor.lidar_x_m + c*sensor.lidar_y_m))
        directions = scan.angle_min + rays*scan.angle_increment + sensor.lidar_yaw_rad + poses[:, 2]
        step = float(scan.angle_increment)
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
                   previous_steering, wheelbase, length, width):
        fresh = source != self.structure_last_source
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
