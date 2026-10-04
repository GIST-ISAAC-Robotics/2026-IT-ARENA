"""G0 평가 전용 기하. 도로·actor 정답을 대회 제어기에 전달하지 않는다."""
from dataclasses import dataclass
import math


def finite(value):
    result = float(value)
    if isinstance(value, bool) or not math.isfinite(result):
        raise ValueError('finite numeric value required')
    return result


@dataclass(frozen=True, slots=True)
class Footprint:
    length_m: float = .20
    width_m: float = .15

    def __post_init__(self):
        if finite(self.length_m) <= 0 or finite(self.width_m) <= 0:
            raise ValueError('footprint dimensions must be positive')

    def lateral_width(self, yaw_rad):
        yaw = finite(yaw_rad)
        return abs(self.width_m * math.cos(yaw)) + abs(self.length_m * math.sin(yaw))

    def polygon(self, x_m=0., y_m=0., yaw_rad=0.):
        x, y, yaw = map(finite, (x_m, y_m, yaw_rad))
        c, s = math.cos(yaw), math.sin(yaw)
        return tuple((x + c * a - s * b, y + s * a + c * b) for a, b in
                     ((-self.length_m/2, -self.width_m/2), (self.length_m/2, -self.width_m/2),
                      (self.length_m/2, self.width_m/2), (-self.length_m/2, self.width_m/2)))


@dataclass(frozen=True, slots=True)
class TruthBox:
    """렌더링/평가 전용. production LocalScene에 존재하지 않는 정답 필드."""
    x_m: float
    y_m: float
    height_m: float
    footprint: Footprint = Footprint()
    yaw_rad: float = 0.
    bottom_m: float = 0.
    label: str = 'target'

    def __post_init__(self):
        for value in (self.x_m, self.y_m, self.height_m, self.yaw_rad, self.bottom_m):
            finite(value)
        if self.height_m <= 0 or not isinstance(self.footprint, Footprint):
            raise ValueError('invalid truth box')


def rectangle_section_y(polygon, x_m):
    """직선 x와 볼록 사각형의 단면. 교차하지 않으면 None."""
    x = finite(x_m)
    ys = []
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        if min(a[0], b[0]) - 1e-12 <= x <= max(a[0], b[0]) + 1e-12:
            if abs(b[0] - a[0]) <= 1e-12:
                if abs(x - a[0]) <= 1e-12:
                    ys.extend((a[1], b[1]))
            else:
                u = (x - a[0]) / (b[0] - a[0])
                if -1e-12 <= u <= 1 + 1e-12:
                    ys.append(a[1] + u * (b[1] - a[1]))
    return (min(ys), max(ys)) if ys else None


def cross_section_gaps(target=None, *, road_width_m=.45, ego=Footprint(),
                       ego_yaw_rad=0., padding_m=.025, section_x_m=None):
    """폭의 필요조건만 계산한다. 경로·swept·진입/복귀 허가는 계산하지 않는다.

    기본은 target 전체 횡방향 envelope다. section_x_m 지정 시 해당 단면도
    계산한다. padding은 ego 양쪽에 한 번씩 적용하는 기존 임시 .025 m이다.
    """
    road, padding = finite(road_width_m), finite(padding_m)
    if road <= 0 or padding < 0:
        raise ValueError('invalid road width/padding')
    required = ego.lateral_width(ego_yaw_rad) + 2 * padding
    lo, hi = -road/2, road/2
    interval = None
    polygon = ()
    if target is not None:
        polygon = target.footprint.polygon(target.x_m, target.y_m, target.yaw_rad)
        interval = ((min(y for _, y in polygon), max(y for _, y in polygon))
                    if section_x_m is None else rectangle_section_y(polygon, section_x_m))
    if interval is None or interval[1] < lo or interval[0] > hi:
        left_gap = right_gap = road
    else:
        left_gap = max(0., hi - max(lo, interval[1]))
        right_gap = max(0., min(hi, interval[0]) - lo)
    left, right = left_gap - required, right_gap - required
    return {'evaluation_only': True, 'scope': 'cross_section_necessary_condition_only',
            'section_x_m': section_x_m, 'road_interval_y_m': [lo, hi],
            'target_interval_y_m': list(interval) if interval is not None else None,
            'target_fully_inside_road': bool(polygon) and all(lo-1e-12 <= y <= hi+1e-12 for _, y in polygon),
            'target_completely_outside_road': bool(polygon) and all(y < lo for _, y in polygon)
                or bool(polygon) and all(y > hi for _, y in polygon),
            'ego_lateral_envelope_m': ego.lateral_width(ego_yaw_rad),
            'padding_each_side_m': padding, 'required_gap_m': required,
            'left_gap_m': left_gap, 'right_gap_m': right_gap,
            'left_residual_m': left, 'right_residual_m': right,
            'cross_section_possible': max(left, right) > 1e-10}


def ray_box_distance(origin_xyz_m, direction_xyz, box, range_min_m=.05, range_max_m=12.):
    """정적 oriented 3D box 광선 교차. roll은 제외; 방향 벡터로 pitch 반영."""
    if len(origin_xyz_m) != 3 or len(direction_xyz) != 3:
        raise ValueError('3D vectors required')
    ox, oy, oz = map(finite, origin_xyz_m)
    dx, dy, dz = map(finite, direction_xyz)
    norm = math.sqrt(dx*dx + dy*dy + dz*dz)
    minimum, maximum = finite(range_min_m), finite(range_max_m)
    if norm <= 0 or minimum < 0 or maximum <= minimum:
        raise ValueError('invalid ray/range')
    dx, dy, dz = dx/norm, dy/norm, dz/norm
    c, s = math.cos(box.yaw_rad), math.sin(box.yaw_rad)
    local_o = (c*(ox-box.x_m)+s*(oy-box.y_m), -s*(ox-box.x_m)+c*(oy-box.y_m), oz)
    local_d = (c*dx+s*dy, -s*dx+c*dy, dz)
    bounds = ((-box.footprint.length_m/2, box.footprint.length_m/2),
              (-box.footprint.width_m/2, box.footprint.width_m/2),
              (box.bottom_m, box.bottom_m+box.height_m))
    enter, leave = -math.inf, math.inf
    for origin, direction, (low, high) in zip(local_o, local_d, bounds):
        if abs(direction) < 1e-14:
            if origin < low or origin > high:
                return None
            continue
        a, b = (low-origin)/direction, (high-origin)/direction
        enter, leave = max(enter, min(a, b)), min(leave, max(a, b))
        if leave < enter:
            return None
    # 가장 가까운 실제 표면이 minimum보다 가까우면 뒤쪽 표면을 새 반환으로 만들지 않는다.
    hit = enter if enter >= 0 else leave
    return hit if minimum <= hit <= maximum else None


def ray_direction(azimuth_rad, pitch_rad=0.):
    """pitch 양수는 측정면을 위로 기울이는 평가용 광선 방향."""
    azimuth, pitch = finite(azimuth_rad), finite(pitch_rad)
    return (math.cos(pitch)*math.cos(azimuth), math.sin(azimuth), math.sin(pitch)*math.cos(azimuth))


def cast_rays(boxes, *, origin_xyz_m=(.06, 0., .095), samples=500,
              angle_min=-math.pi, pitch_rad=0., range_min_m=.05, range_max_m=12.,
              rear_half_angle_deg=30.):
    """평가용 nearest-hit labels와 합성 거리. 마스크는 각 센서 방위에 적용."""
    if isinstance(samples, bool) or not isinstance(samples, int) or not 2 <= samples <= 8192:
        raise ValueError('invalid samples')
    half = finite(rear_half_angle_deg)
    if not 0 <= half < 180:
        raise ValueError('invalid mask')
    distances, labels, masked = [], [], []
    for i in range(samples):
        angle = math.remainder(angle_min + i*math.tau/samples, math.tau)
        direction = ray_direction(angle, pitch_rad)
        hits = [(distance, box.label) for box in boxes
                if (distance := ray_box_distance(origin_xyz_m, direction, box, 0., range_max_m)) is not None]
        distance, label = min(hits, default=(math.inf, None))
        # 가까운 가림 물체를 제거해 뒤쪽 벽으로 광선을 투과시키지 않는다.
        distances.append(distance if distance >= range_min_m else math.nan)
        labels.append(label)
        masked.append(abs(angle) > math.radians(180-half) + 1e-12)
    return {'ranges': distances, 'truth_labels': labels, 'masked': masked,
            'truth_scope': 'evaluation_only_not_controller_input', 'synthetic_rays': True}
