"""재표본화 전 측정 배열과 점별 시간. LaserScan의 균일 metadata로 위장하지 않는다."""
from dataclasses import dataclass
import math
from types import SimpleNamespace

from arena_autonomy.local_scene import MAX_RAYS, _finite, _source


@dataclass(frozen=True, slots=True)
class MeasurementScan:
    ranges_m: tuple[float, ...]
    angles_rad: tuple[float, ...]
    acquired_at_s: tuple[float, ...]
    range_min_m: float = .05
    range_max_m: float = 12.
    source_frame_id: str = 'laser_frame'
    source_epoch: int = 0
    # SDK 추정 시각은 이 값 그대로 남긴다. 센서 취득 시각으로 자동 승격하지 않는다.
    time_basis: str = 'unverified_estimate'
    # 내부 자체 반환 필터의 원천 인덱스. 원시 native wire schema에는 없다.
    self_masked_indices: tuple[int, ...] = ()

    def __post_init__(self):
        n = len(self.ranges_m)
        if not 2 <= n <= MAX_RAYS or len(self.angles_rad) != n or len(self.acquired_at_s) != n:
            raise ValueError('measurement arrays mismatch')
        _source(self.source_frame_id, self.source_epoch)
        angles = tuple(_finite(v, 'angles_rad') for v in self.angles_rad)
        stamps = tuple(_finite(v, 'acquired_at_s') for v in self.acquired_at_s)
        if stamps[0] < 0 or any(b <= a for a, b in zip(stamps, stamps[1:])):
            raise ValueError('measurement times must strictly increase')
        ranges = []
        for value in self.ranges_m:
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError('distance must be numeric')
            ranges.append(float(value))  # NaN/Inf는 삭제하지 않고 unknown으로 전달한다.
        low, high = _finite(self.range_min_m, 'range_min_m'), _finite(self.range_max_m, 'range_max_m')
        if low < 0 or high <= low:
            raise ValueError('invalid measurement range')
        if self.time_basis not in ('unverified_estimate', 'simulation_acquisition', 'validated_sensor_acquisition'):
            raise ValueError('unknown time basis')
        object.__setattr__(self, 'ranges_m', tuple(ranges))
        object.__setattr__(self, 'angles_rad', angles)
        object.__setattr__(self, 'acquired_at_s', stamps)
        indices = tuple(self.self_masked_indices)
        if (any(type(i) is not int or not 0 <= i < n for i in indices)
                or len(set(indices)) != len(indices)
                or any(not math.isnan(ranges[i]) for i in indices)):
            raise ValueError('self masked indices require unique in-range NaN rays')
        object.__setattr__(self, 'self_masked_indices', indices)

    @property
    def ranges(self):
        return self.ranges_m

    @property
    def range_min(self):
        return self.range_min_m

    @property
    def range_max(self):
        return self.range_max_m

    @property
    def header(self):
        # ROS 콜백의 세대/최초 시각 비교용. 점별 보정은 acquired_at_s 원값을 쓴다.
        ns = round(self.acquired_at_s[0]*1e9)
        return SimpleNamespace(frame_id=self.source_frame_id,
            stamp=SimpleNamespace(sec=ns//1_000_000_000, nanosec=ns%1_000_000_000))


def sampling_gaps(angles):
    """취득 순서의 signed 간격. 끝과 처음을 연결하거나 겹친 회전을 삭제하지 않는다."""
    return tuple(math.remainder(b-a, math.tau) for a, b in zip(angles, angles[1:]))


def angular_cell_widths(scene, indices):
    """각 원천 측정의 CW/CCW 반폭. 동일 방향의 이웃 중간까지이며 seam은 연장하지 않는다.

    끝점 바깥 반폭은 같은 끝점의 내부 간격 반폭을 사용한다. 이 폭은 unknown
    질의용이며 끝/처음이 인접한 반환이라는 허가는 아니다. 원천 간격이 잘못된
    장면은 source_valid에서 거부하며 여기서 보간해 복구하지 않는다.
    """
    if scene.source_angles_rad is None:
        return [(math.pi/500, math.pi/500) for _ in indices]
    gaps = sampling_gaps(scene.source_angles_rad)
    if (not gaps or not (all(0 < v <= math.radians(2) for v in gaps)
                         or all(-math.radians(2) <= v < 0 for v in gaps))):
        raise ValueError('invalid source sampling order')
    widths = []
    for index in indices:
        before = abs(gaps[max(0, index-1)])/2
        after = abs(gaps[min(index, len(gaps)-1)])/2
        widths.append((before, after) if gaps[0] > 0 else (after, before))
    return widths


def unknown_cell_directions(scene, rays):
    """원 광선과 함께 이동한 비균일 unknown 각도 셀 경계."""
    lower, upper = [], []
    for ray, (cw, ccw) in zip(rays, angular_cell_widths(scene, [r.source_index for r in rays])):
        x, y = ray.direction_xy
        c, s = math.cos(cw), math.sin(cw)
        lower.append((c*x+s*y, -s*x+c*y))
        c, s = math.cos(ccw), math.sin(ccw)
        upper.append((c*x-s*y, s*x+c*y))
    return lower, upper
