"""ROS 독립 지역 관측 계약. 지도·시뮬 정답·차량 ID는 입력이 아니다.

광선 원점과 유효 끝점만 base_link(t_ref)로 변환한다. 미반사/마스크는
unknown이며 자유 공간 폴리곤을 만들지 않는다. 좌표 투영으로 취득 시각을
갱신하지 않는다. 상대 물체의 자체 운동 보정은 이 계약에 포함하지 않는다.
"""
from dataclasses import dataclass
import math
from typing import Literal


MAX_RAYS = 8192  # 입력 메모리 상한이며 관측 해상도/안전 임계값이 아니다.
RayStatus = Literal['return', 'unknown', 'masked']


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{name} must be finite numeric')
    return float(value)


def _xy(value, name):
    if len(value) != 2:
        raise ValueError(f'{name} must have two components')
    return tuple(_finite(v, name) for v in value)


def _source(frame, epoch):
    if not isinstance(frame, str) or not frame.strip():
        raise ValueError('source_frame_id must be nonempty')
    if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0:
        raise ValueError('source_epoch must be a nonnegative integer')


@dataclass(frozen=True, slots=True)
class SensorRay:
    ray_origin_xy_m: tuple[float, float]
    hit_xy_m: tuple[float, float] | None
    acquired_at_s: float
    status: RayStatus
    source_frame_id: str
    source_epoch: int
    source_index: int
    direction_xy: tuple[float, float]

    def __post_init__(self):
        object.__setattr__(self, 'ray_origin_xy_m', _xy(self.ray_origin_xy_m, 'origin'))
        direction = _xy(self.direction_xy, 'direction_xy')
        if abs(math.hypot(*direction) - 1.) > 1e-10:
            raise ValueError('direction_xy must be unit length')
        object.__setattr__(self, 'direction_xy', direction)
        if self.hit_xy_m is not None:
            object.__setattr__(self, 'hit_xy_m', _xy(self.hit_xy_m, 'hit'))
        stamp = _finite(self.acquired_at_s, 'acquired_at_s')
        if stamp < 0:
            raise ValueError('negative acquired_at_s')
        _source(self.source_frame_id, self.source_epoch)
        if isinstance(self.source_index, bool) or not isinstance(self.source_index, int) or self.source_index < 0:
            raise ValueError('invalid source_index')
        if self.status not in ('return', 'unknown', 'masked'):
            raise ValueError('invalid ray status')
        if (self.status == 'return') != (self.hit_xy_m is not None):
            raise ValueError('only return may have an occupied endpoint')


@dataclass(frozen=True, slots=True)
class LocalScene:
    t_ref_s: float
    source_first_s: float | None
    source_last_s: float | None
    source_frame_id: str
    source_epoch: int
    rays: tuple[SensorRay, ...]
    frame_id: str = 'base_link'

    def __post_init__(self):
        t_ref = _finite(self.t_ref_s, 't_ref_s')
        if t_ref < 0 or self.frame_id != 'base_link':
            raise ValueError('invalid reference time/frame')
        _source(self.source_frame_id, self.source_epoch)
        if len(self.rays) > MAX_RAYS:
            raise ValueError('too many rays')
        rays = tuple(self.rays)
        object.__setattr__(self, 'rays', rays)
        if not rays:
            if self.source_first_s is not None or self.source_last_s is not None:
                raise ValueError('empty input has no source observation times')
            return
        if any(not isinstance(ray, SensorRay) for ray in rays):
            raise ValueError('rays must be SensorRay')
        first = _finite(self.source_first_s, 'source_first_s')
        last = _finite(self.source_last_s, 'source_last_s')
        if first != rays[0].acquired_at_s or last != rays[-1].acquired_at_s or last > t_ref or first < 0:
            raise ValueError('source interval/reference time mismatch')
        previous = -1.
        indices = set()
        for ray in rays:
            if ray.source_frame_id != self.source_frame_id or ray.source_epoch != self.source_epoch:
                raise ValueError('mixed source frame/epoch')
            if ray.acquired_at_s < previous or ray.source_index in indices:
                raise ValueError('source time reversed or duplicate index')
            previous = ray.acquired_at_s
            indices.add(ray.source_index)

    @property
    def occupied_endpoints_xy_m(self):
        """끝점만 점유 증거. 광선 사이/가림 뒤/마스크 영역을 채우지 않는다."""
        return tuple(ray.hit_xy_m for ray in self.rays if ray.status == 'return')


def rear_masked(angle_rad, rear_half_angle_deg=30.):
    angle = math.remainder(_finite(angle_rad, 'angle_rad'), math.tau)
    half = _finite(rear_half_angle_deg, 'rear_half_angle_deg')
    if not 0 <= half < 180:
        raise ValueError('invalid rear mask angle')
    # 센서 축 기준. ±150° 경계의 표본은 현행 mask_scan과 같이 관측한다.
    return abs(angle) > math.radians(180 - half) + 1e-12


def scene_from_scan(ranges, *, angle_min, angle_increment, range_min, range_max,
                    acquired_at_s, t_ref_s, source_frame_id='laser_frame', source_epoch=0,
                    sensor_xy_m=(.06, 0.), sensor_yaw_rad=0.,
                    acquisition_to_ref_se2=None, rear_half_angle_deg=30.):
    """각 취득 base_link→기준 base_link SE(2)를 받는 순수 함수.

    acquisition_to_ref_se2[i]=(dx,dy,dyaw)는 엔코더/자이로 이력으로 계산할
    수 있다. None은 고정 센서/자차라는 호출자의 명시적 가정이다. 이 함수는
    Gazebo 위치·운동 이력·ROS를 읽지 않는다. 타임스탬프는 각 원시 출처 시각이다.
    """
    if len(ranges) > MAX_RAYS or len(acquired_at_s) != len(ranges):
        raise ValueError('ray count/timestamps mismatch')
    values = tuple(ranges)
    stamps = tuple(_finite(v, 'acquired_at_s') for v in acquired_at_s)
    minimum, maximum = _finite(range_min, 'range_min'), _finite(range_max, 'range_max')
    start, increment = _finite(angle_min, 'angle_min'), _finite(angle_increment, 'angle_increment')
    yaw = _finite(sensor_yaw_rad, 'sensor_yaw_rad')
    sx, sy = _xy(sensor_xy_m, 'sensor_xy_m')
    if minimum < 0 or maximum <= minimum or increment <= 0:
        raise ValueError('invalid scan metadata')
    poses = ((0., 0., 0.),) * len(values) if acquisition_to_ref_se2 is None else tuple(acquisition_to_ref_se2)
    if len(poses) != len(values):
        raise ValueError('transform count mismatch')
    rays = []
    for index, (distance, stamp, pose) in enumerate(zip(values, stamps, poses)):
        if len(pose) != 3:
            raise ValueError('SE(2) transform must have three components')
        dx, dy, turn = (_finite(v, 'transform') for v in pose)
        c, s = math.cos(turn), math.sin(turn)
        origin = (dx + c * sx - s * sy, dy + s * sx + c * sy)
        angle = math.remainder(start + index * increment, math.tau)
        masked = rear_masked(angle, rear_half_angle_deg)
        valid = (not isinstance(distance, bool) and isinstance(distance, (int, float))
                 and math.isfinite(distance) and minimum <= distance <= maximum)
        status = 'masked' if masked else ('return' if valid else 'unknown')
        bearing = angle + yaw + turn
        direction = (math.cos(bearing), math.sin(bearing))
        hit = None
        if status == 'return':
            hit = (origin[0] + distance * direction[0], origin[1] + distance * direction[1])
        rays.append(SensorRay(origin, hit, stamp, status, source_frame_id, source_epoch, index, direction))
    return LocalScene(t_ref_s, stamps[0] if stamps else None, stamps[-1] if stamps else None,
                      source_frame_id, source_epoch, tuple(rays))
