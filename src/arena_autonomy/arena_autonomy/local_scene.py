"""ROS 독립 지역 관측 계약. 지도·시뮬 정답·차량 ID는 입력이 아니다.

광선 원점과 유효 끝점만 base_link(t_ref)로 변환한다. 미반사/마스크는
unknown이며 자유 공간 폴리곤을 만들지 않는다. 좌표 투영으로 취득 시각을
갱신하지 않는다. 상대 물체의 자체 운동 보정은 이 계약에 포함하지 않는다.
"""
from dataclasses import dataclass, replace
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
    # 명시 원천 입력만 설정한다. 재투영 방향과 달리 취득 순서/간격을 보존한다.
    source_angles_rad: tuple[float, ...] | None = None

    def __post_init__(self):
        t_ref = _finite(self.t_ref_s, 't_ref_s')
        if t_ref < 0 or self.frame_id != 'base_link':
            raise ValueError('invalid reference time/frame')
        _source(self.source_frame_id, self.source_epoch)
        if len(self.rays) > MAX_RAYS:
            raise ValueError('too many rays')
        rays = tuple(self.rays)
        object.__setattr__(self, 'rays', rays)
        if self.source_angles_rad is not None:
            angles = tuple(_finite(v, 'source_angles_rad') for v in self.source_angles_rad)
            if len(angles) != len(rays):
                raise ValueError('source angles/rays mismatch')
            object.__setattr__(self, 'source_angles_rad', angles)
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


def blind_spec(value):
    """후방 반각(실수) 또는 시험 전용 {'rear_half_angle_deg', 'sectors_deg': [[중심, 반폭], ...]}.

    sectors_deg는 장착 구조처럼 위치를 아는 차체 고정 가림이다. 선택형 range_limits_m이
    있으면 자기 구조 거리의 반환만 가리고 다른 유효 반환은 보존한다. 무반환은 자유 공간으로
    채우지 않는다. 거리 범위가 없는 기존 각도/실수 입력은 기존과 같다.
    """
    if not isinstance(value, dict):
        return _finite(value, 'rear_half_angle_deg'), ()
    if set(value) - {'rear_half_angle_deg', 'sectors_deg', 'range_limits_m'}:
        raise ValueError('unknown blind spec key')
    rear = _finite(value.get('rear_half_angle_deg', 0.), 'rear_half_angle_deg')
    sectors = []
    for sector in value.get('sectors_deg', ()):
        if len(sector) != 2:
            raise ValueError('blind sector needs center and half width')
        center, half = (_finite(v, 'sectors_deg') for v in sector)
        if not 0 < half < 90:
            raise ValueError('invalid blind sector width')
        sectors.append((center, half))
    if 'range_limits_m' in value:
        range_bands(value,len(sectors))
    return rear, tuple(sectors)


def range_bands(value,count):
    """선택형 자기 반환 거리 범위. 없는 기존 프로필은 각도 전체 마스크를 유지한다."""
    if not isinstance(value,dict) or 'range_limits_m' not in value:return None
    rows=value['range_limits_m']
    if not isinstance(rows,(tuple,list)) or len(rows)!=count:raise ValueError('range bands/sector count mismatch')
    result=[]
    for row in rows:
        if len(row)!=2:raise ValueError('range band needs minimum and maximum')
        lo,hi=(_finite(v,'range_limits_m') for v in row)
        if not 0<=lo<hi:raise ValueError('invalid range band')
        result.append((lo,hi))
    return tuple(result)


def blind_parameter(rear_half_angle_deg, sectors_json=''):
    """노드 파라미터 → 마스크 값. 빈 문자열이면 기존 실수 그대로 돌려준다."""
    if not sectors_json:
        return rear_half_angle_deg
    import json
    decoded=json.loads(sectors_json)
    if isinstance(decoded,dict):
        if set(decoded)!={'sectors_deg','range_limits_m'}:raise ValueError('invalid ranged structure contract')
        value={'rear_half_angle_deg':rear_half_angle_deg,**decoded}
    else:
        value = {'rear_half_angle_deg': rear_half_angle_deg, 'sectors_deg': decoded}
    blind_spec(value)
    return value


def _blind_limits(rear_half_angle_deg):
    """blind_spec 검증과 각도 한계(rad). rear_masked와 같은 식·같은 예외 순서다."""
    half, sectors = blind_spec(rear_half_angle_deg)
    if not 0 <= half < 180:
        raise ValueError('invalid rear mask angle')
    limits=(math.radians(180 - half) + 1e-12,
            tuple((math.radians(center), math.radians(width) + 1e-12) for center, width in sectors))
    bands=range_bands(rear_half_angle_deg,len(sectors))
    return limits if bands is None else (*limits,bands)


def _masked_by(angle, limits, distance=None):
    rear, sectors = limits[:2]
    # 센서 축 기준. ±150° 경계의 표본은 현행 mask_scan과 같이 관측한다.
    if abs(angle) > rear:
        return True
    if len(limits)==2 or distance is None:
        return any(abs(math.remainder(angle - center, math.tau)) <= width for center, width in sectors)
    finite=isinstance(distance,(float,int)) and not isinstance(distance,bool) and math.isfinite(distance)
    return any(abs(math.remainder(angle-center,math.tau))<=width and
               (not finite or lo<=distance<=hi)
               for (center,width),(lo,hi) in zip(sectors,limits[2]))


def _reusable_blind_spec(value):
    """한 번 파싱한 값을 광선마다 재사용해도 매번 파싱한 결과와 같은 입력인지. 그 밖은 기존처럼 매번 파싱한다."""
    return not isinstance(value, dict) or isinstance(value.get('sectors_deg', ()), (list, tuple))


def rear_masked(angle_rad, rear_half_angle_deg=30., distance=None):
    angle = math.remainder(_finite(angle_rad, 'angle_rad'), math.tau)
    return _masked_by(angle, _blind_limits(rear_half_angle_deg),distance)


def scene_from_scan(ranges, *, angle_min, angle_increment, range_min, range_max,
                    acquired_at_s, t_ref_s, source_frame_id='laser_frame', source_epoch=0,
                    sensor_xy_m=(.06, 0.), sensor_yaw_rad=0.,
                    acquisition_to_ref_se2=None, rear_half_angle_deg=30., self_masked_indices=()):
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
    return _scene_from_angles(values, stamps, (start + i * increment for i in range(len(values))),
        minimum, maximum, t_ref_s, source_frame_id, source_epoch, sx, sy, yaw,
        acquisition_to_ref_se2, rear_half_angle_deg, self_masked_indices)


def scene_from_measurements(ranges, *, angles_rad, range_min, range_max,
                            acquired_at_s, t_ref_s, source_frame_id='laser_frame', source_epoch=0,
                            sensor_xy_m=(.06, 0.), sensor_yaw_rad=0.,
                            acquisition_to_ref_se2=None, rear_half_angle_deg=30.,
                            preserve_sampling=False, self_masked_indices=()):
    """가변 개수·비균일 각도의 원천 측정을 취득 순서 그대로 지역 장면으로 투영한다.

    angles_rad는 센서 좌표의 +x에서 반시계 방향인 rad 값이다. 배열의 각도가
    증가할 필요는 없으며 wrap·같은 각도의 별도 측정도 정렬/복제/삭제하지 않는다.
    각도·거리·시각·SE(2)는 같은 원천 인덱스로 대응해야 한다. C1 SDK 좌표의
    변환 및 품질 판정은 호출자의 책임이며, ROS 보간 칸의 복제를 복구하지 않는다.

    시각은 호출자가 제공하며 추정 시각을 검증된 시각으로 승격하지 않는다.
    None 변환은 고정 자차 가정이다. preserve_sampling은 원천 각도를 별도로
    보존하며 시간 정확성/전체 회전/주행 가능을 인증하지 않는다. 기본값은 이전
    생성부와 동일하며 기존 ROS 경로는 변경하지 않는다.
    """
    if (len(ranges) > MAX_RAYS or len(acquired_at_s) != len(ranges)
            or len(angles_rad) != len(ranges)):
        raise ValueError('measurement count/angles/timestamps mismatch')
    values = tuple(ranges)
    stamps = tuple(_finite(v, 'acquired_at_s') for v in acquired_at_s)
    angles = tuple(_finite(v, 'angles_rad') for v in angles_rad)
    minimum, maximum = _finite(range_min, 'range_min'), _finite(range_max, 'range_max')
    yaw = _finite(sensor_yaw_rad, 'sensor_yaw_rad')
    sx, sy = _xy(sensor_xy_m, 'sensor_xy_m')
    if minimum < 0 or maximum <= minimum:
        raise ValueError('invalid scan metadata')
    if type(preserve_sampling) is not bool:
        raise ValueError('preserve_sampling must be bool')
    scene = _scene_from_angles(values, stamps, angles, minimum, maximum, t_ref_s,
        source_frame_id, source_epoch, sx, sy, yaw, acquisition_to_ref_se2, rear_half_angle_deg,
        self_masked_indices)
    return replace(scene, source_angles_rad=angles) if preserve_sampling else scene


def _scene_from_angles(values, stamps, angles, minimum, maximum, t_ref_s,
                       source_frame_id, source_epoch, sx, sy, yaw,
                       acquisition_to_ref_se2, rear_half_angle_deg, self_masked_indices=()):
    """균일 스캔과 명시 각도 입력의 같은 변환·마스크·출처 검사."""
    poses = ((0., 0., 0.),) * len(values) if acquisition_to_ref_se2 is None else tuple(acquisition_to_ref_se2)
    if len(poses) != len(values):
        raise ValueError('transform count mismatch')
    rays = []
    self_masked = tuple(self_masked_indices)
    if (any(type(i) is not int or not 0 <= i < len(values) for i in self_masked)
            or len(set(self_masked)) != len(self_masked)
            or any(not math.isnan(values[i]) for i in self_masked)):
        raise ValueError('self masked indices require unique in-range NaN rays')
    self_masked = frozenset(self_masked)
    # 마스크 spec은 첫 광선에서 한 번만 검증·파싱한다(광선 0개면 검증하지 않는 기존 동작, 같은 예외 순서 유지).
    limits = None
    reuse = _reusable_blind_spec(rear_half_angle_deg)
    for index, (distance, stamp, pose, source_angle) in enumerate(zip(values, stamps, poses, angles)):
        if len(pose) != 3:
            raise ValueError('SE(2) transform must have three components')
        dx, dy, turn = (_finite(v, 'transform') for v in pose)
        c, s = math.cos(turn), math.sin(turn)
        origin = (dx + c * sx - s * sy, dy + s * sx + c * sy)
        angle = math.remainder(source_angle, math.tau)
        if reuse:
            masked_angle = math.remainder(_finite(angle, 'angle_rad'), math.tau)
            if limits is None:
                limits = _blind_limits(rear_half_angle_deg)
            masked = _masked_by(masked_angle, limits,distance)
        else:
            masked = rear_masked(angle, rear_half_angle_deg,distance)
        valid = (not isinstance(distance, bool) and isinstance(distance, (int, float))
                 and math.isfinite(distance) and minimum <= distance <= maximum)
        status = 'masked' if masked or index in self_masked else ('return' if valid else 'unknown')
        bearing = angle + yaw + turn
        direction = (math.cos(bearing), math.sin(bearing))
        hit = None
        if status == 'return':
            hit = (origin[0] + distance * direction[0], origin[1] + distance * direction[1])
        rays.append(SensorRay(origin, hit, stamp, status, source_frame_id, source_epoch, index, direction))
    return LocalScene(t_ref_s, stamps[0] if stamps else None, stamps[-1] if stamps else None,
                      source_frame_id, source_epoch, tuple(rays))
