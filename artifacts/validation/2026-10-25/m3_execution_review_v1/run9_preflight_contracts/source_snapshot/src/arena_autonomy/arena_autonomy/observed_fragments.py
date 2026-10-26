"""센서 반환의 관측 조각 계약. 차량 분류/완전 외곽/빈 공간/구동 허가 없음.

광선 취득 순서에서 결측·마스크·인덱스/시각/공간 공백을 넘지 않는다.
오래된 반환도 삭제하지 않고 freshness를 표시한다. 현재 운영 제어 미연결.
"""
from dataclasses import dataclass
import math

from arena_autonomy.local_scene import LocalScene, SensorRay


@dataclass(frozen=True, slots=True)
class FragmentConfig:
    base_gap_m: float = .10
    range_gap_scale: float = .035
    max_acquisition_gap_s: float = .0021
    max_age_s: float = .30

    def __post_init__(self):
        values = (self.base_gap_m, self.range_gap_scale, self.max_acquisition_gap_s, self.max_age_s)
        if any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) or v<=0 for v in values):
            raise ValueError('positive finite fragment bounds required')
        if self.max_age_s > .30:
            raise ValueError('fragment freshness cannot exceed existing .30 s observation limit')


@dataclass(frozen=True, slots=True)
class ObservedFragment:
    rays: tuple[SensorRay, ...]
    reference_s: float
    source_first_s: float
    source_last_s: float
    source_frame_id: str
    source_epoch: int
    observed_bounds_xy_m: tuple[float,float,float,float]
    fresh: bool
    complete_object_extent: bool = False
    vehicle_classification: str = 'unclassified'


@dataclass(frozen=True, slots=True)
class FragmentPartition:
    fragments: tuple[ObservedFragment, ...]
    unknown_indices: tuple[int, ...]
    masked_indices: tuple[int, ...]
    source_first_s: float | None
    source_last_s: float | None
    reference_s: float
    source_frame_id: str
    source_epoch: int
    driving_authorized: bool = False
    free_area_certified: bool = False


def _range(ray):
    dx,dy = (ray.hit_xy_m[i]-ray.ray_origin_xy_m[i] for i in (0,1))
    along = dx*ray.direction_xy[0]+dy*ray.direction_xy[1]
    cross = dx*ray.direction_xy[1]-dy*ray.direction_xy[0]
    if along <= 0 or abs(cross)>1e-8*max(1.,along):
        raise ValueError('incoherent returned endpoint/direction')
    return math.hypot(dx,dy)


def partition_observed(scene, config=FragmentConfig()):
    """원시 인덱스와 시각으로 분할하며 마지막/첫 광선을 이어 붙이지 않는다."""
    if not isinstance(scene,LocalScene) or not isinstance(config,FragmentConfig):
        raise ValueError('typed LocalScene and FragmentConfig required')
    groups, current, unknown, masked = [], [], [], []
    previous, previous_range = None, None
    for ray in scene.rays:
        if ray.status != 'return':
            if current:
                groups.append(tuple(current))
                current=[]
            (unknown if ray.status=='unknown' else masked).append(ray.source_index)
            previous=previous_range=None
            continue
        distance=_range(ray)
        if previous is not None:
            contiguous = (ray.source_index==previous.source_index+1
                          and ray.acquired_at_s-previous.acquired_at_s<=config.max_acquisition_gap_s
                          and math.dist(ray.hit_xy_m,previous.hit_xy_m)<=config.base_gap_m+config.range_gap_scale*min(distance,previous_range))
            if not contiguous:
                groups.append(tuple(current))
                current=[]
        current.append(ray)
        previous,previous_range=ray,distance
    if current: groups.append(tuple(current))
    fragments=[]
    for rays in groups:
        xs,ys=zip(*(ray.hit_xy_m for ray in rays))
        fragments.append(ObservedFragment(rays,scene.t_ref_s,rays[0].acquired_at_s,rays[-1].acquired_at_s,
                         scene.source_frame_id,scene.source_epoch,(min(xs),min(ys),max(xs),max(ys)),
                         0<=scene.t_ref_s-rays[0].acquired_at_s<config.max_age_s))
    # 유효점을 하나도 버리지 않는다. 작은 조각/오래된 조각을 물체 부재로 바꾸지 않는다.
    if sum(len(group.rays) for group in fragments)+len(unknown)+len(masked)!=len(scene.rays):
        raise AssertionError('partition dropped source rays')
    return FragmentPartition(tuple(fragments),tuple(unknown),tuple(masked),scene.source_first_s,
                             scene.source_last_s,scene.t_ref_s,scene.source_frame_id,scene.source_epoch)
