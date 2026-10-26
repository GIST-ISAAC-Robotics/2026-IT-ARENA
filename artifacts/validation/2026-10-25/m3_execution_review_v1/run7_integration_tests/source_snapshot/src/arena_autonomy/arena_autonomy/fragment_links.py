"""관측 조각 사이의 조건부 연관 후보. identity/속도/정지/자유 공간을 확정하지 않는다.

자차 SE(2) 이동은 모터축/자이로 이력으로만 보상한다. 중심점 최근접 선택 대신
가능한 모든 조각 쌍을 남기고 분할/합침·가림·재관측의 모호함을 보존한다.
운영 제어에는 연결하지 않는다.
"""
from dataclasses import dataclass
import math

import numpy as np

from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_scene import LocalScene
from arena_autonomy.observed_fragments import partition_observed
from arena_autonomy.scene_motion import transport_points


@dataclass(frozen=True, slots=True)
class LinkBounds:
    # 실물/공식 확정값으로 오해하지 않도록 호출자가 명시한다.
    object_diameter_m: float
    speed_bound_mps: float
    range_error_m: float
    ego_transport_error_m: float
    max_source_age_s: float = .30
    max_returned_points: int = 512
    max_fragments: int = 128

    def __post_init__(self):
        names=('object_diameter_m','speed_bound_mps','range_error_m',
               'ego_transport_error_m','max_source_age_s')
        if any(isinstance(getattr(self,k),bool) or not isinstance(getattr(self,k),(int,float))
               or not math.isfinite(getattr(self,k)) or getattr(self,k)<0 for k in names):
            raise ValueError('finite nonnegative motion/error bounds required')
        if self.object_diameter_m<=0 or not 0<self.max_source_age_s<=.30:
            raise ValueError('positive diameter and source age at most .30 s required')
        if (type(self.max_returned_points) is not int or not 1<=self.max_returned_points<=1024
                or type(self.max_fragments) is not int or not 1<=self.max_fragments<=256):
            raise ValueError('bounded association budget required')


@dataclass(frozen=True, slots=True)
class SupportLink:
    previous_fragment: int
    current_fragment: int
    previous_source_index: int
    current_source_index: int
    witness_distance_m: float
    witness_bound_m: float
    acquisition_delta_s: float


@dataclass(frozen=True, slots=True)
class LinkGraph:
    evaluated: bool
    reason: str
    links: tuple[SupportLink, ...] = ()
    unmatched_previous: tuple[int, ...] = ()
    unmatched_current: tuple[int, ...] = ()
    ambiguous_previous: tuple[int, ...] = ()
    ambiguous_current: tuple[int, ...] = ()
    previous_reference_s: float | None = None
    current_reference_s: float | None = None
    previous_source_first_s: float | None = None
    current_source_first_s: float | None = None
    identity_certified: bool = False
    stationary_certified: bool = False
    free_area_certified: bool = False
    driving_authorized: bool = False


def candidate_links(previous, current, history, bounds):
    """거리 상한 아래인 쌍은 모두 보존. 한 개만 남아도 동일 물체 증명은 아니다.

같은 유한 물체의 두 표면점 사이 거리에는 물체 지름 + 이동 상한×취득시차 +
양쪽 거리 오차 + 자차 이송 오차를 허용한다. 광선 원점/취득시각은 원본에
남긴다. 이 부등식은 호출자가 제시한 크기/운동/오차 가정에만 유효하다.
"""
    if not isinstance(previous,LocalScene) or not isinstance(current,LocalScene) or not isinstance(bounds,LinkBounds):
        raise ValueError('typed scenes and bounds required')
    meta=dict(previous_reference_s=previous.t_ref_s,current_reference_s=current.t_ref_s,
              previous_source_first_s=previous.source_first_s,current_source_first_s=current.source_first_s)
    def unavailable(reason):
        return LinkGraph(False,reason,**meta)
    if previous.source_epoch!=current.source_epoch or previous.source_frame_id!=current.source_frame_id:
        return unavailable('source_epoch_or_frame_changed')
    if current.t_ref_s<=previous.t_ref_s:
        return unavailable('reference_did_not_advance')
    if previous.source_first_s is None or current.source_first_s is None:
        return unavailable('empty_observation')
    if current.source_first_s<=previous.source_first_s:
        return unavailable('no_new_source_observation')
    if (current.t_ref_s-previous.source_first_s>=bounds.max_source_age_s
            or current.t_ref_s-current.source_first_s>=bounds.max_source_age_s):
        return unavailable('source_age_exceeded')
    old,new=partition_observed(previous),partition_observed(current)
    groups=(old.fragments,new.fragments)
    if any(len(fs)>bounds.max_fragments or sum(len(f.rays) for f in fs)>bounds.max_returned_points for fs in groups):
        return unavailable('association_capacity_exceeded')
    rays=[[ray for f in fs for ray in f.rays] for fs in groups]
    if not all(rays):
        # 전체 결측도 물체 소멸/공간 비움 증거로 사용하지 않는다.
        return unavailable('no_return_support')
    try:
        old_points=transport_points([r.hit_xy_m for r in rays[0]],history,previous.t_ref_s,current.t_ref_s)
    except MotionUnavailable as error:
        return unavailable('ego_motion_unavailable:'+str(error))
    if old_points.shape!=(len(rays[0]),2) or not np.all(np.isfinite(old_points)):
        return unavailable('invalid_ego_transport')
    new_points=np.asarray([r.hit_xy_m for r in rays[1]])
    distances=np.linalg.norm(old_points[:,None,:]-new_points[None,:,:],axis=2)
    old_times=np.asarray([r.acquired_at_s for r in rays[0]])
    new_times=np.asarray([r.acquired_at_s for r in rays[1]])
    differences=np.abs(old_times[:,None]-new_times[None,:])
    allowed=(bounds.object_diameter_m+2*bounds.range_error_m+bounds.ego_transport_error_m
             +bounds.speed_bound_mps*differences)
    offsets=[np.r_[0,np.cumsum([len(f.rays) for f in fs])] for fs in groups]
    links=[]
    for i in range(len(groups[0])):
        a,b=map(int,offsets[0][i:i+2])
        for j in range(len(groups[1])):
            c,d=map(int,offsets[1][j:j+2])
            residual=distances[a:b,c:d]-allowed[a:b,c:d]
            ix,iy=np.unravel_index(int(np.argmin(residual)),residual.shape)
            if residual[ix,iy] <= 0:
                x,y=a+ix,c+iy
                links.append(SupportLink(i,j,rays[0][x].source_index,rays[1][y].source_index,
                    float(distances[x,y]),float(allowed[x,y]),float(differences[x,y])))
    old_degrees=[sum(link.previous_fragment==i for link in links) for i in range(len(groups[0]))]
    new_degrees=[sum(link.current_fragment==j for link in links) for j in range(len(groups[1]))]
    return LinkGraph(True,'conditional_candidates',tuple(links),
        tuple(i for i,v in enumerate(old_degrees) if v==0),
        tuple(i for i,v in enumerate(new_degrees) if v==0),
        tuple(i for i,v in enumerate(old_degrees) if v>1),
        tuple(i for i,v in enumerate(new_degrees) if v>1),**meta)
