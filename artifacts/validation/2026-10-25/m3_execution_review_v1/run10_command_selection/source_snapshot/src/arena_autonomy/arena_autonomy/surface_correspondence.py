"""짧아진 현재 표면의 정적 대응 가능성. 동일 물체/벽/정지 인증은 아님.

각 현재 점이 이전 한 조각의 실제 점 근처에 있는지를 모두 검사한다.
이전 조각 사이 보간·외삽·최근접 단일 정체성 선택·출처 시각 갱신을 하지 않는다.
가림으로 이전 표면의 일부만 보이는 경우를 허용하므로 이 검사는 비대칭이다.
운영 제어/경계 기억 갱신과 미연결이며, 불일치도 물체 소멸 증거가 아니다.
"""
from dataclasses import dataclass
import math

import numpy as np

from arena_autonomy.local_scene import LocalScene
from arena_autonomy.observed_fragments import partition_observed
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable


@dataclass(frozen=True, slots=True)
class SurfaceBounds:
    range_error_m: float
    ego_transport_error_m: float
    max_source_age_s: float = .30
    min_points: int = 5
    max_points: int = 512
    max_fragments: int = 128

    def __post_init__(self):
        for name in ('range_error_m','ego_transport_error_m','max_source_age_s'):
            v=getattr(self,name)
            if type(v) not in (int,float) or not math.isfinite(v) or v<0:
                raise ValueError('finite nonnegative surface assumptions required')
        if not 0<self.max_source_age_s<=.30:
            raise ValueError('surface source age cannot exceed .30 s')
        if (type(self.min_points) is not int or not 2<=self.min_points<=512 or
                type(self.max_points) is not int or not self.min_points<=self.max_points<=1024 or
                type(self.max_fragments) is not int or not 1<=self.max_fragments<=256):
            raise ValueError('bounded surface comparison capacity required')


def stationary_surface_candidates(previous, current, history, bounds):
    """고정 표면과 양립 가능한 대응만 기록. 움직인 표면도 양립할 수 있다.

    두 관측의 표본점이 실제로 동일한 표면점이라는 가정은 하지 않는다.
    허용차는 두 거리 오차+자차 이송 오차다. 점 간 이산 간격을 임의로 더하지
    않으므로 실제 같은 표면도 표본이 성기면 거절될 수 있다. 어느 경우에도
    표면 정체성·정지·빈 공간·0.3초 이후의 기억 갱신을 승인하지 않는다.
    """
    if (not isinstance(previous,LocalScene) or not isinstance(current,LocalScene)
            or not isinstance(bounds,SurfaceBounds)):
        raise ValueError('typed scenes and surface bounds required')
    result=dict(evaluated=False,reason='invalid_pair',links=[],ambiguous_previous=[],
                ambiguous_current=[],unmatched_current=[],current_supported_source_indices=[],
                previous_reference_s=previous.t_ref_s,current_reference_s=current.t_ref_s,
                previous_source_first_s=previous.source_first_s,
                current_source_first_s=current.source_first_s,
                identity_certified=False,stationary_certified=False,wall_certified=False,
                free_area_certified=False,driving_authorized=False,memory_refresh_authorized=False,
                occupancy_points_removed=0)
    def unavailable(reason):return {**result,'reason':reason}
    if previous.source_epoch!=current.source_epoch or previous.source_frame_id!=current.source_frame_id:
        return unavailable('source_epoch_or_frame_changed')
    if current.t_ref_s<=previous.t_ref_s:
        return unavailable('reference_did_not_advance')
    if previous.source_first_s is None or current.source_first_s is None:
        return unavailable('empty_observation')
    if current.source_first_s<=previous.source_first_s:
        return unavailable('no_new_source_observation')
    if (current.t_ref_s-previous.source_first_s>=bounds.max_source_age_s or
            current.t_ref_s-current.source_first_s>=bounds.max_source_age_s):
        return unavailable('source_age_exceeded')
    partitions=[partition_observed(s) for s in (previous,current)]
    groups=[partition.fragments for partition in partitions]
    if any(len(fs)>bounds.max_fragments or sum(len(f.rays) for f in fs)>bounds.max_points for fs in groups):
        return unavailable('surface_capacity_exceeded')
    rays=[[ray for f in fs for ray in f.rays] for fs in groups]
    if not all(rays):return unavailable('no_return_support')
    try:
        transported=transport_points([r.hit_xy_m for r in rays[0]],history,
                                     previous.t_ref_s,current.t_ref_s)
    except MotionUnavailable as error:
        return unavailable('ego_motion_unavailable:'+str(error))
    if transported.shape!=(len(rays[0]),2) or not np.all(np.isfinite(transported)):
        return unavailable('invalid_ego_transport')
    new=np.asarray([r.hit_xy_m for r in rays[1]])
    distance=np.linalg.norm(transported[:,None]-new[None],axis=2)
    offsets=[np.r_[0,np.cumsum([len(f.rays) for f in fs])] for fs in groups]
    tolerance=2*bounds.range_error_m+bounds.ego_transport_error_m
    links=[]
    supported=set()
    for i,old_fragment in enumerate(groups[0]):
        if len(old_fragment.rays)<bounds.min_points:continue
        a,b=map(int,offsets[0][i:i+2])
        for j,new_fragment in enumerate(groups[1]):
            if len(new_fragment.rays)<bounds.min_points:continue
            c,d=map(int,offsets[1][j:j+2])
            nearest=np.min(distance[a:b,c:d],axis=0)
            maximum=float(np.max(nearest))
            if maximum>tolerance+1e-12:continue
            indices=[r.source_index for r in new_fragment.rays]
            supported.update(indices)
            # 대응 표면의 일부가 안 보여도 과거 점의 존재/시각을 갱신하지 않는다.
            links.append(dict(previous_fragment=i,current_fragment=j,
                previous_points=b-a,current_points=d-c,
                current_source_indices=indices,
                maximum_nearest_distance_m=maximum,
                rms_nearest_distance_m=float(np.sqrt(np.mean(nearest**2))),
                tolerance_m=tolerance,
                previous_source_first_s=old_fragment.source_first_s,
                current_source_first_s=new_fragment.source_first_s))
    old_degree=[sum(l['previous_fragment']==i for l in links) for i in range(len(groups[0]))]
    new_degree=[sum(l['current_fragment']==i for l in links) for i in range(len(groups[1]))]
    return {**result,'evaluated':True,'reason':'conditional_static_surface_candidates',
            'links':links,'ambiguous_previous':[i for i,n in enumerate(old_degree) if n>1],
            'ambiguous_current':[i for i,n in enumerate(new_degree) if n>1],
            'unmatched_current':[i for i,n in enumerate(new_degree) if n==0],
            'current_supported_source_indices':sorted(supported),
            'previous_fragments':len(groups[0]),'current_fragments':len(groups[1]),
            'previous_return_count':len(rays[0]),'current_return_count':len(rays[1]),
            'current_unknown_count':len(partitions[1].unknown_indices),
            'current_masked_count':len(partitions[1].masked_indices)}
