"""동일 crop 성분의 현재 이동영역 위험 검사. free/연속충돌/실물 증명이 아니다."""
import math
import numpy as np

from arena_autonomy.observation_support import source_valid
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn
from arena_autonomy.crop_edge_selection import SIMULATION_SCOPE, ESTIMATED_SCOPE, basis_matches, timing_matches


def component_metric(rays, indices, reach, padding):
    """방향 이등분선에 수직인 표본 ray segment 횡폭 상계. padding은 한 번만 포함."""
    origins=np.asarray([rays[i].ray_origin_xy_m for i in indices],dtype=float)
    directions=np.asarray([rays[i].direction_xy for i in indices],dtype=float)
    # 정상 motion_scene/SensorRay의 유한 원점·unit direction 계약을 합성 직접 호출에도 유지한다.
    if (not len(origins) or not np.all(np.isfinite(origins)) or not np.all(np.isfinite(directions))
            or np.any(np.abs(np.linalg.norm(directions,axis=1)-1)>1e-10)
            or not math.isfinite(reach) or not math.isfinite(padding) or min(reach,padding)<0):
        raise ValueError('finite origins, unit directions and nonnegative reach/padding required')
    angles=np.arctan2(directions[:,1],directions[:,0]);unwrapped=np.unwrap(angles)
    span=float(np.max(unwrapped)-np.min(unwrapped))
    diameter=float(np.max(np.linalg.norm(origins[:,None,:]-origins[None,:,:],axis=2)))
    return dict(direction_span_rad=span,origin_diameter_m=diameter,
        transverse_width_bound_m=diameter+2*(reach+padding)*math.sin(span/2) if span<=math.pi else math.inf,
        central=bool(np.any(np.abs(angles)<=math.radians(30))),current_directions_rad=angles.tolist())


def check(scene,raw_scan,queries,padding,component,*,all_queries,motion_epoch,
          observation_scope=SIMULATION_SCOPE,timing_verified=True,timing_policy='verified_only'):
    """queries는 현재 motion_region unknown만, reach는 그 전체 질의를 사용한다."""
    result=dict(accepted=False,reason='invalid_crop_current_input',policy='crop_current_reach_r426',
        unknown_filled=False,residual_hidden_object_risk=True,continuous_sweep_certified=False,
        maximum_gap_rad=math.radians(6),maximum_local_chord_m=.05,
        allowed_region='same_call_current_motion_region_not_body_side_box',
        width_contract='sampled_ray_segment_transverse_bound_not_hidden_object_size')
    def reject(reason,**extra):return {**result,'accepted':False,'reason':reason,**extra}
    if (not source_valid(scene) or not basis_matches(observation_scope,timing_policy,raw_scan.time_basis)
            or (observation_scope==ESTIMATED_SCOPE and not timing_matches(
                observation_scope,timing_verified,timing_policy,raw_scan.time_basis))
            or type(motion_epoch) is not int or motion_epoch<0 or type(component) is not dict):
        return result
    rays=scene.rays;count=len(rays);members=component.get('source_indices');flanks=component.get('flank_source_indices')
    if (any(type(ids) is not list or not ids or any(type(i) is not int or not 0<=i<count for i in ids)
            or len(set(ids))!=len(ids) for ids in (members,flanks)) or len(flanks)!=2):
        return reject('crop_component_invalid')
    ordered=sorted(members);a,b=ordered[0]-1,ordered[-1]+1
    if ordered!=list(range(ordered[0],ordered[-1]+1)) or set(flanks)!={a,b}:
        return reject('crop_component_not_whole_contiguous')
    total=raw_scan.source_epoch+motion_epoch;epoch=total*(total+1)//2+motion_epoch
    if (scene.source_epoch!=epoch or scene.source_frame_id!=raw_scan.source_frame_id
            or scene.source_first_s!=raw_scan.acquired_at_s[0] or scene.source_last_s!=raw_scan.acquired_at_s[-1]
            or scene.source_angles_rad is None or tuple(scene.source_angles_rad)!=raw_scan.angles_rad
            or len(raw_scan.ranges_m)!=count
            or any(r.source_index!=i or r.source_frame_id!=raw_scan.source_frame_id or r.source_epoch!=epoch
                   or r.acquired_at_s!=raw_scan.acquired_at_s[i] for i,r in enumerate(rays))):
        return reject('current_raw_source_metadata_mismatch')
    q=np.asarray(queries,dtype=float).reshape(-1,2);whole=np.asarray(all_queries,dtype=float).reshape(-1,2)
    pad=np.asarray(padding,dtype=float);origins=np.asarray([r.ray_origin_xy_m for r in rays],dtype=float)
    directions=np.asarray([r.direction_xy for r in rays],dtype=float)
    if (not len(q) or not len(whole) or not np.all(np.isfinite(q)) or not np.all(np.isfinite(whole))
            or not np.all(np.isfinite(origins)) or not np.all(np.isfinite(directions))
            or np.any(np.abs(np.linalg.norm(directions,axis=1)-1)>1e-10) or pad.shape!=(count,)
            or not np.all(np.isfinite(pad)) or np.any(pad<0)):
        return result
    # 전체 질의/원점 쌍을 사용하되 큰 임시 NxMx2 배열은 만들지 않는다.
    reach=max(float(np.max(np.linalg.norm(whole[start:start+128,None,:]-origins[None,:,:],axis=2)))
              for start in range(0,len(whole),128))
    maximum_padding=float(pad.max());minimum=reach+maximum_padding
    if not math.isfinite(minimum) or minimum<=0:return result
    result.update(queries=len(q),all_motion_queries=len(whole),query_reach_m=reach,
        maximum_padding_m=maximum_padding,minimum_neighbor_range_m=minimum,
        expected_source_indices=ordered,expected_flank_source_indices=[a,b])
    checked=_bounded_nonreturn(scene,raw_scan,q,pad,result)
    if not checked['accepted']:return checked
    metrics=[]
    for gap in checked['gaps']:
        full=list(range(gap['first'],gap['last']+1))
        if full!=ordered or [gap['first']-1,gap['last']+1]!=[a,b]:
            return reject('different_current_whole_component',gaps=checked['gaps'])
        try:metric=component_metric(rays,[a,*full,b],reach,maximum_padding)
        except ValueError:return reject('current_component_geometry_invalid')
        metrics.append(dict(source_indices=full,flank_source_indices=[a,b],**metric))
        if metric['central']:return reject('current_component_or_flank_central',components=metrics,gaps=checked['gaps'])
        if metric['direction_span_rad']>math.radians(6):
            return reject('wide_current_direction_span',components=metrics,gaps=checked['gaps'])
        if metric['transverse_width_bound_m']>.05:return reject('wide_current_transverse_bound',components=metrics,gaps=checked['gaps'])
    span_sum=sum(m['direction_span_rad'] for m in metrics);width_sum=sum(m['transverse_width_bound_m'] for m in metrics)
    if not metrics or span_sum>math.radians(6) or width_sum>.05:
        return reject('combined_current_component_limit',components=metrics)
    return {**checked,'accepted':True,'reason':'bounded_same_crop_current_reach_risk',
            'components':metrics,'total_direction_span_rad':span_sum,'total_transverse_width_bound_m':width_sum}
