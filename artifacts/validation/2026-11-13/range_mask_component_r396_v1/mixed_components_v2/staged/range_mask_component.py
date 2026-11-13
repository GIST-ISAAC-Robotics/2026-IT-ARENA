"""저장 비교 후보. 원 whole-sector 본문에서 membership 실패 분기만 확장한다.

새 부분 분기에도 10 mm/6 deg/50 mm/0.15 m/s/0.6 m와 원 점유·시각 검사를 유지한다.
공통 함수 설치, 상태 전이, 발행은 없다.
"""
import math
import numpy as np
from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn
from arena_autonomy.rear_fringe_trial import _query_brackets
from arena_autonomy.known_structure import sector_rays
from arena_autonomy.scan_input import scan_angles
from profile_evidence import profile_evidence, partial_component


def bounded_adjacent_structure(scene,raw_scan,queries,padding,length,width,speed,target,sectors,*,
                               mask_profile=None,motion_epoch=None,timing_verified=None):
    old=bounded_body_nonreturn(scene,raw_scan,queries,padding,length,width,speed,target)
    if old.get('accepted') or old.get('reason') not in ('unbounded_nonreturn','mask_boundary'):
        return old
    result={**old,'policy':'guard_adjacent_structure_gap_r149','unknown_filled':False,
            'residual_hidden_object_risk':True,'maximum_speed_mps':.15}
    def reject(reason,**fields):
        return {**result,'accepted':False,'reason':reason,**fields}
    if not 0<=speed<=.15 or not 0<target<=.15 or not sectors:
        return reject('adjacent_structure_outside_scope')
    q=np.asarray(queries,dtype=float).reshape(-1,2);pad=np.asarray(padding,dtype=float)
    rays=scene.rays;count=len(rays)
    origins=np.asarray([r.ray_origin_xy_m for r in rays])
    directions=np.asarray([r.direction_xy for r in rays])
    status=np.asarray([r.status for r in rays]);values=np.asarray(raw_scan.ranges,dtype=float)
    depths=np.asarray([math.dist(r.ray_origin_xy_m,r.hit_xy_m) if r.status=='return' else -1. for r in rays])
    memberships=[set(row.tolist()) for row in sector_rays(0.,0.,count,sectors,source_angles=scan_angles(raw_scan))]
    groups={};checked=set();profile_bands=None;partial_components=[];pure_components=[]
    angle=lambda a,b:abs(math.atan2(float(directions[a,0]*directions[b,1]-directions[a,1]*directions[b,0]),
                                   float(np.dot(directions[a],directions[b]))))
    # 전단 body 검사가 출처/시간/국소 차체 범위와 알려진 점유를 검증했다.
    # 첫 무경계 구간 뒤의 모든 질의도 아래에서 끝까지 다시 검사한다.
    for radius,angles,lo,hi in _query_brackets(q,origins,directions):
        if np.any(radius<=1e-9):return reject('query_at_sensor_origin')
        if (angles[lo]<-1e-10 or angles[hi]>1e-10 or abs(lo-hi)>1
                or abs(angles[lo])+abs(angles[hi])>.025):
            return reject('unbracketed_query')
        if not any(status[i]=='unknown' for i in (lo,hi)):return reject('not_nonreturn_only')
        for i in (lo,hi):
            # 한 질의의 각도 세포가 mask/unknown 경계에 걸릴 수도 있다.
            # 양쪽 모두 masked인 질의는 위에서 거절했고, 짝 unknown에서
            # 이 마스크를 포함한 성분 전체와 바깥 반환을 검증한다.
            if status[i]=='masked':continue
            if status[i]=='return':
                if depths[i]-radius[i]-pad[i]<=0:return reject('near_return')
                continue
            if i in checked:continue
            start=end=i
            while start>0 and status[start-1] in ('unknown','masked'):start-=1
            while end+1<count and status[end+1] in ('unknown','masked'):end+=1
            a,b=start-1,end+1
            if a<0 or b>=count or status[a]!='return' or status[b]!='return':
                return reject('combined_unbounded')
            member=set(range(start,end+1))
            masked={j for j in member if status[j]=='masked'}
            # 묶음 안 마스크는 한 개의 명시된 장착 구간 전체와 일치해야 한다.
            if not masked or not any(masked==m for m in memberships):
                if mask_profile is None:
                    return reject('not_one_declared_mount_sector')
                try:
                    if profile_bands is None:
                        profile_bands=profile_evidence(scene,raw_scan,sectors,mask_profile,
                                                       motion_epoch,timing_verified)
                except (ValueError,TypeError,AttributeError,OverflowError) as error:
                    return reject('range_profile_evidence_invalid',profile_error=str(error))
                if masked:
                    partial=partial_component(masked,memberships,a,b,values,profile_bands)
                    if partial is None:
                        return reject('not_range_split_declared_component')
                    partial_components.append(partial)
                else:
                    # 실제 반환 사이의 일반 무반환 성분은 장착 마스크가 아니다.
                    # 아래 동일한 raw/점유/전방/폭 검사와 전체 공백 합산을 적용한다.
                    # 원 profile 미지정 경로는 앞에서 그대로 거절해 보존한다.
                    pure_components.append(dict(first=start,last=end))
            unknown=sorted(member-masked)
            if not np.all(np.isposinf(values[unknown])) or not np.all(np.isnan(values[list(masked)])):
                return reject('invalid_raw_gap')
            if any(abs(math.atan2(directions[j,1],directions[j,0]))<=math.radians(100) for j in member):
                return reject('front_gap_outside_scope')
            if min(depths[a],depths[b])<result['minimum_neighbor_range_m']:
                return reject('near_outer_neighbor')
            if (not np.all(np.isfinite(values[[a,b]]))
                    or max(abs(values[a]-depths[a]),abs(values[b]-depths[b]))>1e-6):
                return reject('raw_scene_mismatch')
            spans=[]
            for j in unknown:
                if j-1 in member and status[j-1]=='unknown':continue
                last=j
                while last+1 in member and status[last+1]=='unknown':last+=1
                spans.append(angle(j-1,last+1))
            gap=sum(spans)
            if gap>math.radians(6):return reject('wide_unknown_part')
            combined=angle(a,b)
            if combined>=math.pi/2:return reject('wide_combined_angle')
            reach=max(float(np.max(np.linalg.norm(q-origins[j],axis=1))) for j in member)
            chord=2*(reach+float(pad.max()))*math.sin(combined/2)
            if chord>result['maximum_local_chord_m']:return reject('wide_combined_local_gap',maximum_chord_m=chord)
            groups[(start,end)]=dict(first=start,last=end,masked_indices=sorted(masked),
                unknown_gap_rad=gap,combined_gap_rad=combined,local_chord_m=chord,
                outer_return_ranges_m=[float(depths[a]),float(depths[b])])
            checked.update(member)
    if sum(g['unknown_gap_rad'] for g in groups.values())>math.radians(6):
        return reject('many_unknown_gaps')
    if sum(g['local_chord_m'] for g in groups.values())>result['maximum_local_chord_m']:
        return reject('many_combined_local_gaps')
    accepted={**result,'accepted':True,'reason':'bounded_adjacent_mount_gap_risk',
              'gaps':list(groups.values()),'maximum_chord_m':max(g['local_chord_m'] for g in groups.values())}
    if partial_components:
        accepted.update(policy='range_limited_mount_component_r396',
                        range_mask_components=partial_components)
    if pure_components:
        accepted.update(policy='mixed_bounded_components_r396_v2',
                        ordinary_nonreturn_components=pure_components)
    return accepted
