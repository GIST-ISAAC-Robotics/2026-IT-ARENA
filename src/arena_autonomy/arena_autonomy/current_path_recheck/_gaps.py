"""현재 입력의 경로 재검사. 명시 선택용 순수 함수이며 상태 전이/발행은 없다.

원 함수: 2026-11-11/body_nonreturn_r136_v1/candidate.py, 2026-11-12/adjacent_mount_gap_r149_v1/candidate.py
이관/동등성 근거: current_recheck_module_r176_v1.
"""
import math
import numpy as np
from arena_autonomy.observation_support import source_valid
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn, _query_brackets
from arena_autonomy.known_structure import sector_rays
from arena_autonomy.scan_input import scan_angles


def bounded_body_nonreturn(scene,raw_scan,queries,padding,length,width,speed,target):
    result=dict(accepted=False,reason='invalid_input',unknown_filled=False,
        residual_hidden_object_risk=True,maximum_exterior_m=.01,
        maximum_gap_rad=math.radians(6),minimum_neighbor_range_m=.6,
        maximum_local_chord_m=.05,policy='guard_low_speed_body_nonreturn_r136')
    def reject(reason):return {**result,'reason':reason}
    if (raw_scan is None or not source_valid(scene)
            or any(isinstance(x,bool) or not isinstance(x,(int,float)) or not math.isfinite(x)
                   for x in (length,width,speed,target))
            or length<=0 or width<=0 or not 0<=speed<=.35 or not 0<target<=.35):
        return result
    q=np.asarray(queries,dtype=float).reshape(-1,2);pad=np.asarray(padding,dtype=float)
    if (not len(q) or not np.all(np.isfinite(q)) or pad.shape!=(len(scene.rays),)
            or not np.all(np.isfinite(pad)) or np.any(pad<0)):
        return result
    exterior=np.abs(q[:,1])-width/2
    result.update(queries=len(q),exterior_max_m=float(exterior.max()))
    if np.any(abs(q[:,0])>length/2) or np.any(exterior<=0) or np.any(exterior>.01):
        return reject('outside_small_body_side')
    checked=_bounded_nonreturn(scene,raw_scan,q,pad,result)
    if not checked['accepted']:return checked
    chords=[]
    for gap in checked['gaps']:
        rays=scene.rays[gap['first']:gap['last']+1]
        if any(abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(100) for r in rays):
            return {**checked,'accepted':False,'reason':'front_gap_outside_scope'}
        radius=max(float(np.max(np.linalg.norm(q-np.asarray(r.ray_origin_xy_m),axis=1))) for r in rays)
        chord=2*(radius+float(pad.max()))*math.sin(gap['gap_rad']/2)
        chords.append(chord)
        if chord>.05:return {**checked,'accepted':False,'reason':'wide_local_gap','maximum_chord_m':chord}
    return {**checked,'reason':'bounded_low_speed_body_nonreturn_risk','maximum_chord_m':max(chords)}


def bounded_adjacent_structure(scene,raw_scan,queries,padding,length,width,speed,target,sectors):
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
    groups={};checked=set()
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
                return reject('not_one_declared_mount_sector')
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
    return {**result,'accepted':True,'reason':'bounded_adjacent_mount_gap_risk',
            'gaps':list(groups.values()),'maximum_chord_m':max(g['local_chord_m'] for g in groups.values())}
