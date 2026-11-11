"""기본 비활성: 명시 장착 가림에 인접한 좁은 무반환의 국소 위험 분류."""
import importlib.util
import math
from pathlib import Path

import numpy as np
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.known_structure import parse_sectors, sector_rays
from arena_autonomy.rear_fringe_trial import _query_brackets
from arena_autonomy.scan_input import scan_angles

P=Path(__file__).resolve().parent.parent/'guard_horizon_r148_v1/candidate.py'
spec=importlib.util.spec_from_file_location('r149_previous',P)
prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)
body=prior.prior.body


def bounded_adjacent_structure(scene,raw_scan,queries,padding,length,width,speed,target,sectors):
    old=body.bounded_body_nonreturn(scene,raw_scan,queries,padding,length,width,speed,target)
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


def same_path_adjacent(owner,scene,raw_scan,region,context,speed,target,steer,previous,baseline):
    old=body.same_path_with_body(owner,scene,raw_scan,region,context,speed,target,steer,previous,baseline)
    check=old.get('body_nonreturn_recheck') or {}
    if (old.get('accepted') or check.get('reason') not in ('unbounded_nonreturn','mask_boundary')
            or not context or owner.follow_config.known_structure_policy!='structure_aware_v1'):
        return old
    q=context['queries'][~context['free'] & ~context['masked']]
    proof=bounded_adjacent_structure(scene,raw_scan,q,context['padding'],owner.follow_config.length_m,
        owner.follow_config.width_m,speed,target,parse_sectors(owner.follow_config.known_structure_sectors_json))
    return proof if proof['accepted'] else {**old,'adjacent_structure_recheck':proof}


def install():
    prior.install()
    ps.same_path_risk=same_path_adjacent


def uninstall():
    prior.uninstall()


def worker_entry(*args):
    install()
    from arena_autonomy.decision_process import _worker
    return _worker(*args)
