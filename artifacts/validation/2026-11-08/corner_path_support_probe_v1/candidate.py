"""현재 두 직선 면이 직전 코너를 지지하는지 비교한다. 미연결 분석 후보."""
import math
import numpy as np
from arena_autonomy.local_follow import path_agrees_arc
from arena_autonomy.piecewise_wall import corner_path, _vertices
from arena_autonomy.local_path import _origin_error, _path_ahead


def propose(points, reference, side, config, reference_age):
    fail=dict(accepted=False,reason='current_corner_support_absent',driving_authorized=False,
              free_space_certified=False,added_sensor_points=0)
    p=np.asarray(points,dtype=float)
    if (reference is None or side not in ('left','right') or
        not 0<=reference_age<config.memory_ttl_s or p.ndim!=2 or p.shape[1]!=2 or
        len(p)<24 or not np.all(np.isfinite(p))):
        return None,{**fail,'reason':'invalid_or_expired_reference'}
    reference=np.asarray(reference,dtype=float)
    sign=1 if side=='left' else -1
    order=np.argsort(-sign*np.arctan2(p[:,1],p[:,0]),kind='stable')
    limits=.10+.035*np.minimum(np.linalg.norm(p[order[:-1]],axis=1),np.linalg.norm(p[order[1:]],axis=1))
    groups=np.split(order,np.flatnonzero(np.linalg.norm(np.diff(p[order],axis=0),axis=1)>limits)+1)

    def fit(ids):
        pts=p[ids]
        if len(pts)<12:return None
        center=pts.mean(axis=0);t=np.linalg.svd(pts-center,full_matrices=False)[2][0]
        if t@(pts[-1]-pts[0])<0:t=-t
        n=np.array([-t[1],t[0]])
        residual=(pts-center)@n
        center=center+(residual.min()+residual.max())/2*n
        if np.max(np.abs((pts-center)@n))>.03 or np.ptp(pts@t)<.25:return None
        return center,t

    near=[];faces=[]
    for group in groups:
        pts=p[group]
        mask=(abs(pts[:,0])<=.35)&(sign*pts[:,1]>.1)&(abs(pts[:,1]-sign*config.wall_offset_m)<=.12)
        ids=group[mask];count=len(ids)
        # 기존 fragmented_wall_hypothesis의 3 cm/70% 근거리 직선 재적합과 같다.
        # 가까운 차량 끝면이 몇 점 이어져도 경로용 벽 면에서만 구별한다.
        # 전체 반환은 호출자의 원시 점유 목록에 그대로 남는다.
        for _ in range(3):
            if len(ids)<12:break
            q=p[ids];c=q.mean(axis=0);t=np.linalg.svd(q-c,full_matrices=False)[2][0]
            good=abs((q-c)@np.array([-t[1],t[0]]))<=.03
            if np.count_nonzero(good)<max(12,.7*count):break
            ids=ids[good]
        line=fit(ids) if len(ids)>=max(12,.7*count) else None
        if line is not None:
            c,t=line
            if t[0]<0:t=-t
            if t[0]>=math.cos(math.radians(30)):near.append((ids,c,t))
        if len(group)>=12:
            vertices=_vertices(pts,.025)
            for lo,hi in zip(vertices[:-1],vertices[1:]):
                ids=group[lo:hi+1];line=fit(ids)
                if line is not None and np.ptp(p[ids]@line[1])>=.35:faces.append((ids,*line))
    options=[]
    for aid,a,t0 in near:
        for bid,b,t1 in faces:
            if np.intersect1d(aid,bid).size:continue
            # 각 면의 관측 순서로 진행 방향을 유지하고, 직전 코너 형상도 별도 대조한다.
            angle=math.atan2(float(np.cross(t0,t1)),float(t0@t1))
            if not math.radians(40)<=abs(angle)<=math.radians(135):continue
            u,v=np.linalg.solve(np.column_stack((t0,-t1)),b-a)
            intersection=a+u*t0
            apos=(p[aid]-intersection)@t0;bpos=(p[bid]-intersection)@t1
            gap=-float(apos.max())
            if not .10<gap<=1.2 or bpos.min()<-.03 or bpos.min()>.5:continue
            # 가려진 접근 면 사이에 더 가까운 실제 반환이 있어야 한다.
            projected=(p-intersection)@t0;normal=np.array([-t0[1],t0[0]])
            witness=(projected>apos.max())&(projected<-.03)&(sign*((p-a)@normal)<-.05)&(sign*p[:,1]>.08)
            witness[np.r_[aid,bid]]=False  # 지지 면 자체를 가림 물체의 증거로 재사용하지 않는다.
            if np.count_nonzero(witness)<3:continue
            # 이 표본은 적합 두 직선의 경로용 표현이다. 센서/점유 목록에는 넣지 않는다.
            # 기존 지역 경계 탐색 1.8 m 안에서 표현하여 먼 벽 때문에 36점 표현이 성기지 않게 한다.
            end=min(float(bpos.max()),1.8)
            if end<float(bpos.min()):continue
            guide=np.vstack((intersection+np.linspace(float(apos.min()),0.,32)[:,None]*t0,
                             intersection+np.linspace(0.,end,32)[1:,None]*t1))
            path=corner_path(guide,side,config.wall_offset_m,config.wheelbase_m/math.tan(config.max_steering_rad))
            if (path is None or _origin_error(path)>config.boundary_anchor_error_m or
                _path_ahead(path)<config.boundary_min_ahead_m or
                not path_agrees_arc(path,reference,config.boundary_agreement_m,hi=1.5)):continue
            evidence={**fail,'accepted':True,'reason':'current_two_faces_support_previous_corner',
                'near_source_indices':aid.tolist(),'far_source_indices':bid.tolist(),
                'occluder_indices':np.flatnonzero(witness).tolist(),
                'corner_xy_m':intersection.tolist(),'unobserved_approach_gap_m':gap,
                'far_face_start_from_corner_m':float(bpos.min()),'turn_rad':angle,
                'path_anchor_error_m':_origin_error(path),'path_ahead_m':_path_ahead(path),
                'observed_face_span_sum_m':float(np.ptp(p[aid]@t0)+np.ptp(p[bid]@t1)),
                'geometry_intersection_is_unobserved':True,'reference_age_s':reference_age}
            options.append((path,evidence))
    if len(options)!=1:return None,{**fail,'reason':'ambiguous_or_absent_face_pair','pairs':len(options)}
    return options[0]
