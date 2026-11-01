"""진단 전용: 비스듬한 옆 벽의 연속 광선/국소 직선 근거로 연결 간격을 확인한다."""
import math
import numpy as np


def supported_gaps(points, gaps, limits, scene, horizon, padding=.03):
    accepted=np.zeros(len(gaps),dtype=bool)
    source={tuple(r.hit_xy_m):r.source_index for r in scene.rays if r.status=='return'}
    indices=np.array([source.get(tuple(p),-10000) for p in points])
    for i in np.flatnonzero(gaps>limits):
        # 두 끝 사이에 원시 결측/다른 반환/보정 뒤 순서 역전이 있으면 잇지 않는다.
        if i<2 or i+3>=len(points):continue
        ids=indices[i-2:i+4];steps=np.diff(ids)
        if not (np.all(steps==1) or np.all(steps==-1)):continue
        if np.max(np.linalg.norm(points[i:i+2],axis=1))>horizon:continue
        window=points[i-2:i+4]
        # 양쪽 세 실제 점으로 검증한다. 작은 면 끝/차량 사이를 전방 벽으로 외삽하지 않는다.
        before,after=window[:3],window[3:]
        if min(np.linalg.norm(before[-1]-before[0]),np.linalg.norm(after[-1]-after[0]))<.10:continue
        def fit(part):
            center=part.mean(axis=0);_,_,vh=np.linalg.svd(part-center,full_matrices=False)
            tangent=vh[0]
            if np.dot(tangent,part[-1]-part[0])<0:tangent=-tangent
            return center,tangent,np.array([-tangent[1],tangent[0]])
        a,at,an=fit(before);b,bt,bn=fit(after)
        if np.dot(at,bt)<=0:continue
        if max(np.max(abs((window-a)@an)),np.max(abs((window-b)@bn)))>padding:continue
        if np.any(np.diff(window@at)<=0):continue
        # 원래 광선 방향과 원점에서 이 직선의 교점을 예측한다. 반환을 생성하지 않는다.
        valid=True
        for k in (i,i+1):
            ray=scene.rays[indices[k]];origin=np.asarray(ray.ray_origin_xy_m);direction=np.asarray(ray.direction_xy)
            denominator=float(direction@an)
            if abs(denominator)<1e-8:valid=False;break
            predicted=float((a-origin)@an)/denominator
            measured=float(np.linalg.norm(points[k]-origin))
            # 수직 잔차와 방사 오차를 구분한다. 기존 3cm 방사 오차 가정 그대로.
            if predicted<=0 or abs(predicted-measured)>padding:valid=False;break
        if valid:accepted[i]=True
    return accepted
