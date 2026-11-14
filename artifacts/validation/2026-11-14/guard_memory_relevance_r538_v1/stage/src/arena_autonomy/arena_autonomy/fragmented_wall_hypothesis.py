"""차량에 가린 벽의 실제 직선 조각으로 만드는 경로 가설.

근거리 직선을 재적합하고 전방의 같은 선/코너 관측과 가까운 가림 반환을
대조한다. 투영은 경로 기하에만 사용하며 입력 scene/점유점은 변경하지 않는다.
가림 사이의 자유 공간이나 동일 물체를 증명하지 않는다. 선택/점유/독립 보호는
호출자가 별도로 검사한다. 기본 비활성 교통 후보 전용이다.
"""
import math
import numpy as np
from arena_autonomy.piecewise_wall import corner_path


def propose(points,side,offset,minimum_radius):
    p=np.asarray(points,dtype=float)
    fail={'accepted':False,'reason':'no_supported_fragment_pair','free_space_certified':False}
    if side not in ('left','right') or p.ndim!=2 or p.shape[1]!=2 or len(p)<24 or not np.all(np.isfinite(p)):
        return None,{**fail,'reason':'invalid_input'}
    sign=1 if side=='left' else -1
    order=np.argsort(-sign*np.arctan2(p[:,1],p[:,0]),kind='stable')
    ordered=p[order]
    limits=.10+.035*np.minimum(np.linalg.norm(ordered[:-1],axis=1),np.linalg.norm(ordered[1:],axis=1))
    components=np.split(order,np.flatnonzero(np.linalg.norm(np.diff(ordered,axis=0),axis=1)>limits)+1)
    options=[]
    for ia,aidx in enumerate(components):
        a=p[aidx];near=(np.abs(a[:,0])<=.35)&(sign*a[:,1]>.10)&(np.abs(a[:,1]-sign*offset)<=.12)
        an=a[near]
        if len(an)<12:continue
        inliers=an.copy()
        for _ in range(3):
            center=inliers.mean(axis=0);t=np.linalg.svd(inliers-center,full_matrices=False)[2][0]
            if t[0]<0:t=-t
            n=np.array([-t[1],t[0]])
            good=np.abs((an-center)@n)<=.03
            if np.count_nonzero(good)<max(12,.7*len(an)):break
            inliers=an[good]
        if np.count_nonzero(good)<max(12,.7*len(an)) or t[0]<math.cos(math.radians(30)):continue
        # Keep only the initial straight component, no unseen returns are inserted.
        mask=(np.abs((a-center)@n)<=.03)&(a[:,0]>=-.3)&(a[:,0]<=2.5)
        a=a[mask];ai=aidx[mask]
        if len(a)<12 or np.ptp(a@t)<.25:continue
        apos=a@t;amax=float(np.max(apos))
        for ib,bidx in enumerate(components):
            if ia==ib or len(bidx)<12:continue
            b=p[bidx];aligned=(np.abs((b-center)@n)<=.03)&((b@t)>amax+.10)
            line=b[aligned]
            if len(line)<5 or np.ptp(line@t)<.15:continue
            bmin=float(np.min(line@t));gap=bmin-amax
            if not .10<gap<=1.2:continue
            # Require actual nearer returns inside the missing wall interval as an occlusion witness.
            witness=(p@t>amax)&(p@t<bmin)&(sign*((p-center)@n)<-.05)&(sign*p[:,1]>.08)
            if np.count_nonzero(witness)<3:continue
            # Do not jump a second missing component or extrapolate past observed B.
            starts=np.flatnonzero(aligned)
            b=b[int(starts[0]):];bi=bidx[int(starts[0]):]
            arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(b,axis=0),axis=1))]
            count=int(np.searchsorted(arc,1.8+2*offset,side='right'))
            b,bi=b[:count],bi[:count]
            fitted_a=center+((a-center)@t)[:,None]*t[None,:]
            joined=np.vstack((fitted_a[np.argsort(a@t)],b))
            path=corner_path(joined,side,offset,minimum_radius)
            if path is None:continue
            dist=float(np.min(np.linalg.norm(path,axis=1)))
            if dist>.12:continue
            ids=np.r_[ai[np.argsort(a@t)],bi]
            info=dict(accepted=True,reason='observed_collinear_fragments_with_nearer_returns',free_space_certified=False,
                      source_indices=ids.tolist(),occluder_indices=np.flatnonzero(witness).tolist(),gap_m=gap,
                      anchor_error_m=dist,points_used=len(ids),added_sensor_points=0)
            options.append((dist,path,info))
    if not options:return None,fail
    _,path,info=min(options,key=lambda x:x[0])
    return path,info
