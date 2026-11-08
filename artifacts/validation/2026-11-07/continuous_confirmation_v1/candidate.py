"""현재 관측 경로의 공통 선분 대조 탐색. 센서 결측을 보간하지 않는다."""
import numpy as np

def agrees(candidate,reference,tolerance):
    paths=[np.asarray(p,dtype=float) for p in (candidate,reference)]
    if any(p.ndim!=2 or p.shape[1]!=2 or len(p)<5 or not np.all(np.isfinite(p))
           or np.any(np.diff(p[:,0])<=0) for p in paths):return False
    lo=max(.05,*(p[0,0] for p in paths));hi=min(.65,*(p[-1,0] for p in paths))
    if hi-lo<.25:return False
    # 원래 표본 수를 요구한다. 선분 끝의 보간점을 새 관측 표본으로 세지 않는다.
    if any(np.count_nonzero((p[:,0]>=lo)&(p[:,0]<=hi))<5 for p in paths):return False
    clipped=[]
    for p in paths:
        x=np.r_[lo,p[(p[:,0]>lo)&(p[:,0]<hi),0],hi]
        clipped.append(np.c_[x,np.interp(x,p[:,0],p[:,1])])
    def gap(p,q):
        delta=np.diff(q,axis=0);offset=p[:,None,:]-q[None,:-1,:]
        fraction=np.clip(np.sum(offset*delta,axis=2)/np.sum(delta*delta,axis=1),0.,1.)
        distance=np.linalg.norm(offset-fraction[:,:,None]*delta,axis=2)
        return float(np.max(np.min(distance,axis=1)))
    return max(gap(*clipped),gap(*clipped[::-1]))<=tolerance
