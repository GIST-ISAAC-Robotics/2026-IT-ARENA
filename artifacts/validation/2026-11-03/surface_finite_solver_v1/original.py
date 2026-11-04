"""짧은 스캔 사이의 공통 표면 이동 후보. 위치/속도 추정기로 사용하지 않는다.

gyro 회전은 그대로 둔다. 고정 표면이 충분하다는 가정이 필요하며, 여러 물체가
함께 움직이는 모호성은 해결하지 못한다. 현재 점군/unknown은 변경하지 않는다.
"""
import math
import time
import numpy as np


def surface_translation(previous_now, current, source_indices):
    started=time.perf_counter()
    output=dict(accepted=False,scope='correspondence_only_not_independent_motion',
                static_background_assumption=True,yaw_correction_rad=0.)
    def finish(reason,**extra):
        return {**output,**extra,'reason':reason,'processing_ms':(time.perf_counter()-started)*1000}
    old=np.asarray(previous_now,dtype=float).reshape(-1,2)
    current=np.asarray(current,dtype=float).reshape(-1,2)
    ids=np.asarray(source_indices)
    if (len(current)!=len(ids) or len(old)<50 or not 50<=len(current)<=1024 or
            not np.all(np.isfinite(old)) or not np.all(np.isfinite(current))):
        return finish('insufficient_or_invalid_points')
    # 큰 입력에도 행렬 크기를 제한한다. 현은 실제 인접 표본으로 먼저 만든다.
    old=old[::max(1,math.ceil(len(old)/240))]
    chosen=(np.linalg.norm(current,axis=1)>=.15)&(np.linalg.norm(current,axis=1)<=3.5)
    current,ids=current[chosen],ids[chosen]
    if (len(current)<50 or np.count_nonzero(current[:,1]>.15)<20 or
            np.count_nonzero(current[:,1]<-.15)<20 or np.ptp(current[:,0])<.7):
        return finish('localized_support')
    edge=np.diff(current,axis=0)
    lengths=np.linalg.norm(edge,axis=1)
    valid=(np.diff(ids)==1)&(lengths>.001)&(lengths<=.12)
    cosine=np.sum(edge[:-1]*edge[1:],axis=1)/np.maximum(1e-12,lengths[:-1]*lengths[1:])
    triple=valid[:-1]&valid[1:]&(cosine>=math.cos(math.radians(30)))
    supported=np.zeros(len(edge),dtype=bool)
    supported[:-1]|=triple;supported[1:]|=triple
    # 현을 띄엄띄엄 고르면 실제로 관측한 벽에 인위적인 구멍이 생긴다.
    # 현재 입력 개수 제한 안에서는 인접 현을 모두 유지한다.
    indices=np.flatnonzero(supported)
    a,v,length=current[:-1][indices],edge[indices],lengths[indices]
    if len(a)<30:return finish('insufficient_observed_surface')
    normals=np.column_stack((-v[:,1],v[:,0]))/length[:,None]
    shift=np.zeros(2);steps=[];converged=False
    for _ in range(6):
        points=old+shift
        delta=points[:,None,:]-a[None,:,:]
        fraction=np.sum(delta*v[None,:,:],axis=2)/(length**2)
        residual=delta-np.clip(fraction,0,1)[:,:,None]*v[None,:,:]
        distance=np.linalg.norm(residual,axis=2)
        nearest=np.argmin(distance,axis=1);rows=np.arange(len(points))
        good=(distance[rows,nearest]<.08)&(fraction[rows,nearest]>=0)&(fraction[rows,nearest]<=1)
        rows=rows[good]
        if len(rows)<50 or len(rows)/len(old)<.65:
            return finish('few_surface_matches',steps=steps,matched_points=len(rows),
                          sampled_previous_points=len(old),observed_segments=len(a))
        normal=normals[nearest[rows]]
        error=np.sum((points[rows]-a[nearest[rows]])*normal,axis=1)
        weights=np.minimum(1.,.015/np.maximum(np.abs(error),1e-12))
        weighted=normal*np.sqrt(weights)[:,None]
        spectrum=np.linalg.eigvalsh(weighted.T@weighted)
        ratio=float(spectrum[0]/max(spectrum[-1],1e-12))
        step=np.linalg.lstsq(weighted,-error*np.sqrt(weights),rcond=None)[0]
        rms=float(np.sqrt(np.average(error**2,weights=weights)))
        steps.append(dict(matches=len(rows),fraction=len(rows)/len(old),normal_eigen_ratio=ratio,
                          residual_rms_m=rms,step_m=step.tolist()))
        if ratio<.02:return finish('translation_unobservable',steps=steps)
        if np.linalg.norm(shift+step)>.06:
            return finish('translation_outside_bound',steps=steps)
        shift+=step
        if np.linalg.norm(step)<1e-5:
            converged=True
            break
    if not converged or steps[-1]['residual_rms_m']>.01:
        return finish('unsettled_or_high_residual',steps=steps)
    if np.linalg.norm(shift)>.002 and steps[-1]['residual_rms_m']>steps[0]['residual_rms_m']*.6:
        return finish('weak_improvement',steps=steps)
    return finish('bounded_surface_correspondence',accepted=True,translation_m=shift.tolist(),
                  steps=steps,observed_segments=len(a),sampled_previous_points=len(old))
