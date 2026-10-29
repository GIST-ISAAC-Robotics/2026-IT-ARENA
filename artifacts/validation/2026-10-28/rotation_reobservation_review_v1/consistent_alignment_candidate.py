"""미연결 비교안 v2: 같은 표본·현·가중치의 오차로 작은 정합을 검증한다."""
import math
import numpy as np
from arena_autonomy.partial_surface_alignment import surface_translation as original_translation


def bounded_solution(result):
    if result.get('reason') not in ('weak_improvement','bounded_surface_correspondence'):
        return None
    rows=result.get('steps',[])
    if not rows:return None
    try:
        step=np.asarray([r['step_m'] for r in rows],dtype=float)
        residual=np.asarray([r['residual_rms_m'] for r in rows],dtype=float)
        support=np.asarray([[r['fraction'],r['normal_eigen_ratio']] for r in rows],dtype=float)
    except (TypeError,ValueError,KeyError):return None
    if (step.shape!=(len(rows),2) or not all(np.all(np.isfinite(x)) for x in (step,residual,support))
        or np.any(residual<0) or residual[-1]>.005 or np.any(support[:,0]<.65)
        or np.any(support[:,1]<.02) or np.linalg.norm(step[-1])>=1e-5):return None
    shift=step.sum(axis=0)
    return shift if np.linalg.norm(shift)<=.01 else None


def common_support_loss(old,current,source_indices,shift):
    """처음 선택한 현에 대해 이동 전/후를 같은 표본과 가중치로 대조한다."""
    old=np.asarray(old,dtype=float);points=np.asarray(current,dtype=float)
    ids=np.asarray(source_indices);shift=np.asarray(shift,dtype=float)
    if (old.ndim!=2 or old.shape[1]!=2 or not 50<=len(old)<=1024 or
        points.ndim!=2 or points.shape[1]!=2 or not 50<=len(points)<=1024 or
        ids.shape!=(len(points),) or not np.issubdtype(ids.dtype,np.integer) or
        np.any(np.diff(ids)<=0) or shift.shape!=(2,) or
        not all(np.all(np.isfinite(v)) for v in (old,points,shift))):
        return dict(passed=False,reason='invalid_support')
    chosen=(np.linalg.norm(points,axis=1)>=.15)&(np.linalg.norm(points,axis=1)<=3.5)
    points,ids=points[chosen],ids[chosen]
    if len(points)<3:return dict(passed=False,reason='no_surface')
    vector=np.diff(points,axis=0);length=np.linalg.norm(vector,axis=1)
    valid=(np.diff(ids)==1)&(length>.001)&(length<=.12)
    cosine=np.sum(vector[:-1]*vector[1:],axis=1)/np.maximum(1e-12,length[:-1]*length[1:])
    triple=valid[:-1]&valid[1:]&(cosine>=math.cos(math.radians(30)))
    supported=np.zeros(len(vector),bool);supported[:-1]|=triple;supported[1:]|=triple
    a=points[:-1][supported];vector=vector[supported];length=length[supported]
    if len(a)<30:return dict(passed=False,reason='few_surface_segments')
    normal=np.column_stack((-vector[:,1],vector[:,0]))/length[:,None]
    delta=old[:,None,:]-a[None,:,:]
    fraction=np.sum(delta*vector[None,:,:],axis=2)/length**2
    distance=np.linalg.norm(delta-np.clip(fraction,0,1)[:,:,None]*vector[None,:,:],axis=2)
    nearest=np.argmin(distance,axis=1);rows=np.arange(len(old))
    before_ok=(distance[rows,nearest]<.08)&(fraction[rows,nearest]>=0)&(fraction[rows,nearest]<=1)
    f_after=np.sum((old+shift-a[nearest])*vector[nearest],axis=1)/length[nearest]**2
    after_ok=(f_after>=0)&(f_after<=1)
    keep=before_ok&after_ok
    n=int(keep.sum());before_n=int(before_ok.sum())
    if n<50 or n/len(old)<.65 or n<.95*before_n:
        return dict(passed=False,reason='fixed_support_lost',matches=n,before_matches=before_n)
    normals=normal[nearest[keep]]
    before=np.sum((old[keep]-a[nearest[keep]])*normals,axis=1)
    after=before+normals@shift
    weights=np.minimum(1.,.015/np.maximum(np.abs(before),1e-12))
    spectrum=np.linalg.eigvalsh((normals*np.sqrt(weights)[:,None]).T@(normals*np.sqrt(weights)[:,None]))
    ratio=float(spectrum[0]/max(spectrum[-1],1e-12))
    rms0=float(np.sqrt(np.average(before**2,weights=weights)))
    rms1=float(np.sqrt(np.average(after**2,weights=weights)))
    return dict(passed=ratio>=.02 and rms1<=.005 and rms1<=rms0+1e-9,
        reason='fixed_correspondences',matches=n,before_matches=before_n,
        before_rms_m=rms0,after_rms_m=rms1,normal_eigen_ratio=ratio)


def surface_translation(previous_now,current,source_indices):
    result=original_translation(previous_now,current,source_indices)
    if result.get('accepted') or result.get('reason')!='weak_improvement':return result
    shift=bounded_solution(result)
    if shift is None:return result
    old=np.asarray(previous_now,dtype=float).reshape(-1,2)
    if len(old)<100:return result
    diagnostic=dict(full=common_support_loss(old,current,source_indices,shift),partitions=[])
    result['fixed_split_probe']=diagnostic
    if not diagnostic['full']['passed']:return result
    pieces=[]
    for parity in (0,1):
        check=original_translation(old[parity::2],current,source_indices)
        piece=bounded_solution(check)
        validation=None if piece is None else common_support_loss(old[1-parity::2],current,source_indices,piece)
        diagnostic['partitions'].append(dict(parity=parity,fit_reason=check.get('reason'),
            shift_m=None if piece is None else piece.tolist(),heldout=validation))
        if piece is None or np.linalg.norm(piece-shift)>.002 or not validation['passed']:return result
        pieces.append(piece)
    spread=float(np.linalg.norm(pieces[0]-pieces[1]))
    if spread>.002:return result
    return {**result,'accepted':True,'reason':'small_fixed_support_consistent',
        'translation_m':shift.tolist(),'split_difference_m':spread,'trial_only':True}
