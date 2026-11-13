"""선택형 재관측: 비스듬한 실제 표면의 드문 표본 간격을 구분한다.

연속 반환 네 점의 국소 형상을 요구하며 미관측 광선을 잇거나 현재 점군을
바꾸지 않는다. 같은 물체의 신원 또는 정적 배경임을 증명하지는 않는다.
"""
import math

import numpy as np

from arena_autonomy.partial_observation import short_observed_surface
from arena_autonomy.partial_surface_alignment import surface_translation as original_translation


def supported_surface(point, current, source_indices):
    """기존 짧은 현의 성공을 유지하고 6–12 cm 현만 더 강한 지지로 검사한다."""
    legacy = short_observed_surface(point, current, source_indices)
    if legacy is not None:
        return legacy
    point = np.asarray(point, dtype=float)
    current = np.asarray(current, dtype=float)
    ids = np.asarray(source_indices)
    if (point.shape != (2,) or current.ndim != 2 or current.shape[1] != 2 or
            not 4 <= len(current) <= 1024 or ids.shape != (len(current),) or
            not np.issubdtype(ids.dtype, np.integer) or not np.all(np.isfinite(point)) or
            not np.all(np.isfinite(current)) or np.any(np.diff(ids) <= 0)):
        return None
    # 호출자와 같은4cm 이내 대응만 다룬다. 거리가 먼 배경으로 옛 점을 옮기지 않는다.
    if np.min(np.linalg.norm(current-point, axis=1)) > .04:
        return None
    vectors = np.diff(current, axis=0)
    lengths = np.linalg.norm(vectors, axis=1)
    adjacent = (np.diff(ids) == 1) & (lengths > 1e-6) & (lengths <= .12)
    cosine = np.sum(vectors[:-1]*vectors[1:], axis=1) / np.maximum(1e-12, lengths[:-1]*lengths[1:])
    straight = cosine >= math.cos(math.radians(20))
    starts = np.flatnonzero(adjacent[:-2] & adjacent[1:-1] & adjacent[2:] & straight[:-1] & straight[1:])
    for index in starts:
        if not np.any(lengths[index:index+3] > .06):
            continue
        patch = current[index:index+4]
        centered = patch-patch.mean(axis=0)
        _, _, axes = np.linalg.svd(centered, full_matrices=False)
        tangent, normal = axes
        residual = np.abs(centered @ normal)
        if np.max(residual) > .005:
            continue
        along = patch @ tangent
        if not (np.all(np.diff(along) > 0) or np.all(np.diff(along) < 0)):
            continue
        for start in range(index, index+3):
            fraction = float((point-current[start]) @ vectors[start] / lengths[start]**2)
            if not 0 <= fraction <= 1:
                continue
            distance = float(np.linalg.norm(point-current[start]-fraction*vectors[start]))
            if distance <= .01:
                return dict(source_indices=ids[index:index+4].tolist(), current_points=patch.tolist(),
                    normal_distance_m=distance, segment_lengths_m=lengths[index:index+3].tolist(),
                    support_kind='four_consecutive_observed_returns',
                    maximum_line_residual_m=float(np.max(residual)), unknown_filled=False)
    return None


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
    """현을 무한 연장하지 않고 같은 유한 선분·표본·가중치로 대조한다."""
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
    # 작은 접선 이동이 같은 벽의 다음 현으로 넘어가는 것은 관측 소실이 아니다.
    # 최초 선분을 바꾸지 않고 끝점까지의 실제 유한 거리를 오차에 포함한다.
    # 끝 밖을 무한 평면으로 투영해 0 오차로 처리하지 않는다.
    keep=before_ok
    n=int(keep.sum());before_n=int(before_ok.sum())
    if n<50 or n/len(old)<.65 or n<.95*before_n:
        return dict(passed=False,reason='fixed_support_lost',matches=n,before_matches=before_n)
    normals=normal[nearest[keep]]
    before=distance[rows[keep],nearest[keep]]
    closest=a[nearest[keep]]+np.clip(f_after[keep],0,1)[:,None]*vector[nearest[keep]]
    after=np.linalg.norm(old[keep]+shift-closest,axis=1)
    weights=np.minimum(1.,.015/np.maximum(np.abs(before),1e-12))
    spectrum=np.linalg.eigvalsh((normals*np.sqrt(weights)[:,None]).T@(normals*np.sqrt(weights)[:,None]))
    ratio=float(spectrum[0]/max(spectrum[-1],1e-12))
    rms0=float(np.sqrt(np.average(before**2,weights=weights)))
    rms1=float(np.sqrt(np.average(after**2,weights=weights)))
    return dict(passed=ratio>=.02 and rms1<=.005 and rms1<=rms0+1e-9,
        reason='fixed_finite_segments',matches=n,before_matches=before_n,
        beyond_initial_segment=int(np.count_nonzero((f_after[keep]<0)|(f_after[keep]>1))),
        before_rms_m=rms0,after_rms_m=rms1,normal_eigen_ratio=ratio)


def small_surface_translation(previous_now,current,source_indices):
    result=original_translation(previous_now,current,source_indices)
    if result.get('accepted') or result.get('reason')!='weak_improvement':return result
    shift=bounded_solution(result)
    if shift is None:return result
    old=np.asarray(previous_now,dtype=float).reshape(-1,2)
    if len(old)<100:return result
    diagnostic=dict(full=common_support_loss(old,current,source_indices,shift),partitions=[])
    result['finite_split_probe']=diagnostic
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
    return {**result,'accepted':True,'reason':'small_finite_support_consistent',
        'translation_m':shift.tolist(),'split_difference_m':spread,'trial_only':True}
