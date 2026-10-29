"""미연결 비교안: 작은 정합의 절대 잔차·분할 일관성. 제조사 보장값 아님."""
import numpy as np
from arena_autonomy.partial_surface_alignment import surface_translation as original_translation


def small_converged(result):
    if result.get('reason') not in ('weak_improvement', 'bounded_surface_correspondence'):
        return None
    rows=result.get('steps',[])
    if not rows:
        return None
    try:
        steps=np.asarray([r['step_m'] for r in rows],dtype=float)
        residual=np.asarray([r['residual_rms_m'] for r in rows],dtype=float)
        fraction=np.asarray([r['fraction'] for r in rows],dtype=float)
        spectrum=np.asarray([r['normal_eigen_ratio'] for r in rows],dtype=float)
    except (TypeError,ValueError,KeyError):
        return None
    if (steps.shape!=(len(rows),2) or
        not all(np.all(np.isfinite(v)) for v in (steps,residual,fraction,spectrum))):
        return None
    shift=np.sum(steps,axis=0)
    if (np.linalg.norm(steps[-1])>=1e-5 or np.linalg.norm(shift)>.01 or
        np.any(residual<0) or residual[-1]>.005 or residual[-1]>residual[0] or
        np.any(fraction<.65) or np.any(spectrum<.02)):
        return None
    return shift


def surface_translation(previous_now,current,source_indices):
    result=original_translation(previous_now,current,source_indices)
    if result.get('accepted') or result.get('reason')!='weak_improvement':
        return result
    shift=small_converged(result)
    if shift is None:
        return result
    result['small_split_probe']=dict(full_quality_passed=True,partitions=[])
    old=np.asarray(previous_now,dtype=float).reshape(-1,2)
    if len(old)<100:
        return result
    pieces=[]
    for parity in (0,1):
        check=original_translation(old[parity::2],current,source_indices)
        piece=small_converged(check)
        result['small_split_probe']['partitions'].append(dict(parity=parity,
            reason=check.get('reason'),steps=check.get('steps'),quality_passed=piece is not None,
            shift_m=None if piece is None else piece.tolist(),
            difference_to_full_m=None if piece is None else float(np.linalg.norm(piece-shift))))
        if piece is None or np.linalg.norm(piece-shift)>.002:
            return result
        pieces.append(piece)
    spread=float(np.linalg.norm(pieces[0]-pieces[1]))
    if spread>.002:
        return result
    return {**result,'accepted':True,'reason':'small_split_consistent_correspondence',
        'translation_m':shift.tolist(),'relative_gain_rejected':True,
        'split_translations_m':[p.tolist() for p in pieces], 'split_difference_m':spread,
        'maximum_translation_m':.01,'maximum_residual_m':.005,
        'maximum_split_difference_m':.002,'trial_only':True}
