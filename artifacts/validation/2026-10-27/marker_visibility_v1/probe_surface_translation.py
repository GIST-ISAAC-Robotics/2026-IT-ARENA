"""고정된 실패 점군의 표면 정합 가설. 제어 연결/자유 공간/물체 신원 판정 없음."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def fit_translation(old, current, ids):
    old=np.asarray(old,dtype=float).reshape(-1,2)
    current=np.asarray(current,dtype=float).reshape(-1,2)
    ids=np.asarray(ids)
    output=dict(accepted=False,scope='offline_surface_translation_hypothesis',yaw_correction_rad=0.)
    if len(current)!=len(ids) or len(old)<30 or len(current)<3:
        return {**output,'reason':'insufficient_points'}
    edge=np.diff(current,axis=0)
    sizes=np.linalg.norm(edge,axis=1)
    valid=(np.diff(ids)==1) & (sizes>.001) & (sizes<.12)
    cosine=np.sum(edge[:-1]*edge[1:],axis=1)/np.maximum(1e-12,sizes[:-1]*sizes[1:])
    triple=valid[:-1]&valid[1:]&(cosine>np.cos(np.radians(30)))
    supported=np.zeros(len(edge),dtype=bool)
    supported[:-1]|=triple;supported[1:]|=triple
    a=current[:-1][supported];v=edge[supported];length=sizes[supported]
    if len(a)<10:return {**output,'reason':'insufficient_observed_surface'}
    normal=np.column_stack((-v[:,1],v[:,0]))/length[:,None]
    shift=np.zeros(2);steps=[]
    for _ in range(6):
        points=old+shift
        delta=points[:,None,:]-a[None,:,:]
        fraction=np.sum(delta*v[None,:,:],axis=2)/(length**2)
        residual=delta-np.clip(fraction,0,1)[:,:,None]*v[None,:,:]
        distance=np.linalg.norm(residual,axis=2)
        nearest=np.argmin(distance,axis=1)
        rows=np.arange(len(points));good=(distance[rows,nearest]<.08)&(fraction[rows,nearest]>=0)&(fraction[rows,nearest]<=1)
        rows=rows[good]
        if len(rows)<30:return {**output,'reason':'few_interior_matches','steps':steps}
        n=normal[nearest[rows]]
        error=np.sum((points[rows]-a[nearest[rows]])*n,axis=1)
        weights=np.minimum(1.,.015/np.maximum(np.abs(error),1e-12))
        weighted=n*np.sqrt(weights)[:,None]
        spectrum=np.linalg.eigvalsh(weighted.T@weighted)
        ratio=float(spectrum[0]/max(spectrum[-1],1e-12))
        step=np.linalg.lstsq(weighted,-error*np.sqrt(weights),rcond=None)[0]
        steps.append(dict(matches=len(error),normal_eigen_ratio=ratio,
                          residual_rms_m=float(np.sqrt(np.average(error**2,weights=weights))),step_m=step.tolist()))
        if ratio<.02:return {**output,'reason':'translation_unobservable','steps':steps}
        if np.linalg.norm(shift+step)>.06:
            return {**output,'reason':'translation_outside_bound','steps':steps}
        shift+=step
        if np.linalg.norm(step)<1e-5:break
    output.update(translation_m=shift.tolist(),steps=steps,
                  old_points=len(old),current_points=len(current),observed_segments=len(a))
    if steps[-1]['residual_rms_m']>.01:
        return {**output,'reason':'residual_high'}
    return {**output,'accepted':True,'reason':'bounded_observed_surface_fit_only'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    source=args.run/'trajectory.jsonl'
    for line in source.open(encoding='utf-8'):
        row=json.loads(line)
        evidence=((row.get('safety') or {}).get('partial_observation') or {}).get('latch_evidence')
        if evidence and 'correspondence_context' in evidence:break
    else:raise ValueError('no captured correspondence context')
    context=evidence['correspondence_context']
    result=fit_translation(context['previous_points_now_xy'],context['current_return_points_xy'],
                           context['current_return_source_indices'])
    result.update(record_time_s=row['time'],latch_reference_s=evidence['reference_s'],
                  source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in (source,Path(__file__))})
    with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
    print(json.dumps(result))


if __name__=='__main__':main()
