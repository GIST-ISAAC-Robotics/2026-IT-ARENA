"""보존된 중앙 정체 5장면의 관측 점유만으로 평행 통과 폭의 필요조건을 비교한다.

경로·새 관측·도로/미관측·추종/제동 검사를 통과했다는 뜻이 아니다.
정답 자세는 사용하지 않고 당시 분류된 외벽 반환으로 좌표를 잡는다.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
INPUT=ROOT/'artifacts/validation/2026-10-29/g2_tracking_preview_v1/prototype2_stop_diagnosis.json'


def subtract(intervals,blocked):
    result=[]
    a,b=blocked
    for lo,hi in intervals:
        if b<=lo or a>=hi:result.append((lo,hi));continue
        if a>lo:result.append((lo,min(hi,a)))
        if b<hi:result.append((max(lo,b),hi))
    return result


def allowed(points,padding,station,upper=.295):
    # 차체를 관측 외벽과 평행하게 놓은 점유 필요조건이다. 전이 경로가 아니다.
    intervals=[(0.,upper)]
    for (x,y),p in zip(points,padding):
        if abs(x-station)<=.1+p:
            intervals=subtract(intervals,(y-.075-p,y+.075+p))
    return intervals


def intersection(a,b):
    return [(max(x,u),min(y,v)) for x,y in a for u,v in b if max(x,u)<min(y,v)]


def analyze(row):
    geometry=row['geometry'];points=np.asarray(geometry['points']);padding=np.asarray(geometry['padding_m'])
    mask=np.isclose(padding,.055,atol=1e-12,rtol=0.)
    if int(mask.sum())!=geometry['classification']['static_return_count']:
        raise ValueError('static classification count mismatch')
    wall=points[mask]
    slope,intercept=np.linalg.lstsq(np.c_[wall[:,0],np.ones(len(wall))],wall[:,1],rcond=None)[0]
    norm=math.hypot(1.,float(slope))
    tangent=np.asarray([1.,slope])/norm;normal=np.asarray([-slope,1.])/norm
    # 0.425 m는 현재 제어기의 벽-기준선 가정이며 지도 정답을 읽지 않는다.
    origin_y=float(intercept)/norm-.425
    rotated=np.c_[points@tangent,points@normal-origin_y]
    fit_residual=np.abs(wall@normal-float(intercept)/norm)
    upper=.425-.075-.055
    rows=[];common=[(0.,upper)]
    for station in np.arange(0.,.901,.02):
        intervals=allowed(rotated,padding,float(station),upper)
        common=intersection(common,intervals)
        rows.append(dict(station_m=float(station),allowed_offset_intervals_m=intervals))
    return dict(reference_s=row['t'],source_first_s=geometry['source_first_s'],
        fit=dict(slope=float(slope),intercept=float(intercept),static_count=int(mask.sum()),
            normal_residual_median_m=float(np.median(fit_residual)),normal_residual_max_m=float(np.max(fit_residual))),
        observed_wall_offset_assumption_m=.425,parallel_center_upper_m=upper,
        original_registered_offsets_m=[.16,.20,.24,.27],
        fixed_parallel_band_0_to_09m=common,
        registered_offsets_in_band=[x for x in (.16,.20,.24,.27) if any(lo<x<hi for lo,hi in common)],
        stations=rows,points_wall_frame=rotated.tolist(),padding_m=padding.tolist(),
        scope='necessary point-occupancy condition only; no unseen free space, transition, grass or control authorization')


if __name__=='__main__':
    # 전방 모서리 점 하나와 외벽의 폭을 해석식과 대조한다.
    a=allowed(np.array([[.4,.075]]),np.array([.13]),.4)
    assert len(a)==1 and np.allclose(a[0],[.28,.295])
    assert allowed(np.array([[.4,.075]]),np.array([.15]),.4)==[]
    assert intersection([(0.,.2)],[(.1,.3)])==[(.1,.2)]
    before=hashlib.sha256(INPUT.read_bytes()).hexdigest()
    saved=json.loads(INPUT.read_text())
    if saved['matched']!=531 or saved['source_matched'] is not True:
        raise ValueError('expected earlier verified reconstruction')
    results=[analyze(row) for row in saved['samples']]
    report=dict(input=str(INPUT.relative_to(ROOT)),input_sha256=before,
        input_unchanged=hashlib.sha256(INPUT.read_bytes()).hexdigest()==before,
        analysis_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),results=results)
    out=BASE/'analysis1.json'
    with out.open('x',encoding='utf-8') as f:json.dump(report,f,indent=2,allow_nan=False)
    for row in results:
        print(json.dumps({k:row[k] for k in ('reference_s','fit','fixed_parallel_band_0_to_09m','registered_offsets_in_band')}))
