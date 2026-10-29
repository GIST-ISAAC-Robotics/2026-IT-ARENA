"""run3의 서로 다른 센서 표본에서 경계 끝·정합 잔차를 분리한다. 운영값 변경 없음."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call, freeze, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.partial_surface_alignment import surface_translation
from extract_latch import digest


def local_geometry(point,current,ids):
    distances=np.linalg.norm(current-point,axis=1)
    nearest=int(np.argmin(distances))
    rows=[]
    for i in range(max(0,nearest-2),min(len(current)-2,nearest+2)):
        patch=current[i:i+3]
        vectors=np.diff(patch,axis=0)
        lengths=np.linalg.norm(vectors,axis=1)
        if not np.all(lengths>1e-8):continue
        cosine=float(vectors[0]@vectors[1]/np.prod(lengths))
        edge=[]
        for j in range(2):
            fraction=float((point-patch[j])@vectors[j]/lengths[j]**2)
            normal=float(np.linalg.norm(point-patch[j]-fraction*vectors[j]))
            edge.append(dict(fraction=fraction,normal_m=normal,
                outside_distance_m=max(0.,-fraction,fraction-1)*float(lengths[j])))
        rows.append(dict(ids=ids[i:i+3].tolist(),points=patch.tolist(),
            lengths_m=lengths.tolist(),direction_change_deg=float(np.degrees(np.arccos(np.clip(cosine,-1,1)))),
            edges=edge))
    return dict(nearest_distance_m=float(distances[nearest]),nearest_id=int(ids[nearest]),
        nearest_point=current[nearest].tolist(),triples=rows)


def main(fixture,output):
    if output.exists():raise FileExistsError(output)
    before=digest(fixture)
    data=thaw(json.loads(fixture.read_text(encoding='utf-8')))
    evidence=data['evidence']; calls=data['calls']
    motion=MotionConfig(**data['header']['motion_config'])
    event=next(row for row in calls if abs(row['input']['reference_s']-evidence['previous_reference_s'])<1e-8)
    original,scan,history=restore_call(event['input'],motion)
    old,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,
        scan.range_max,evidence['previous_source_first_s'],scan.time_increment,original['reference_s'],'both',True)
    keep=np.linalg.norm(old,axis=1)<=3.;old,ids=old[keep],ids[keep]
    context=evidence['correspondence_context']
    np.testing.assert_allclose(old,context['previous_points_at_reference_xy'],atol=1e-8,rtol=0)
    carried=np.asarray(context['previous_points_now_xy'])
    retained_ids=set()
    for witness in evidence['witnesses']:
        d=np.linalg.norm(carried-witness['previous_point_now_xy'],axis=1)
        idx=int(np.argmin(d));assert d[idx]<1e-8
        retained_ids.add(idx)
    retained=old[sorted(retained_ids)]
    probe=SurfaceAnchorTrial(extended_surface=True)
    probe.start(retained,old,ids,evidence['previous_reference_s'],evidence['previous_source_first_s'])
    rows=[];last=None
    for event in calls:
        inp,scan,history=restore_call(event['input'],motion)
        now=inp['reference_s']
        if now<evidence['reference_s']:continue
        source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
        current,ids,_=history.project(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,
            scan.range_max,source,scan.time_increment,now,'both',True)
        # 승인되지 않은 정합 결과도 진단만 계산한다. probe 상태에는 적용하지 않는다.
        corrected=None;fit=None
        if probe.anchor is not None and not probe.ended and source!=last:
            old,reference,_=probe.anchor
            moved=transport_points(old,history,reference,now)
            carried_retained=transport_points(probe.retained,history,reference,now)
            fit=surface_translation(moved,current,ids)
            steps=fit.get('steps',[])
            shift=np.sum([r['step_m'] for r in steps],axis=0) if steps else np.zeros(2)
            corrected=carried_retained+shift
            fit.pop('processing_ms',None)
        actual=probe.observe(current,ids,history,now,source)
        if source==last:continue
        last=source
        rows.append(dict(t=now,actual={k:actual.get(k) for k in ('reason','matched','consecutive_fresh_sources')},
            diagnostic_fit=fit,diagnostic_geometry=[] if corrected is None else
            [local_geometry(point,current,ids) for point in corrected]))
        if probe.ended:break
    result=dict(scope='isolated unchanged anchor and hypothetical rejected-fit geometry; no acceptance change',
        fixture_sha256=before,input_unchanged=digest(fixture)==before,rows=rows)
    output.write_text(json.dumps(freeze(result),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps([dict(t=r['t'],actual=r['actual'],fit=(r['diagnostic_fit']or{}).get('reason')) for r in rows]))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('fixture',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    main(args.fixture,args.output)
