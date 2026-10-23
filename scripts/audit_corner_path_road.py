#!/usr/bin/env python3
"""저장 센서로 만든 코너 경로와 평가용 실제 노면의 대조. 제어 입력으로 사용 금지."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from audit_local_pursuit_road import road_polygon
from build_experimental_track import rectangle,obstacle_polygons
from shapely.strtree import STRtree
from probe_short_corner_support import point_support


def exact_pose(rows,time_s):
    """부동소수 표현 오차 1 ns만 허용한다. 근처 자세 보간/대입 없음."""
    matched=[r for r in rows if abs(r[0]-time_s)<=1e-9]
    if len(matched)!=1:raise ValueError('exact unique evaluation pose unavailable')
    values=np.asarray(matched[0],dtype=float)
    if values.shape!=(4,) or not np.all(np.isfinite(values)):
        raise ValueError('finite time/x/y/yaw pose required')
    return values[1:]


def bracketing_poses(rows,time_s,maximum_gap_s=.0021):
    """앞뒤 실측 자세만 반환한다. 가운데 자세나 중간 운동의 상한을 추정하지 않는다."""
    if not math.isfinite(time_s) or not 0<maximum_gap_s<=.0021:
        raise ValueError('bounded bracketing query required')
    before=[r for r in rows if r[0]<time_s]
    after=[r for r in rows if r[0]>time_s]
    if not before or not after:raise ValueError('two-sided pose coverage unavailable')
    lo,hi=max(before,key=lambda r:r[0]),min(after,key=lambda r:r[0])
    if hi[0]-lo[0]>maximum_gap_s:raise ValueError('pose bracket exceeds diagnostic budget')
    for row in (lo,hi):
        exact_pose(rows,row[0])  # 중복 시각/비유한 좌표는 민감도 검사에도 사용하지 않는다.
    return [lo,hi]


def summary_metrics(rows):
    evaluated=[r for r in rows if r['evaluated']]
    sensitivity=[r for r in rows if r.get('sensitivity_evaluated',False)]
    return dict(scenes=len(rows),evaluated=len(evaluated),metric_available=bool(evaluated),
        outside_samples=sum(r['audit']['outside_over_1cm2'] for r in evaluated) if evaluated else None,
        wall_overlap_samples=sum(r['audit']['wall_overlap_over_1mm2'] for r in evaluated) if evaluated else None,
        sensitivity_scenes=len(sensitivity),
        sensitivity_pose_samples=sum(len(r['bracket_evaluations']) for r in sensitivity),
        sensitivity_outside_samples=sum(a['audit']['outside_over_1cm2'] for r in sensitivity
            for a in r['bracket_evaluations']) if sensitivity else None,
        sensitivity_wall_overlap_samples=sum(a['audit']['wall_overlap_over_1mm2'] for r in sensitivity
            for a in r['bracket_evaluations']) if sensitivity else None)


def path_footprints(path,pose,road,obstacles=(),*,length=.20,width=.15):
    p=np.asarray(path,dtype=float)
    pose=np.asarray(pose,dtype=float)
    if (p.ndim!=2 or p.shape[1]!=2 or len(p)<3 or pose.shape!=(3,) or
        not np.all(np.isfinite(p)) or not np.all(np.isfinite(pose)) or
        not all(math.isfinite(x) and x>0 for x in (length,width))):
        raise ValueError('finite path and pose/footprint dimensions required')
    tangent=np.gradient(p,axis=0)
    if np.any(np.linalg.norm(tangent,axis=1)<=1e-9):raise ValueError('degenerate tangent')
    angle=np.arctan2(tangent[:,1],tangent[:,0])+pose[2]
    c,s=math.cos(pose[2]),math.sin(pose[2])
    world=p@np.array([[c,s],[-s,c]])+pose[:2]
    tree=STRtree(obstacles) if obstacles else None
    origin=int(np.argmin(np.linalg.norm(p,axis=1)))
    result=[]
    for i in range(origin,len(p)):
        body=rectangle(float(world[i,0]),float(world[i,1]),float(angle[i]),length,width)
        outside=body.difference(road).area
        candidates=tree.query(body) if tree is not None else ()
        overlap=max((body.intersection(obstacles[int(j)]).area for j in candidates),default=0.)
        result.append(dict(path_index=i,world_x_m=float(world[i,0]),world_y_m=float(world[i,1]),
            yaw_rad=float(angle[i]),outside_road_area_m2=float(outside),wall_overlap_area_m2=float(overlap)))
    return dict(samples=result,maximum_outside_road_area_m2=max(r['outside_road_area_m2'] for r in result),
        outside_over_1cm2=sum(r['outside_road_area_m2']>.0001 for r in result),
        maximum_wall_overlap_area_m2=max(r['wall_overlap_area_m2'] for r in result),
        wall_overlap_over_1mm2=sum(r['wall_overlap_area_m2']>1e-6 for r in result),
        continuous_sweep_verified=False,tracking_dynamics_verified=False,driving_authorized=False)


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('geometry',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--pose-mode',choices=('exact','bracket_sensitivity'),default='exact')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    inputs=[args.run/'runtime_world.sdf',args.run/'ego_poses.jsonl',args.geometry]
    hashes={str(p):digest(p) for p in inputs}
    root=Path(__file__).resolve().parents[1]
    files=['scripts/audit_corner_path_road.py','scripts/audit_local_pursuit_road.py',
        'scripts/build_experimental_track.py','scripts/probe_short_corner_support.py','tests/test_corner_path_road.py']
    sources={p:digest(root/p) for p in files}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in files:archive.write(root/p,p)
    road=road_polygon(inputs[0])
    obstacles=obstacle_polygons(inputs[0])
    with inputs[1].open() as stream:poses=[json.loads(line) for line in stream]
    geometry=json.loads(args.geometry.read_text())
    results=[]
    for row in geometry['rows']:
        side=row['recorded'][2]['preferred_wall']
        other='right' if side=='left' else 'left'
        path=row['paths'][other]['path']
        points=np.asarray(row['points'])
        if path is None or not row['components']:
            results.append(dict(seq=row['seq'],evaluated=False,reason='missing_sensor_path_or_component'))
            continue
        sign=1 if side=='left' else -1
        near=min(row['components'],key=lambda part:float(np.min(np.linalg.norm(points[part['indices']]-[0,sign*.425],axis=1))))
        support=point_support(points[near['indices']],path,side)
        origin=int(np.argmin(np.linalg.norm(path,axis=1)))
        retained=np.asarray(path)[:origin+support['supported_prefix_samples']]
        if len(retained)<3:
            results.append(dict(seq=row['seq'],time_s=row['t'],evaluated=False,reason='geometry_prefix_short'))
            continue
        try:pose=exact_pose(poses,row['t'])
        except ValueError as error:
            result=dict(seq=row['seq'],time_s=row['t'],evaluated=False,reason=str(error))
            if args.pose_mode=='bracket_sensitivity':
                try:
                    bracket=bracketing_poses(poses,row['t'])
                    result.update(sensitivity_evaluated=True,contemporaneous_pose_verified=False,
                        reference_time_inside_bracket=True,bracket_span_s=bracket[1][0]-bracket[0][0],
                        support_prefix_m=support['path_prefix_m'],retained_path=retained.tolist(),
                        bracket_evaluations=[dict(pose_time_s=r[0],offset_from_reference_s=r[0]-row['t'],
                            pose=r[1:],audit=path_footprints(retained,r[1:],road,obstacles)) for r in bracket])
                except ValueError as bracket_error:
                    result.update(sensitivity_evaluated=False,sensitivity_error=str(bracket_error))
            results.append(result)
            continue
        results.append(dict(seq=row['seq'],time_s=row['t'],evaluated=True,pose=pose.tolist(),
            support_prefix_m=support['path_prefix_m'],retained_path=retained.tolist(),
            original_reason=row['evidence']['reason'],audit=path_footprints(retained,pose,road,obstacles)))
    report=dict(scope='evaluation_truth_after_sensor_path_generation_only',pose_mode=args.pose_mode,
        rows=results,summary=summary_metrics(results),
        input_sha256=hashes,input_unchanged=all(digest(p)==v for p,v in hashes.items()),
        source_sha256=sources,source_unchanged=all(digest(root/p)==v for p,v in sources.items()),
        road_truth_to_controller=False,driving_authorized=False,runtime_changed=False)
    with (args.output/'report.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(**report['summary'],
        input_unchanged=report['input_unchanged'],source_unchanged=report['source_unchanged'])))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
