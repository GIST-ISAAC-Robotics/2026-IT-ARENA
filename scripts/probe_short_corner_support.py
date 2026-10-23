#!/usr/bin/env python3
"""짧은 표면의 경로 간격 지지 가능성과 정체성 모호함. 제어/한도 변경 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.corner_distance import corner_distance_evidence
from probe_corner_observability import annular_scan
from profile_follow_trace import SNAPSHOT_MODULES


def point_support(points,reference,side,offset=.425,tolerance=.08):
    """실제 표본점만의 간격·법선 대조. 벽 분류/자유 공간/연결/주행 허가가 아니다."""
    p,path=np.asarray(points,dtype=float),np.asarray(reference,dtype=float)
    if (side not in ('left','right') or p.ndim!=2 or path.ndim!=2 or
        p.shape[1]!=2 or path.shape[1]!=2 or len(p)<2 or len(path)<3 or
        not np.all(np.isfinite(p)) or not np.all(np.isfinite(path))):
        raise ValueError('finite observed points and path required')
    step=np.linalg.norm(np.diff(path,axis=0),axis=1)
    tangent=np.gradient(path,axis=0)
    magnitude=np.linalg.norm(tangent,axis=1)
    if np.any(step<=1e-9) or np.any(magnitude<=1e-9):raise ValueError('nondegenerate path required')
    tangent/=magnitude[:,None]
    sign=1 if side=='left' else -1
    normal=sign*np.column_stack((-tangent[:,1],tangent[:,0]))
    origin=int(np.argmin(np.linalg.norm(path,axis=1)))
    arc=np.r_[0.,np.cumsum(step)]
    delta=p[None]-path[origin:,None]
    distances=np.linalg.norm(delta,axis=2)
    chosen=np.argmin(distances,axis=1)
    distance=distances[np.arange(len(chosen)),chosen]
    vector=delta[np.arange(len(chosen)),chosen]
    cosine=np.sum(vector*normal[origin:],axis=1)/np.maximum(distance,1e-9)
    supported=(np.abs(distance-offset)<=tolerance)&(cosine>=math.cos(math.radians(30)))
    invalid=np.flatnonzero(~supported)
    end=int(invalid[0]) if len(invalid) else len(chosen)
    extent=float(np.max(np.linalg.norm(p[:,None]-p[None],axis=2)))
    return dict(observed_extent_m=extent,point_count=len(p),
        path_prefix_m=float(arc[origin+end-1]-arc[origin]) if end else 0.,
        current_path_ahead_m=float(arc[-1]-arc[origin]),supported_prefix_samples=end,
        nearest_point_indices=chosen.tolist(),distance_m=distance.tolist(),normal_cosine=cosine.tolist(),
        is_wall_evidence=False,is_drive_permission=False,free_area_certified=False,
        unknown_gaps_filled=False,added_sensor_points=0)


def compact_circle_fixture(sign=1):
    # 지름 0.15 m 원은 0.20×0.15 m 외형 안에 든다.
    # 전자 부품/차량 물리 재현이 아닌 2D 관측의 기하 반례다.
    radius=.075
    ranges,labels,directions,origin,center=annular_scan(radius,sign)
    angles=np.arctan2(directions[:,1],directions[:,0])
    indices=np.flatnonzero(labels & (np.abs(angles)<=math.radians(150)))
    points=origin+directions[indices]*ranges[indices,None]
    theta=np.r_[np.linspace(-.3,0.,13),np.linspace(0.,math.pi/2,65)[1:]]
    path=np.column_stack(((radius+.425)*np.sin(theta),sign*(radius+.425)*(1-np.cos(theta))))
    return points,path,dict(footprint_length_m=.15,footprint_width_m=.15,radius_m=radius,
        geometry='first-return circle plus outer circle; inner returns only in geometry test',
        hypotheses=['small_static_boundary','stationary_compact_object'],
        indistinguishable_geometry=True,physical_vehicle_verified=False,
        labels_used_by_controller=False,source_indices=indices.tolist())


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('geometry_report',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    root=Path(__file__).resolve().parents[1]
    before=digest(args.geometry_report)
    sources=[f'src/{pkg}/{pkg}/{mod}.py' for pkg,mods in SNAPSHOT_MODULES.items() for mod in mods]
    sources+=['scripts/probe_short_corner_support.py','scripts/probe_corner_observability.py',
        'scripts/inspect_corner_geometry.py','scripts/profile_follow_trace.py',
        'tests/test_short_corner_support.py','tests/test_corner_distance_probe.py']
    hashes={p:digest(root/p) for p in sources}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources:archive.write(root/p,p)
    data=json.loads(args.geometry_report.read_text())
    rows=[]
    for row in data['rows']:
        points=np.asarray(row['points'])
        side=row['recorded'][2]['preferred_wall']
        sign=1 if side=='left' else -1
        other='right' if side=='left' else 'left'
        path=row['paths'][other]['path']
        if path is None or not row['components']:continue
        near=min(row['components'],key=lambda part:float(np.min(np.linalg.norm(points[part['indices']]-[0,sign*.425],axis=1))))
        result=point_support(points[near['indices']],path,side)
        rows.append(dict(seq=row['seq'],time_s=row['t'],source_indices=near['indices'],geometry=result,
            original_candidate_reason=row['evidence']['reason']))
    fixtures=[]
    for sign in (1,-1):
        p,path,info=compact_circle_fixture(sign)
        side='left' if sign==1 else 'right'
        geometry=point_support(p,path,side)
        original=corner_distance_evidence(p,path,side,FollowConfig())
        fixtures.append(dict(side=side,fixture=info,geometry=geometry,original=original,
            geometry_supports_065_m=geometry['path_prefix_m']>=.65,
            existing_length_contract_rejects=not original['supported']))
    result=dict(scope='shadow_geometry_only_not_candidate_controller',saved_rows=rows,fixtures=fixtures,
        input_sha256=before,input_unchanged=before==digest(args.geometry_report),source_sha256=hashes,
        source_unchanged=all(digest(root/p)==v for p,v in hashes.items()),
        runtime_changed=False,thresholds_changed=False,driving_authorized=False)
    with (args.output/'report.json').open('x') as stream:json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(saved_scenes=len(rows),fixtures=len(fixtures),
        compact_counterexamples=sum(f['geometry_supports_065_m'] and f['existing_length_contract_rejects'] for f in fixtures),
        input_unchanged=result['input_unchanged'],source_unchanged=result['source_unchanged'])))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 2


if __name__=='__main__':raise SystemExit(main())
