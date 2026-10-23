#!/usr/bin/env python3
"""관측 순서/점별 오차 원판에 따른 곡선 길이 하한. 경계 판정에 미연결."""
import argparse
import hashlib
import json
from pathlib import Path
import zipfile

import numpy as np

from inspect_corner_geometry import components
from probe_corner_observability import annular_scan

ROOT = Path(__file__).resolve().parents[1]


def ordered_length_lower_bound(points, error_m):
    """오차 원판을 순서대로 지난다는 조건하의 실제 곡선 길이 하한.

    어떤 부분수열도 실제 순서상 경로를 나눈다. 각 현의 길이에서 양 끝의
    오차 반지름을 빼고 합하면 삼각부등식에 의해 하한이다. DP는 가능한
    부분수열 중 가장 큰 하한만 찾는다. 하나의 물체/벽이라는 보증은 없다.
    """
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or not np.all(np.isfinite(p)):
        raise ValueError('finite ordered N x 2 points required')
    if isinstance(error_m, bool) or not np.isscalar(error_m) or not np.isfinite(error_m) or error_m < 0:
        raise ValueError('finite nonnegative point error required')
    if len(p) > 8192:
        raise ValueError('point count exceeds source ray bound')
    if len(p) < 2:
        return dict(lower_bound_m=0., witness_indices=[], diameter_lower_bound_m=0.)
    best = np.zeros(len(p))
    previous = np.full(len(p), -1, dtype=int)
    diameter = 0.
    for j in range(1, len(p)):
        gaps = np.maximum(0., np.linalg.norm(p[:j]-p[j], axis=1)-2*error_m)
        diameter = max(diameter, float(np.max(gaps)))
        values = best[:j]+gaps
        i = int(np.argmax(values))
        best[j], previous[j] = values[i], i
    end = int(np.argmax(best))
    value, indices = float(best[end]), []
    while end >= 0:
        indices.append(int(end))
        end = previous[end]
    return dict(lower_bound_m=value, witness_indices=list(reversed(indices)),
                diameter_lower_bound_m=diameter)


def measure(points, error_m=.03):
    p = np.asarray(points)
    result = ordered_length_lower_bound(p, error_m)
    diameter = float(np.max(np.linalg.norm(p[:, None]-p[None], axis=2))) if len(p)>1 else 0.
    return {**result, 'points':len(p), 'measured_diameter_m':diameter,
            'raw_polyline_m':float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),
            'point_error_assumption_m':error_m, 'wall_identity_certified':False,
            'driving_authorized':False}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('geometry',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    before=digest(args.geometry)
    source=json.loads(args.geometry.read_text())
    saved=[]
    for row in source['rows']:
        points=np.asarray(row['points'])
        for part in row['components']:
            p=points[part['indices']]
            saved.append(dict(t=row['t'],seq=row['seq'],**measure(p)))
    synthetic=[]
    for radius in (.10,.15,.20,.30,.40,.60,.80):
        ranges,labels,directions,origin,_=annular_scan(radius)
        p=origin+directions[labels]*ranges[labels,None]
        synthetic.append(dict(case='annular_inner',radius_m=radius,**measure(p)))
    # 짧은 물체도 길고 주름진 표면을 가질 수 있다. 길이만으로 벽 인증 금지.
    p=np.column_stack((np.linspace(0.,.2,41),np.where(np.arange(41)%2,.07,-.07)))
    synthetic.append(dict(case='short_corrugated_object',**measure(p)))
    sources=['scripts/probe_observed_curve_length.py','scripts/probe_corner_observability.py',
             'scripts/inspect_corner_geometry.py','tests/test_observed_curve_length.py']
    hashes={p:digest(ROOT/p) for p in sources}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sources:z.write(ROOT/p,p)
    report=dict(scope='ordered_surface_length_diagnostic_not_boundary_classifier',
                input_sha256=before,input_unchanged=before==digest(args.geometry),
                source_sha256=hashes,source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()),
                saved=saved,synthetic=synthetic,driving_authorized=False,runtime_changed=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(saved=[{k:r[k] for k in ('t','points','measured_diameter_m','lower_bound_m')} for r in saved],
                          synthetic=[{k:r.get(k) for k in ('case','radius_m','measured_diameter_m','lower_bound_m')} for r in synthetic])))


if __name__=='__main__':main()
