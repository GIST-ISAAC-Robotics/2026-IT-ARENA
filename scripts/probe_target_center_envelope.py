#!/usr/bin/env python3
"""가장자리 접촉·시간별 속도 변화를 허용하는 유한 원판의 평가 전용 반박.

현재 질의점을 포함하는 모든 원판 중심을 검사한다. 과거 중심은 현재 중심에서
speed_bound*age 이내일 수 있다고 넓혀 허용한다. 어떤 단일 실제 반환 선분과도
피할 수 없는 교차가 생기거나 자차와 반드시 겹치면 해당 현재 중심을 반박한다.
반박하지 못해도 시간적으로 연결된 회피 궤적의 존재를 입증한 것은 아니다.
"""
import argparse
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

from audit_corridor_runtime import digest
from audit_observation_geometry import collect
from probe_target_velocity_bounds import DiskClearanceModel, velocity_bounds
from probe_observation_target_extent import ROOT, rectangle_distance
from probe_observation_target_extent import finite_return_clearance,ego_clearance


class CenterEnvelope:
    def __init__(self, models, speed_bound):
        if not models or not np.isfinite(speed_bound) or speed_bound < 0:
            raise ValueError('nonempty models and nonnegative finite speed required')
        self.models, self.speed_bound = models, speed_bound
        self.lipschitz = np.ones(sum(len(m.lipschitz) for m in models))

    def distances(self, offset):
        delta = np.asarray(offset,dtype=float)
        if delta.shape != (2,) or not np.isfinite(delta).all():
            raise ValueError('finite 2D center offset required')
        parts = []
        for m in self.models:
            center = m.center+delta
            relative = center-m.origins
            along = np.clip(np.sum(relative*m.directions,axis=1),m.starts,m.ends)
            rays = np.linalg.norm(relative-along[:,None]*m.directions,axis=1)+self.speed_bound*abs(m.rdt)
            relative = center-m.base
            local = np.column_stack((m.co*relative[:,0]+m.si*relative[:,1],
                                     -m.si*relative[:,0]+m.co*relative[:,1]))
            ego = rectangle_distance(local,m.cfg.length_m,m.cfg.width_m)+self.speed_bound*abs(m.dt)
            now = rectangle_distance(center,m.cfg.length_m,m.cfg.width_m)
            parts.append(np.r_[rays,ego,now])
        return np.concatenate(parts)


def center_bounds(model, radius, max_cells=2048):
    """같은 닫힌 원판 탐색. 입력 변수는 속도가 아니라 현재 중심의 변위[m]."""
    raw = velocity_bounds(model,radius,tolerance_m=1e-4,max_cells=max_cells)
    return dict(center_offset_xy_m=raw['velocity_xy_mps'],
        score_lower_m=raw['radius_lower_m'],score_upper_m=raw['radius_upper_m'],gap_m=raw['gap_m'],
        evaluated_cells=raw['evaluated_cells'],tolerance_met=raw['tolerance_met'],
        max_cells=raw['max_cells'],
        all_centers_refuted=raw['radius_upper_m'] < radius-1e-9,
        unresolved=raw['radius_upper_m'] >= radius-1e-9,
        unrefuted_center_sample_found=raw['radius_lower_m'] >= radius,
        search_domain='current_centers_covering_query_variable_velocity_bound')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--sources',choices=('witness','all'),default='witness')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    capture=ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'
    evidence=ROOT/'artifacts/validation/2026-10-21/g1_target_extent_v1/run/report.json'
    inputs={str(p.relative_to(ROOT)):digest(p) for p in (capture,evidence)}
    sources={}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':
                sources[p.relative_to(ROOT).as_posix()]=digest(p)
    sources['tests/test_target_center_envelope.py']=digest(ROOT/'tests/test_target_center_envelope.py')
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name,name)
    cfg,motion,found,checked,mismatches=collect(capture,{3156})
    if mismatches:
        raise ValueError('original decision mismatch')
    original=json.loads(evidence.read_text())
    rows=[]
    # 단일 원래 반례 출처와 현재/최근 출처 결합을 명시적으로 구분한다.
    for r in original['rows']:
        selected=found[3156]['sources'] if args.sources=='all' else [
            next(s for s in found[3156]['sources'] if s.source_first_s==r['source_first_s'])]
        models=[DiskClearanceModel(s,r['query_xy_m'],motion,cfg,
                                  original['raw_scan_metadata'][str(s.source_first_s)]) for s in selected]
        envelope=CenterEnvelope(models,cfg.object_motion_bound_mps)
        cases=[dict(diameter_m=d,**center_bounds(envelope,d/2)) for d in (.005,.01,.02,.05,.1,.15)]
        rows.append(dict(query_xy_m=r['query_xy_m'],source_first_s=r['source_first_s'],cases=cases))
    summary=[]
    for diameter in (.005,.01,.02,.05,.1,.15):
        cases=[c for r in rows for c in r['cases'] if c['diameter_m']==diameter]
        summary.append(dict(diameter_m=diameter,all_centers_refuted=sum(c['all_centers_refuted'] for c in cases),
            unresolved=sum(c['unresolved'] for c in cases),
            unrefuted_center_sample_found=sum(c['unrefuted_center_sample_found'] for c in cases),
            tolerance_met=sum(c['tolerance_met'] for c in cases),
            maximum_gap_m=max(c['gap_m'] for c in cases)))
    # 미결정 조건의 대표 중심에서 등속 궤적을 찾는다. 실패해도 부재 증명은 아니다.
    trajectories=[]
    if args.sources=='all':
        for stamp in sorted({r['source_first_s'] for r in rows}):
            for diameter in (.1,.15):
                candidates=[(r,c) for r in rows for c in r['cases']
                            if r['source_first_s']==stamp and c['diameter_m']==diameter and c['unresolved']]
                if not candidates:
                    continue
                r,case=max(candidates,key=lambda p:p[1]['score_lower_m'])
                center=np.array(r['query_xy_m'])+case['center_offset_xy_m']
                models=[DiskClearanceModel(s,center,motion,cfg,
                         original['raw_scan_metadata'][str(s.source_first_s)]) for s in found[3156]['sources']]
                class Combined:
                    lipschitz=np.concatenate([m.lipschitz for m in models])
                    def distances(self,v):
                        return np.concatenate([m.distances(v) for m in models])
                bounds=velocity_bounds(Combined(),cfg.object_motion_bound_mps)
                velocity=bounds['velocity_xy_mps']
                independent=[]
                for s in found[3156]['sources']:
                    metadata=original['raw_scan_metadata'][str(s.source_first_s)]
                    finite=finite_return_clearance(s,center,velocity,range_min=metadata['range_min'],
                                                    range_error=cfg.measurement_padding_m)
                    independent.extend(e['clearance_m'] for e in finite)
                    independent.append(ego_clearance(s,center,velocity,motion,cfg,metadata))
                check=min(independent)
                if abs(check-bounds['radius_lower_m'])>1e-10:
                    raise ValueError('independent trajectory audit disagrees')
                trajectories.append(dict(query_xy_m=r['query_xy_m'],diameter_m=diameter,
                    center_xy_m=center.tolist(),witness_source_first_s=stamp,**bounds,
                    independent_clearance_m=check,
                    covers_query=bool(np.linalg.norm(center-r['query_xy_m'])<=diameter/2),
                    finite_returns_and_sampled_ego_not_refuted=check>=diameter/2))
    report=dict(scope='conditional_target_center_and_speed_envelope_not_runtime',
        replayed=checked,replay_mismatches=mismatches,rows=rows,summary=summary,
        runtime_modified=False,driving_authorized=False,
        source_mode=args.sources,trajectory_samples=trajectories,
        any_unrefuted_sampled_trajectory_constructed=any(r['finite_returns_and_sampled_ego_not_refuted']
                                                         for r in trajectories),all_target_shapes_searched=False,
        continuous_ego_nonintersection_verified=False,
        assumptions=['opaque_planar_disk','bounded_speed_not_constant_velocity',
                     'first_return_prefix_valid','nominal_ego_pose','no_angle_or_pose_error',
                     args.sources+'_source_selection','all_current_centers_covering_query'],
        input_sha256=inputs,input_unchanged=all(digest(ROOT/n)==h for n,h in inputs.items()),
        source_sha256=sources,source_unchanged=all(digest(ROOT/n)==h for n,h in sources.items()))
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256','input_sha256')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__=='__main__':
    raise SystemExit(main())
