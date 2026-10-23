#!/usr/bin/env python3
"""고정 현재 중심의 등속 원판: 속도 원판 전체에 대한 조건부 크기 상·하계.

평가 전용. 최소 검출 물체 크기를 결정하거나 미관측 공간을 free로 바꾸지 않는다.
광선별 첫 반환 선분과 표본 자차 사각형의 거리 함수가 1-Lipschitz임을 이용한다.
현재 중심은 고정하고 속도만 탐색하므로 연속 공간의 모든 표적 탐색은 아니다.
"""
import argparse
import heapq
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

from audit_observation_geometry import collect
from audit_corridor_runtime import digest
from probe_observation_target_extent import ROOT, rectangle_distance


class DiskClearanceModel:
    """거리 제약 = 실제 반환 선분 + 취득 시각별 자차 + 현재 자차.

    반환 없는 광선에서 공간 정보를 만들지 않는다. 몸체 자세는 관측 투영을
    역산한 명목 값이다. 실물 각도/자차 운동 오차와 표적 가속도는 포함하지 않는다.
    """
    def __init__(self, scene, center, motion, cfg, metadata):
        self.center = np.asarray(center, dtype=float)
        if self.center.shape != (2,) or not np.isfinite(self.center).all() or not scene.rays:
            raise ValueError('finite 2D center and nonempty scene required')
        if len(scene.rays) != metadata['count']:
            raise ValueError('metadata/rays mismatch')
        self.cfg = cfg
        all_dt = np.array([r.acquired_at_s-scene.t_ref_s for r in scene.rays])
        origins = np.array([r.ray_origin_xy_m for r in scene.rays])
        directions = np.array([r.direction_xy for r in scene.rays])
        angles = metadata['angle_min']+np.array([r.source_index for r in scene.rays])*metadata['angle_increment']
        yaw = np.arctan2(directions[:, 1], directions[:, 0])-angles-motion.lidar_yaw_rad
        self.co, self.si = np.cos(yaw), np.sin(yaw)
        self.base = origins-np.column_stack((self.co*motion.lidar_x_m-self.si*motion.lidar_y_m,
                                            self.si*motion.lidar_x_m+self.co*motion.lidar_y_m))
        valid, depths = [], []
        for i, r in enumerate(scene.rays):
            if r.status == 'return':
                end = math.dist(r.ray_origin_xy_m, r.hit_xy_m)-cfg.measurement_padding_m
                if end > metadata['range_min']:
                    valid.append(i)
                    depths.append(end)
        self.dt = all_dt
        self.rdt = all_dt[valid]
        self.origins, self.directions = origins[valid], directions[valid]
        self.starts, self.ends = metadata['range_min'], np.array(depths)
        self.lipschitz = np.r_[abs(self.rdt), abs(all_dt), 0.]

    def distances(self, velocity):
        v = np.asarray(velocity, dtype=float)
        if v.shape != (2,) or not np.isfinite(v).all():
            raise ValueError('finite velocity required')
        centers = self.center+self.rdt[:, None]*v
        along = np.clip(np.sum((centers-self.origins)*self.directions, axis=1), self.starts, self.ends)
        rays = np.linalg.norm(centers-self.origins-along[:, None]*self.directions, axis=1)
        delta = self.center+self.dt[:, None]*v-self.base
        local = np.column_stack((self.co*delta[:, 0]+self.si*delta[:, 1],
                                 -self.si*delta[:, 0]+self.co*delta[:, 1]))
        ego = rectangle_distance(local, self.cfg.length_m, self.cfg.width_m)
        now = rectangle_distance(self.center, self.cfg.length_m, self.cfg.width_m)
        return np.r_[rays, ego, now]


def velocity_bounds(model, speed_bound, tolerance_m=1e-4, max_cells=4096, seeds=()):
    """속도 원판을 덮는 사각 셀 분할. 최대 min-distance의 상계와 실측 하계.

    셀 중심 밖까지의 최대 속도차 sqrt(2)*half에 |광선 나이|를 곱한다.
    셀마다 min_i(d_i(center)+L_i*radius)가 전체 셀의 min-distance 상계다.
    속도 원판과 닿는 셀만 유지한다. 예산 소진 시 남은 상계를 반드시 반환한다.
    """
    if (not math.isfinite(speed_bound) or speed_bound < 0 or
            not math.isfinite(tolerance_m) or tolerance_m <= 0 or
            isinstance(max_cells, bool) or not isinstance(max_cells, int) or max_cells < 1):
        raise ValueError('invalid search limits')
    best, best_v, visits, serial = -1., None, 0, 0
    heap = []
    for v in ((0., 0.), *seeds):
        v = np.asarray(v, dtype=float)
        if v.shape != (2,) or not np.isfinite(v).all() or np.linalg.norm(v) > speed_bound:
            continue
        score = float(model.distances(v).min())
        if score > best:
            best, best_v = score, v.tolist()

    def add(center, half):
        nonlocal best, best_v, visits, serial
        if np.linalg.norm(np.maximum(np.abs(center)-half, 0.)) > speed_bound:
            return
        distances = model.distances(center)
        visits += 1
        if np.linalg.norm(center) <= speed_bound and float(distances.min()) > best:
            best, best_v = float(distances.min()), center.tolist()
        upper = float(np.min(distances+model.lipschitz*(math.sqrt(2)*half)))+1e-12
        if upper > best:
            serial += 1
            heapq.heappush(heap, (-upper, serial, center, half))

    add(np.zeros(2), speed_bound)
    while heap and -heap[0][0]-best > tolerance_m and visits+4 <= max_cells:
        _, _, center, half = heapq.heappop(heap)
        half /= 2
        for x, y in ((-1,-1),(-1,1),(1,-1),(1,1)):
            add(center+half*np.array([x,y]), half)
    upper = max(best, -heap[0][0] if heap else best)
    return dict(radius_lower_m=best, radius_upper_m=upper, gap_m=upper-best,
                velocity_xy_mps=best_v, evaluated_cells=visits,
                tolerance_met=upper-best <= tolerance_m, tolerance_m=tolerance_m,
                max_cells=max_cells, search_domain='constant_velocity_disk_fixed_current_center')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    capture = ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'
    evidence = ROOT/'artifacts/validation/2026-10-21/g1_target_extent_v1/run/report.json'
    inputs = {str(p.relative_to(ROOT)):digest(p) for p in (capture,evidence)}
    sources = {}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            p = Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                sources[p.relative_to(ROOT).as_posix()] = digest(p)
    sources['tests/test_target_velocity_bounds.py'] = digest(ROOT/'tests/test_target_velocity_bounds.py')
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    cfg, motion, found, checked, mismatches = collect(capture,{3156})
    if mismatches:
        raise ValueError('recorded decision mismatch')
    original = json.loads(evidence.read_text())
    rows = []
    for r in original['rows']:
        scene = next(s for s in found[3156]['sources'] if s.source_first_s == r['source_first_s'])
        model = DiskClearanceModel(scene, r['query_xy_m'], motion, cfg,
                                    original['raw_scan_metadata'][str(r['source_first_s'])])
        bounds = velocity_bounds(model, cfg.object_motion_bound_mps, seeds=(r['velocity_xy_mps'],))
        rows.append(dict(query_xy_m=r['query_xy_m'], source_first_s=r['source_first_s'], **bounds))
    summary = []
    for diameter in (.001,.002,.005,.01,.02,.05,.10,.15):
        summary.append(dict(diameter_m=diameter,
            refuted_all_constant_velocities=sum(r['radius_upper_m'] < diameter/2-1e-9 for r in rows),
            found_unrefuted_velocity=sum(r['radius_lower_m'] >= diameter/2 for r in rows),
            unresolved=sum(r['radius_lower_m'] < diameter/2 and r['radius_upper_m'] >= diameter/2-1e-9 for r in rows)))
    report = dict(scope='conditional_fixed_current_center_constant_velocity_radius_bounds',
        replayed=checked,replay_mismatches=mismatches,rows=rows,summary=summary,
        all_tolerances_met=all(r['tolerance_met'] for r in rows),
        maximum_diameter_upper_m=max(2*r['radius_upper_m'] for r in rows),
        maximum_gap_m=max(r['gap_m'] for r in rows),
        cells_total=sum(r['evaluated_cells'] for r in rows),
        runtime_modified=False,driving_authorized=False,all_target_shapes_or_centers_searched=False,
        continuous_ego_nonintersection_verified=False,
        assumptions=original['model_assumptions'],
        input_sha256=inputs,input_unchanged=all(digest(ROOT/n)==h for n,h in inputs.items()),
        source_sha256=sources,source_unchanged=all(digest(ROOT/n)==h for n,h in sources.items()))
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256','input_sha256')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
