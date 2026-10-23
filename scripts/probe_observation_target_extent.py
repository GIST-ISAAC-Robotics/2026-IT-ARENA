#!/usr/bin/env python3
"""점형 측방 witness를 유한 원판으로 키웠을 때 원래 반환과 모순되는지 진단.

운영 미연결. 모든 가능한 중심/속도/형상 탐색이나 물체 부재 증명이 아니다.
같은 원판이 스캔 평면을 가리고 첫 반사를 지배한다는 조건부 2D 광학 모형이다.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

from audit_corridor_runtime import digest
from audit_observation_geometry import collect, events

ROOT = Path(__file__).resolve().parents[1]


def rectangle_distance(points, length=.20, width=.15):
    p = np.asarray(points, dtype=float)
    excess = np.maximum(np.abs(p)-[length/2, width/2], 0.)
    return np.linalg.norm(excess, axis=-1)


def timed_centers(scene, center_now, velocity):
    return np.asarray(center_now)+np.array([r.acquired_at_s-scene.t_ref_s for r in scene.rays])[:, None]*velocity


def finite_return_clearance(scene, center_now, velocity, range_min=.05, range_error=.03):
    """각 실제 반환 전의 유한 선분과 그 시각 원판 중심 사이 최소 거리.

    반환 끝의 range_error를 제외한다. beam 폭/각도 오차/다중 반사는 모형에 없다.
    unknown/masked 광선에서는 자유 공간이나 반사 증거를 만들지 않는다.
    """
    c, v = np.asarray(center_now, dtype=float), np.asarray(velocity, dtype=float)
    if c.shape != (2,) or v.shape != (2,) or not np.isfinite([*c, *v, range_min, range_error]).all():
        raise ValueError('finite 2D center/velocity and range parameters required')
    if range_min < 0 or range_error < 0:
        raise ValueError('nonnegative range limits required')
    centers = timed_centers(scene, c, v)
    rows = []
    for i, ray in enumerate(scene.rays):
        if ray.status != 'return':
            continue
        end = math.dist(ray.ray_origin_xy_m, ray.hit_xy_m)-range_error
        if end <= range_min:
            continue
        origin, direction = np.array(ray.ray_origin_xy_m), np.array(ray.direction_xy)
        along = float(np.clip(np.dot(centers[i]-origin, direction), range_min, end))
        distance = float(np.linalg.norm(centers[i]-(origin+along*direction)))
        rows.append(dict(index=ray.source_index, clearance_m=distance, acquired_at_s=ray.acquired_at_s,
                         prefix_end_m=end, nearest_prefix_distance_m=along))
    return rows


def ego_clearance(scene, center_now, velocity, motion, cfg, metadata):
    """각 광선의 own-motion 투영을 역산한 자차 사각형과 원판 중심 거리 표본.

    원래 스캔의 각 메타데이터를 사용하며 실측 몸체/연속 충돌 검사가 아니다.
    """
    if len(scene.rays) != metadata['count']:
        raise ValueError('source metadata/ray count mismatch')
    angles = metadata['angle_min']+np.array([r.source_index for r in scene.rays])*metadata['angle_increment']
    yaw = np.array([math.atan2(r.direction_xy[1], r.direction_xy[0]) for r in scene.rays])-angles-motion.lidar_yaw_rad
    co, si = np.cos(yaw), np.sin(yaw)
    origins = np.array([r.ray_origin_xy_m for r in scene.rays])
    base = origins-np.column_stack((co*motion.lidar_x_m-si*motion.lidar_y_m,
                                    si*motion.lidar_x_m+co*motion.lidar_y_m))
    delta = timed_centers(scene, center_now, velocity)-base
    local = np.column_stack((co*delta[:, 0]+si*delta[:, 1], -si*delta[:, 0]+co*delta[:, 1]))
    return min(float(rectangle_distance(local, cfg.length_m, cfg.width_m).min()),
               float(rectangle_distance(center_now, cfg.length_m, cfg.width_m)))


def analyse(source, row, motion, cfg, metadata):
    q, start = np.array(row['query_xy_m']), np.array(row['possible_start_xy_m'])
    duration = source.t_ref_s-row['ray_acquired_at_s']
    if duration <= 0:
        raise ValueError('positive witness time required')
    velocity = (q-start)/duration
    if np.linalg.norm(velocity) > cfg.object_motion_bound_mps+1e-9:
        raise ValueError('witness exceeds original motion bound')
    evidence = finite_return_clearance(source, q, velocity, range_min=metadata['range_min'],
                                       range_error=cfg.measurement_padding_m)
    ego = ego_clearance(source, q, velocity, motion, cfg, metadata)
    nearest = min(evidence, key=lambda r:r['clearance_m']) if evidence else None
    cases = []
    for diameter in (.001, .002, .005, .01, .02, .05, .10, .15):
        radius = diameter/2
        conflicts = [r['index'] for r in evidence if r['clearance_m'] < radius-1e-9]
        overlaps = ego < radius-1e-9
        cases.append(dict(diameter_m=diameter, conflicting_return_count=len(conflicts),
                          conflicting_return_indices=conflicts,
                          sampled_ego_overlap=overlaps,
                          this_trajectory_not_refuted=not conflicts and not overlaps))
    return dict(query_xy_m=q.tolist(), possible_start_xy_m=start.tolist(),
                source_first_s=source.source_first_s, witness_ray_index=row['ray_index'],
                duration_s=duration, velocity_xy_mps=velocity.tolist(),
                velocity_mps=float(np.linalg.norm(velocity)), nearest_finite_prefix=nearest,
                sampled_ego_clearance_m=ego, cases=cases)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    capture = ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'
    witnesses = ROOT/'artifacts/validation/2026-10-19/g1_observation_geometry_v2_retry/run/witnesses.json'
    inputs = {str(p.relative_to(ROOT)): digest(p) for p in (capture, witnesses)}
    sources = {}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            p = Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                sources[p.relative_to(ROOT).as_posix()] = digest(p)
    sources['tests/test_observation_target_extent.py'] = digest(ROOT/'tests/test_observation_target_extent.py')
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    cfg, motion, found, checked, mismatches = collect(capture, {3156})
    if mismatches:
        raise ValueError('original decision replay mismatch')
    rows = json.loads(witnesses.read_text())['3156']['recorded']
    if len(rows) != 225 or any(r['cause'] != 'lateral_reach' or not r['lateral_witness_found'] for r in rows):
        raise ValueError('expected fixed 225 lateral witnesses')
    scenes = found[3156]['sources']
    metadata = {}
    for event in events(capture):
        if event['seq'] > 3156:
            break
        if event['kind'] != 'call':
            continue
        scan = event['data']['input']['scan']
        stamp = float(scan['sec'])+float(scan['nanosec'])*1e-9
        if stamp not in {s.source_first_s for s in scenes}:
            continue
        entry = {k:scan[k] for k in ('angle_min','angle_increment','range_min')}
        entry['count'] = len(scan['ranges'])
        if stamp in metadata and metadata[stamp] != entry:
            raise ValueError('same scan metadata changed')
        metadata[stamp] = entry
    results = [analyse(next(s for s in scenes if s.source_first_s == r['source_first_s']), r, motion, cfg,
                       metadata[r['source_first_s']]) for r in rows]
    summary = []
    for diameter in (.001, .002, .005, .01, .02, .05, .10, .15):
        cases = [c for r in results for c in r['cases'] if c['diameter_m'] == diameter]
        summary.append(dict(diameter_m=diameter, witnesses=len(cases),
            finite_return_conflict=sum(c['conflicting_return_count'] > 0 for c in cases),
            ego_overlap=sum(c['sampled_ego_overlap'] for c in cases),
            not_refuted=sum(c['this_trajectory_not_refuted'] for c in cases)))
    report = dict(scope='fixed_point_witness_lifted_to_constant_velocity_disk_not_runtime',
        replayed=checked, replay_mismatches=mismatches, rows=results, summary=summary,
        model_assumptions=['opaque_planar_circular_cross_section', 'observed_first_return_prefix_valid',
                          'constant_velocity_per_witness', 'ego_pose_from_existing_motion',
                          'no_beam_angle_or_pose_error', 'one_recorded_source_per_witness'],
        all_possible_targets_searched=False, unknown_filled=False, runtime_modified=False,
        continuous_ego_nonintersection_verified=False, driving_authorized=False,
        raw_scan_metadata=metadata,
        input_sha256=inputs, input_unchanged=all(digest(ROOT/n) == h for n,h in inputs.items()),
        source_sha256=sources, source_unchanged=all(digest(ROOT/n) == h for n,h in sources.items()))
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256','input_sha256')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
