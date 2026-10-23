#!/usr/bin/env python3
"""회랑 shadow의 저장 노면/정적 첫 반환 반례. 정답은 사후 평가에만 사용한다."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from shapely.geometry import Point

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from audit_corner_path_road import bracketing_poses, exact_pose, path_footprints
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons, rectangle
from probe_corner_corridor import shadow_scene
from probe_corner_observability import annular_scan, circle_first_hit
from profile_follow_trace import SNAPSHOT_MODULES

ROOT = Path(__file__).resolve().parents[1]


def rectangle_first_hit(origin, directions, center, half_extents):
    """축 정렬 사각형 첫 양수 광선 교점. 진입/접선/축 평행을 명시 처리한다."""
    origin, directions = np.asarray(origin, dtype=float), np.asarray(directions, dtype=float)
    center, half = np.asarray(center, dtype=float), np.asarray(half_extents, dtype=float)
    if (origin.shape != (2,) or center.shape != (2,) or half.shape != (2,) or
            directions.ndim != 2 or directions.shape[1] != 2 or np.any(half <= 0) or
            not all(np.all(np.isfinite(x)) for x in (origin, directions, center, half)) or
            np.any(np.abs(np.linalg.norm(directions, axis=1)-1.) > 1e-10)):
        raise ValueError('finite unit rays and positive rectangle required')
    near, far = np.full(len(directions), -np.inf), np.full(len(directions), np.inf)
    valid = np.ones(len(directions), dtype=bool)
    for axis in (0,1):
        parallel = np.abs(directions[:,axis]) < 1e-14
        valid &= ~parallel | (abs(origin[axis]-center[axis]) <= half[axis])
        a, b = np.full(len(directions), -np.inf), np.full(len(directions), np.inf)
        np.divide(center[axis]-half[axis]-origin[axis], directions[:,axis], out=a, where=~parallel)
        np.divide(center[axis]+half[axis]-origin[axis], directions[:,axis], out=b, where=~parallel)
        near, far = np.maximum(near, np.minimum(a,b)), np.minimum(far, np.maximum(a,b))
    valid &= (far >= near) & (far >= 0)
    return np.where(valid, np.where(near >= 0, near, far), np.inf)


def static_case(radius, side, kind, noise_name):
    sign = 1 if side == 'left' else -1
    cfg = FollowConfig(occupancy_timing_mode='per_ray_candidate')
    clean, _, directions, origin, center = annular_scan(radius, sign)
    ranges = clean.copy()
    obstacles = []
    if kind in ('missing_inner', 'short_box_inner'):
        ranges = circle_first_hit(origin, directions, center, radius+.85)
    if kind in ('center_block', 'side_block', 'short_box_inner'):
        xy = {'center_block':(.26,0.), 'side_block':(.30,sign*.18),
              'short_box_inner':(0.,sign*.50)}[kind]
        ranges = np.minimum(ranges, rectangle_first_hit(origin, directions, xy, (.10,.075)))
        obstacles.append(rectangle(*xy, 0., .20, .15))
    noise = {'none':np.zeros(500), 'bias_positive':np.full(500,.03),
             'bias_negative':np.full(500,-.03),
             'uniform_seed0':np.random.default_rng(0).uniform(-.03,.03,500)}[noise_name]
    ranges = ranges+noise
    if kind == 'front_unknown':
        ranges[250] = np.inf
    scene = scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=math.tau/500,
                            range_min=.05, range_max=12., acquired_at_s=(.3+np.arange(500)*.0002).tolist(),
                            t_ref_s=.4, rear_half_angle_deg=30.)
    # 정지 차량이 코너 방향으로 미리 조향한 합성 입력. 취득 중 이동/폐루프가 아니다.
    previous_steering = sign*math.atan(cfg.wheelbase_m/(radius+cfg.wall_offset_m))
    result = shadow_scene(scene, 0., side, previous_steering, .02, cfg)
    # 0.85 m 벽 사이에 0.45 m 도로, 좌우 0.20 m 여백을 둔 해석용 원형 장면.
    # 공식 트랙 반경/실제 노면 측정치로 해석하지 않는다.
    road = Point(center).buffer(radius+.65, quad_segs=256).difference(
        Point(center).buffer(radius+.20, quad_segs=256))
    audit = None
    if result['geometry_supported']:
        audit = path_footprints(result['geometry']['path'], (0.,0.,0.), road, obstacles)
    return dict(radius_m=radius, side=side, kind=kind, noise=noise_name,
                previous_steering_rad=previous_steering, shadow=result, footprint_audit=audit,
                actual_motion_during_scan=False, official_track_geometry=False, driving_authorized=False)


def saved_path_audit(report, poses, road, obstacles):
    rows = []
    for row in report['rows']:
        result = row['shadow']
        if not result['geometry_supported']:
            continue
        item = dict(seq=row['seq'], time_s=row['time_s'],
                    preliminary_motion_supported=result['preliminary_motion_supported'])
        path = result['geometry']['path']
        try:
            pose = exact_pose(poses, row['time_s'])
            item.update(evaluated=True, audit=path_footprints(path, pose, road, obstacles))
        except ValueError as error:
            item.update(evaluated=False, reason=str(error))
            try:
                bracket = bracketing_poses(poses, row['time_s'])
                item.update(sensitivity_evaluated=True, contemporaneous_pose_verified=False,
                            bracket_evaluations=[dict(time_s=r[0], offset_s=r[0]-row['time_s'],
                                audit=path_footprints(path, r[1:], road, obstacles)) for r in bracket])
            except ValueError as bracket_error:
                item.update(sensitivity_evaluated=False, sensitivity_reason=str(bracket_error))
        rows.append(item)
    return rows


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('shadow_report', type=Path)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = [args.shadow_report, args.run/'runtime_world.sdf', args.run/'ego_poses.jsonl']
    input_hashes = {str(p):digest(p) for p in inputs}
    files = [f'src/{pkg}/{pkg}/{mod}.py' for pkg,mods in SNAPSHOT_MODULES.items() for mod in mods]
    files += ['scripts/audit_corridor_shadow.py', 'scripts/probe_corner_corridor.py',
              'scripts/audit_corner_path_road.py','scripts/audit_local_pursuit_road.py',
              'scripts/build_experimental_track.py','scripts/probe_short_corner_support.py',
              'scripts/probe_corner_observability.py','scripts/inspect_corner_geometry.py',
              'scripts/profile_follow_trace.py','tests/test_corridor_shadow_audit.py']
    sources = {p:digest(ROOT/p) for p in files}
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in files:
            archive.write(ROOT/p,p)
    original = json.loads(inputs[0].read_text())
    with inputs[2].open() as stream:
        poses = [json.loads(line) for line in stream]
    saved = saved_path_audit(original, poses, road_polygon(inputs[1]), obstacle_polygons(inputs[1]))
    fixtures = [static_case(radius, side, kind, noise) for radius in (.15,.30)
                for side in ('left','right')
                for kind in ('open','center_block','side_block','missing_inner','short_box_inner','front_unknown')
                for noise in ('none','bias_positive','bias_negative','uniform_seed0')]
    sensitivity = [a['audit'] for r in saved if r.get('sensitivity_evaluated') for a in r['bracket_evaluations']]
    motion_sensitivity = [a['audit'] for r in saved if r.get('sensitivity_evaluated') and r['preliminary_motion_supported']
                          for a in r['bracket_evaluations']]
    motion_fixtures = [r for r in fixtures if r['shadow']['preliminary_motion_supported']]
    summary = dict(saved_geometry_scenes=len(saved), saved_exact_scenes=sum(r['evaluated'] for r in saved),
        saved_sensitivity_poses=len(sensitivity),
        saved_sensitivity_paths_outside=sum(a['outside_over_1cm2']>0 for a in sensitivity) if sensitivity else None,
        saved_sensitivity_motion_paths=len(motion_sensitivity),
        saved_sensitivity_motion_paths_outside=sum(a['outside_over_1cm2']>0 for a in motion_sensitivity) if motion_sensitivity else None,
        static_fixtures=len(fixtures), static_preliminary_motion=len(motion_fixtures),
        static_motion_paths_outside=sum(r['footprint_audit']['outside_over_1cm2']>0 for r in motion_fixtures) if motion_fixtures else None,
        static_motion_paths_touching_box=sum(r['footprint_audit']['wall_overlap_over_1mm2']>0 for r in motion_fixtures) if motion_fixtures else None)
    result = dict(scope='posthoc_road_truth_and_static_first_return_counterexamples', summary=summary,
        saved=saved, fixtures=fixtures, input_sha256=input_hashes, source_sha256=sources,
        input_unchanged=all(digest(p)==v for p,v in input_hashes.items()),
        source_unchanged=all(digest(ROOT/p)==v for p,v in sources.items()),
        runtime_changed=False, ground_truth_to_controller=False, driving_authorized=False,
        continuous_footprint_verified=False, future_motion_verified=False)
    with (args.output/'report.json').open('x') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps({**summary, 'input_unchanged':result['input_unchanged'], 'source_unchanged':result['source_unchanged']}))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 2


if __name__=='__main__':
    raise SystemExit(main())
