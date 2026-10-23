#!/usr/bin/env python3
"""저장된 전방 무응답과 정적 벽의 광선 교차를 대조하는 평가 전용 도구.

원시 무응답을 채우거나 제어기/보호 판정을 바꾸지 않는다. GPU 재실행이 아니라
수평 측정면의 수학적 교차이며 재질/렌더러/시간 동작 재현과 구별한다.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest
from g0_geometry import Footprint, TruthBox, ray_box_distance
from summarize_corner_trace import summarize

ROOT = Path(__file__).resolve().parents[1]


def pose(element):
    node = element.find('pose')
    if node is not None and node.get('relative_to'):
        raise ValueError('relative pose unsupported in this diagnostic')
    values = list(map(float, element.findtext('pose', '0 0 0 0 0 0').split()))
    if len(values) != 6 or not all(math.isfinite(v) for v in values) or any(abs(v) > 1e-12 for v in values[3:5]):
        raise ValueError('finite planar static pose required')
    return values


def compose(parent, child):
    c, s = math.cos(parent[5]), math.sin(parent[5])
    return [parent[0]+c*child[0]-s*child[1], parent[1]+s*child[0]+c*child[1],
            parent[2]+child[2], 0., 0., parent[5]+child[5]]


def obj_triangles(path, transform):
    vertices, faces = [], []
    with Path(path).open() as stream:
        for line in stream:
            parts = line.split()
            if parts and parts[0] == 'v':
                vertices.append(list(map(float, parts[1:4])))
            elif parts and parts[0] == 'f':
                if len(parts) != 4:
                    raise ValueError('explicit triangles required')
                faces.append([int(p.split('/')[0])-1 for p in parts[1:]])
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces, dtype=int)
    if vertices.ndim != 2 or vertices.shape[1] != 3 or faces.ndim != 2 or faces.shape[1] != 3 or np.min(faces) < 0 or np.max(faces) >= len(vertices):
        raise ValueError('invalid mesh')
    c, s = math.cos(transform[5]), math.sin(transform[5])
    rotation = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])
    return (vertices@rotation+transform[:3])[faces]


def wall_geometry(world_path):
    model = ET.parse(world_path).find("./world/model[@name='it_arena_track_static']")
    if model is None:
        raise ValueError('known static model required')
    walls = model.find("link[@name='walls']")
    if walls is None:
        raise ValueError('known walls link required')
    transform = compose(pose(model), pose(walls))
    boxes, triangles, meshes = [], [], []
    for node in walls.findall('collision'):
        size = node.findtext('geometry/box/size')
        if size is None:
            raise ValueError('nonbox wall collision unsupported')
        sx, sy, sz = map(float, size.split())
        xyz = compose(transform, pose(node))
        boxes.append(TruthBox(xyz[0], xyz[1], sz, Footprint(sx, sy), xyz[5], xyz[2]-sz/2, node.get('name')))
    for node in walls.findall('visual'):
        uri = node.findtext('geometry/mesh/uri')
        if uri is None or node.find('geometry/mesh/scale') is not None:
            raise ValueError('known unscaled batched visual required')
        # 보존본과 같은 디렉터리의 메시를 사용하며 시스템 URI를 임의로 따라가지 않는다.
        path = Path(world_path).parent/Path(uri).name
        meshes.append(path)
        triangles.append(obj_triangles(path, compose(transform, pose(node))))
    if not boxes or not triangles:
        raise ValueError('nonempty collision and visual geometry required')
    return boxes, np.concatenate(triangles), meshes


def triangle_distance(origin, direction, triangles, front_only=False, maximum=100.):
    origin, direction = np.asarray(origin), np.asarray(direction)
    direction = direction/np.linalg.norm(direction)
    edge1, edge2 = triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]
    cross = np.cross(direction, edge2)
    determinant = np.einsum('ij,ij->i', edge1, cross)
    valid = determinant > 1e-12 if front_only else np.abs(determinant) > 1e-12
    inverse = np.divide(1., determinant, out=np.zeros_like(determinant), where=valid)
    delta = origin-triangles[:, 0]
    u = np.einsum('ij,ij->i', delta, cross)*inverse
    q = np.cross(delta, edge1)
    v = q@direction*inverse
    t = np.einsum('ij,ij->i', edge2, q)*inverse
    valid &= (u >= -1e-10) & (v >= -1e-10) & (u+v <= 1+1e-10) & (t >= 0) & (t <= maximum)
    return float(np.min(t[valid])) if np.any(valid) else None


def raw_kind(value, minimum, maximum):
    if not math.isfinite(value):
        return 'nan' if math.isnan(value) else ('positive_infinity' if value > 0 else 'negative_infinity')
    return 'below_minimum' if value < minimum else ('above_maximum' if value > maximum else 'finite_return')


def pose_window(poses, when, capture_error=.0021):
    """취득 근사 구간을 둘러싼 실제 기록 두 개. 보간/중간 안전 상한이 아니다."""
    times = poses[:, 0]
    lo = int(np.searchsorted(times, when-capture_error, side='right')-1)
    hi = int(np.searchsorted(times, when, side='left'))
    if lo < 0 or hi >= len(times) or times[hi]-times[lo] > .006:
        raise ValueError('diagnostic pose window unavailable')
    return poses[[lo, hi]]


def front_invalid(scene, scan):
    return [dict(index=r.source_index, acquired_at_s=r.acquired_at_s, status=r.status,
                 raw_kind=raw_kind(scan.ranges[r.source_index], scan.range_min, scan.range_max),
                 raw_angle_deg=math.degrees(scan.angle_min+r.source_index*scan.angle_increment),
                 current_angle_deg=math.degrees(math.atan2(r.direction_xy[1], r.direction_xy[0])))
            for r in scene.rays if abs(math.atan2(r.direction_xy[1], r.direction_xy[0])) <= math.radians(100)
            and r.status != 'return']


def geometry_summary(examples):
    missing = [c for row in examples for c in row['casts'] if c['raw_kind'] == 'positive_infinity']
    return dict(expected_missing_ray_pose_samples=2*sum(len(row['invalid']) for row in examples),
        missing_ray_pose_samples=len(missing), unavailable_ray_windows=sum(len(row.get('unavailable', [])) for row in examples),
        beyond_configured_range=sum(c['collision_distance_m'] is not None and
            c['collision_distance_m'] > row['range_max_m'] for row in examples for c in row['casts']
            if c['raw_kind'] == 'positive_infinity'),
        box_visual_agree=sum(all(c[k] is not None for k in ('collision_distance_m', 'visual_two_sided_m', 'visual_front_facing_m'))
            and max(abs(c['collision_distance_m']-c[k]) for k in ('visual_two_sided_m', 'visual_front_facing_m')) <= 1e-6
            for c in missing),
        minimum_wall_distance_m=min((c['collision_distance_m'] for c in missing if c['collision_distance_m'] is not None), default=None),
        maximum_wall_distance_m=max((c['collision_distance_m'] for c in missing if c['collision_distance_m'] is not None), default=None),
        sampling_scope='recorded acquisition-window endpoints; not continuous-time proof')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--all-unknown-scans', action='store_true',
                        help='모든 고유 무응답 스캔(최대 128개)의 최초 호출을 비교한다')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    capture = args.run/'follow_decisions.jsonl'
    inputs = [capture, args.run/'runtime_world.sdf', args.run/'runtime_vehicle.sdf',
              args.run/'ego_poses.jsonl', args.run/'report.json', args.run/'trajectory.jsonl']
    before = {str(p): digest(p) for p in inputs}
    sources = {}
    for module in tuple(sys.modules.values()):
        location = getattr(module, '__file__', None)
        if location:
            path = Path(location).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                sources[str(path.relative_to(ROOT))] = digest(path)
    test = ROOT/'tests/test_front_unknown_diagnostic.py'
    sources[str(test.relative_to(ROOT))] = digest(test)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    with capture.open() as stream:
        capture_check = summarize(json.loads(line) for line in stream)
    # 전체 원시 입력을 메모리에 쌓지 않고 대상 호출만 보존한다.
    first_bad = last_bad = previous = last_good = None
    unique_events = {}
    rows = []
    with capture.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
            if event['kind'] != 'call':
                continue
            if event['data']['output'][2]['reason'] == 'follow_front_unobserved':
                if first_bad is None:
                    first_bad, last_good = event, previous
                last_bad = event
                data, scan, history = restore_call(event['data']['input'], motion)
                scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                     epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
                rows.append(dict(seq=event['seq'], time_s=data['reference_s'], source_s=scene.source_first_s,
                                 invalid=front_invalid(scene, scan)))
                key = (scene.source_frame_id, scene.source_epoch, scene.source_first_s)
                if args.all_unknown_scans and key not in unique_events:
                    if len(unique_events) >= 128:
                        raise ValueError('all-scan diagnostic exceeds fixed 128-scan budget')
                    unique_events[key] = event
            previous = event
    if first_bad is None:
        raise ValueError('no recorded front-unobserved event')
    boxes, triangles, meshes = wall_geometry(args.run/'runtime_world.sdf')
    before.update({str(p): digest(p) for p in meshes})
    with (args.run/'ego_poses.jsonl').open() as stream:
        poses = np.array([json.loads(line) for line in stream])
    if poses.shape[1] != 4 or np.any(np.diff(poses[:, 0]) <= 0):
        raise ValueError('strictly ordered dense planar truth required')
    vehicle = ET.parse(args.run/'runtime_vehicle.sdf').getroot().find('model')
    sensor = pose(vehicle.find("link[@name='lidar_link']"))
    chassis = pose(vehicle.find("link[@name='chassis']"))
    report = json.loads((args.run/'report.json').read_text())
    settled_z = report['last_observation']['dynamics']['truth_z_m']
    sensor_z = settled_z+sensor[2]-chassis[2]
    examples = []
    chosen = [last_good, *unique_events.values()] if args.all_unknown_scans else [last_good, first_bad, last_bad]
    for event in chosen:
        data, scan, history = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        bad = front_invalid(scene, scan)
        indices = set(r['index'] for r in bad)
        if not indices:
            indices.update(r['index'] for r in rows[0]['invalid'])
        indices |= {i+j for i in list(indices) for j in (-1, 1) if 0 <= i+j < len(scan.ranges)}
        casts, unavailable = [], []
        for index in sorted(indices):
            acquired = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9+index*scan.time_increment
            try:
                bracket = pose_window(poses, acquired)
            except ValueError as error:
                unavailable.append(dict(index=index, acquired_s=acquired, error=str(error),
                    raw_kind=raw_kind(scan.ranges[index], scan.range_min, scan.range_max),
                    first_recorded_pose_s=float(poses[0, 0]), last_recorded_pose_s=float(poses[-1, 0])))
                continue
            for sample in bracket:
                _, x, y, yaw = sample
                c, s = math.cos(yaw), math.sin(yaw)
                origin = (x+c*sensor[0]-s*sensor[1], y+s*sensor[0]+c*sensor[1], sensor_z)
                angle = yaw+sensor[5]+scan.angle_min+index*scan.angle_increment
                direction = (math.cos(angle), math.sin(angle), 0.)
                hits = [(d, b.label) for b in boxes if (d := ray_box_distance(origin, direction, b, 0., 100.)) is not None]
                distance, label = min(hits, default=(None, None))
                casts.append(dict(index=index, acquired_s=acquired, pose_sample_s=float(sample[0]),
                    origin_xyz_m=list(origin), world_angle_rad=angle,
                    raw_kind=raw_kind(scan.ranges[index], scan.range_min, scan.range_max),
                    measured_m=float(scan.ranges[index]) if math.isfinite(scan.ranges[index]) else None,
                    collision_distance_m=distance, collision_label=label,
                    visual_two_sided_m=triangle_distance(origin, direction, triangles),
                    visual_front_facing_m=triangle_distance(origin, direction, triangles, True)))
        examples.append(dict(seq=event['seq'], time_s=data['reference_s'], range_max_m=scan.range_max,
                             reason=event['data']['output'][2]['reason'],
                             invalid=bad, casts=casts, unavailable=unavailable))
    result = dict(scope='saved_scan_and_planar_static_wall_ray_diagnostic_only',
        summary=dict(calls=capture_check['calls'], unknown_calls=len(rows),
            unique_unknown_scans=len({r['source_s'] for r in rows}),
            first_unknown_s=rows[0]['time_s'], last_unknown_s=rows[-1]['time_s'],
            raw_kind_counts=dict(Counter(v['raw_kind'] for r in rows for v in r['invalid']))),
        rows=rows, examples=examples, probe_mode='all_unique_unknown_scans' if args.all_unknown_scans else 'three_examples',
        geometry_summary=geometry_summary(examples), collision_boxes=len(boxes), visual_triangles=len(triangles),
        planar_sensor_height_m=sensor_z, height_assumption='settled final chassis z plus fixed SDF lidar offset; planar sensitivity, not exact acquisition pose',
        pose_assumption='recorded bracket endpoints around up-to-2.1ms acquisition lag; no interpolated truth or intermediate guarantee',
        runtime_changed=False, truth_to_controller=False, missing_rays_filled=False,
        complete_sensor_model_verified=False, input_sha256=before,
        input_unchanged=all(digest(p) == h for p, h in before.items()), source_sha256=sources,
        source_unchanged=all(digest(ROOT/p) == h for p, h in sources.items()))
    (args.output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(**result['summary'], geometry_summary=result['geometry_summary'])))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
