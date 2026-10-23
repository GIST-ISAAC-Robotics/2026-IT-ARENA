#!/usr/bin/env python3
"""평가용 코너 시점 격자: 관측 가능한 다른 진입 위치가 있는지 유한 표본으로 검사.

지도/정답은 가상 센서와 노면 사후 평가에만 사용한다. 경로 계획기·복구 명령이
아니며 정적 후보 사이의 실제 이동, 연속 외곽, 동적 차량을 증명하지 않는다.
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
from shapely.geometry import Polygon
from shapely.strtree import STRtree

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.observation_support import sweep_support
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons
from diagnose_front_unknown import digest, pose, wall_geometry
from observation_witness import arc_body

ROOT = Path(__file__).resolve().parents[1]
STEERS = (-.12, -.06, 0., .06, .12, .18)


def horizontal_segments(triangles, height):
    """변환된 삼각형의 측정면 교차. 면 전체 일치는 지원하지 않는다."""
    a = np.asarray(triangles, dtype=float)
    if a.ndim != 3 or a.shape[1:] != (3, 3) or not np.isfinite(a).all() or not math.isfinite(height):
        raise ValueError('finite triangles and height required')
    segments = []
    for tri in a:
        dz = tri[:, 2]-height
        if np.all(abs(dz) < 1e-12):
            raise ValueError('coplanar triangle at measurement plane unsupported')
        points = [tri[k, :2] for k in range(3) if abs(dz[k]) < 1e-12]
        for i, j in ((0, 1), (1, 2), (2, 0)):
            if dz[i]*dz[j] < 0:
                points.append(tri[i, :2]+dz[i]/(dz[i]-dz[j])*(tri[j, :2]-tri[i, :2]))
        unique = []
        for p in points:
            if not any(np.linalg.norm(p-q) < 1e-12 for q in unique):
                unique.append(p)
        if len(unique) == 2 and np.linalg.norm(unique[0]-unique[1]) > 1e-12:
            segments.append(unique)
    if not segments:
        raise ValueError('no wall intersects measurement plane')
    return np.asarray(segments)


def cast_segments(segments, origin, angles):
    """양면 시각 메시 교차. 무교차는 inf 그대로이며 최대거리로 채우지 않는다."""
    seg = np.asarray(segments, dtype=float)
    origin, angles = np.asarray(origin, dtype=float), np.asarray(angles, dtype=float)
    if (seg.ndim != 3 or seg.shape[1:] != (2, 2) or angles.ndim != 1 or origin.shape not in ((2,), (len(angles), 2))
            or not np.isfinite(seg).all() or not np.isfinite(origin).all() or not np.isfinite(angles).all()):
        raise ValueError('finite planar rays/segments required')
    result = np.full(len(angles), np.inf)
    edge = seg[:, 1]-seg[:, 0]
    for start in range(0, len(angles), 32):
        a = angles[start:start+32]
        origins = np.broadcast_to(origin, (len(angles), 2))[start:start+len(a)]
        delta = seg[None, :, 0]-origins[:, None]
        numerator = delta[:, :, 0]*edge[None, :, 1]-delta[:, :, 1]*edge[None, :, 0]
        dx, dy = np.cos(a)[:, None], np.sin(a)[:, None]
        denominator = dx*edge[None, :, 1]-dy*edge[None, :, 0]
        valid = abs(denominator) > 1e-12
        t = np.divide(numerator, denominator, out=np.full_like(denominator, np.inf), where=valid)
        u = np.divide(delta[:, :, 0]*dy-delta[:, :, 1]*dx, denominator,
                      out=np.full_like(denominator, np.inf), where=valid)
        hits = valid & (t > 1e-6) & (u >= -1e-12) & (u <= 1.+1e-12)
        result[start:start+len(a)] = np.min(np.where(hits, t, np.inf), axis=1)
    return result


def shifted_pose(x, y, yaw, lateral, yaw_offset=0.):
    return np.array([x-math.sin(yaw)*lateral, y+math.cos(yaw)*lateral, yaw+yaw_offset])


def scene_at(base, segments, raw, motion, age, rear_half_angle):
    c, s = math.cos(base[2]), math.sin(base[2])
    origin = [base[0]+c*motion.lidar_x_m-s*motion.lidar_y_m,
              base[1]+s*motion.lidar_x_m+c*motion.lidar_y_m]
    angles = raw.angle_min+np.arange(500)*raw.angle_increment
    distances = cast_segments(segments, origin, angles+base[2]+motion.lidar_yaw_rad)
    # 공식 범위 밖/너무 가까운 반환은 원래와 같이 unknown으로 남긴다.
    ranges = np.where((distances >= raw.range_min) & (distances <= raw.range_max), distances, np.inf)
    scene = scene_from_scan(ranges.tolist(), angle_min=raw.angle_min, angle_increment=raw.angle_increment,
        range_min=raw.range_min, range_max=raw.range_max,
        acquired_at_s=[.5+i*raw.time_increment for i in range(500)], t_ref_s=.5+age,
        sensor_xy_m=(motion.lidar_x_m, motion.lidar_y_m), sensor_yaw_rad=motion.lidar_yaw_rad,
        rear_half_angle_deg=rear_half_angle)
    return scene, ranges


def arc_transforms(travels, steering, cfg):
    """시나리오 생성기용 일정 후륜 속도/곡률의 base_link SE(2). 실제 운동 추정 아님."""
    distance = np.asarray(travels, dtype=float)
    curvature = math.tan(steering)/cfg.wheelbase_m
    yaw = curvature*distance
    x = distance.copy() if curvature == 0. else np.sin(yaw)/curvature
    y = np.zeros_like(distance) if curvature == 0. else 2*np.sin(yaw*.5)**2/curvature
    return np.column_stack((x+cfg.wheelbase_m*.5*(np.cos(yaw)-1.),
                            y+cfg.wheelbase_m*.5*np.sin(yaw), yaw))


def steady_scenes(base, segments, raw, motion, cfg, age, rear_half_angle, steering, speed=.2):
    """현재 이전 0.3 s 안의 일정 원호에서 순차 취득한 가상 관측을 현재로 투영.

    지도는 광선 생성에만 사용한다. 완전한 운동 보정이 가능한 가정의 진입 여지
    검사이며 모터/IMU 오차, 동역학, 실제 제어기의 이력 오차를 재현하지 않는다.
    """
    now = .5+age
    cycle = raw.time_increment*500
    sources = []
    for k in range(3):
        first = .5-k*cycle
        if now-first >= .3:
            continue
        stamps = first+np.arange(500)*raw.time_increment
        poses = arc_transforms(speed*(stamps-now), steering, cfg)
        turn = poses[:, 2]
        origins = poses[:, :2]+np.column_stack((np.cos(turn)*motion.lidar_x_m-np.sin(turn)*motion.lidar_y_m,
                                                np.sin(turn)*motion.lidar_x_m+np.cos(turn)*motion.lidar_y_m))
        c, s = math.cos(base[2]), math.sin(base[2])
        world = origins@np.array([[c, s], [-s, c]])+base[:2]
        angles = raw.angle_min+np.arange(500)*raw.angle_increment
        distances = cast_segments(segments, world, base[2]+turn+motion.lidar_yaw_rad+angles)
        ranges = np.where((distances >= raw.range_min) & (distances <= raw.range_max), distances, np.inf)
        sources.append(scene_from_scan(ranges.tolist(), angle_min=raw.angle_min, angle_increment=raw.angle_increment,
            range_min=raw.range_min, range_max=raw.range_max, acquired_at_s=stamps.tolist(), t_ref_s=now,
            sensor_xy_m=(motion.lidar_x_m, motion.lidar_y_m), sensor_yaw_rad=motion.lidar_yaw_rad,
            acquisition_to_ref_se2=poses.tolist(), rear_half_angle_deg=rear_half_angle))
    return sources


def footprint_audit(base, steering, distance, cfg, road, obstacles, tree):
    """원래 20x15 cm 명목 몸체의 1 cm 간격 표본. 기존 면적 보고 기준을 유지."""
    c, s = math.cos(base[2]), math.sin(base[2])
    rotation = np.array([[c, s], [-s, c]])
    outside, overlap = [], []
    for travel in np.r_[np.arange(0., distance, math.copysign(.01, distance)), distance]:
        body = Polygon(arc_body(steering, travel, cfg)@rotation+np.asarray(base[:2]))
        outside.append(float(body.difference(road).area))
        overlap.append(max((float(body.intersection(obstacles[int(j)]).area) for j in tree.query(body)), default=0.))
    return dict(samples=len(outside), maximum_outside_road_m2=max(outside), maximum_wall_overlap_m2=max(overlap),
                sampled_geometry_pass=bool(max(outside) <= .0001 and max(overlap) <= 1e-6),
                continuous_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--history-mode', choices=('stopped', 'steady-arc'), default='stopped')
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    centerline = ROOT/'src/arena_gazebo/worlds/it_arena_official/centerline.csv'
    inputs = [args.run/n for n in ('follow_decisions.jsonl', 'ego_poses.jsonl', 'runtime_world.sdf',
                                   'runtime_vehicle.sdf', 'report.json')]+[centerline]
    hashes = {str(p): digest(p) for p in inputs}
    with inputs[0].open() as stream:
        for line in stream:
            e = json.loads(line)
            if e['kind'] == 'header':
                motion = MotionConfig(**e['data']['motion_config'])
                cfg = FollowConfig(**e['data']['follow_config'])
            if e['kind'] == 'call' and e['seq'] == 3501:
                data, raw, _ = restore_call(e['data']['input'], motion)
                break
        else:
            raise ValueError('fixed seq3501 required')
    vehicle = ET.parse(inputs[3]).find('model')
    sensor, chassis = pose(vehicle.find("link[@name='lidar_link']")), pose(vehicle.find("link[@name='chassis']"))
    if not np.allclose([sensor[0], sensor[1], sensor[5]],
                       [motion.lidar_x_m, motion.lidar_y_m, motion.lidar_yaw_rad], atol=1e-9, rtol=0.):
        raise ValueError('recorded sensor geometry differs')
    z = json.loads(inputs[4].read_text())['last_observation']['dynamics']['truth_z_m']+sensor[2]-chassis[2]
    _, triangles, meshes = wall_geometry(inputs[2])
    hashes.update({str(p): digest(p) for p in meshes})
    segments = horizontal_segments(triangles, z)
    road, obstacles = road_polygon(inputs[2]), obstacle_polygons(inputs[2])
    tree = STRtree(obstacles)
    age = data['reference_s']-(raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9)
    sensor_pose = (motion.lidar_x_m, motion.lidar_y_m, motion.lidar_yaw_rad)
    # 고정한 0.2 m/s 목표와 기존 거리식만 사용한다. 속도 상한은 조정하지 않는다.
    distance = cfg.standstill_gap_m+.06+.2*(age+.10)+.2*.2/4
    paths = {Path(__file__).resolve(), ROOT/'tests/test_corner_visibility_grid.py'}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            p = Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix == '.py':
                paths.add(p)
    source_hashes = {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(paths)}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in source_hashes:
            archive.write(ROOT/name, name)
    poses = np.array([json.loads(line) for line in inputs[1].open()])
    p = poses[np.argmin(abs(poses[:, 0]-data['reference_s']))]
    if abs(p[0]-data['reference_s']) > .006:
        raise ValueError('nearby recorded pose unavailable')
    original_scene, original_ranges = scene_at(p[1:], segments, raw, motion, age, data['rear_half_angle_deg'])
    front = abs(raw.angle_min+np.arange(500)*raw.angle_increment) <= math.radians(100)
    both = front & np.isfinite(original_ranges) & np.isfinite(raw.ranges)
    error = abs(original_ranges[both]-np.asarray(raw.ranges)[both])
    calibration = dict(recorded_pose=p.tolist(), uses_nearest_pose_and_last_report_z=True,
        samples=len(error), absolute_error_p50_m=float(np.median(error)),
        absolute_error_p95_m=float(np.percentile(error, 95)), absolute_error_max_m=float(max(error)),
        simulated_unknown_indices=np.flatnonzero(front & ~np.isfinite(original_ranges)).tolist(),
        recorded_unknown_indices=np.flatnonzero(front & ~np.isfinite(raw.ranges)).tolist())
    cl = np.genfromtxt(centerline, delimiter=',', names=True)
    indices = np.flatnonzero((cl['s_m'] >= 40.4) & (cl['s_m'] <= 42.56))[::8]
    rows = []
    for i in indices:
        yaw = math.atan2(cl['y_m'][i+1]-cl['y_m'][i-1], cl['x_m'][i+1]-cl['x_m'][i-1])
        for lateral in (-.06, 0., .06):
            for yaw_deg in ((-8., 0., 8.) if args.history_mode == 'stopped' else (0.,)):
                base = shifted_pose(cl['x_m'][i], cl['y_m'][i], yaw, lateral, math.radians(yaw_deg))
                body = footprint_audit(base, 0., 0., cfg, road, obstacles, tree)
                row = dict(s_m=float(cl['s_m'][i]), lateral_m=lateral, yaw_offset_deg=yaw_deg,
                           world_pose=base.tolist(), body=body, cases=[])
                if body['sampled_geometry_pass']:
                    scene, _ = scene_at(base, segments, raw, motion, age, data['rear_half_angle_deg'])
                    row['unknown_bearings_deg'] = [math.degrees(math.atan2(r.direction_xy[1], r.direction_xy[0]))
                                                    for r in scene.rays if r.status == 'unknown']
                    for steer in STEERS:
                        moving = args.history_mode == 'steady-arc'
                        sources = (steady_scenes(base, segments, raw, motion, cfg, age, data['rear_half_angle_deg'], steer)
                                   if moving else [scene])
                        previous, speed = (steer, .2) if moving else (0., 0.)
                        support = sweep_support(sources, speed, .2, steer, previous, cfg, protected_sensor_pose=sensor_pose)
                        geometry = footprint_audit(base, steer, distance, cfg, road, obstacles, tree)
                        past = footprint_audit(base, steer, -.2*(sources[0].t_ref_s-sources[-1].source_first_s)
                                               if moving else 0., cfg, road, obstacles, tree)
                        row['cases'].append(dict(steering_rad=steer, observation=support, geometry=geometry,
                            history_geometry=past, sources=len(sources),
                            eligible_static_sample=bool(support['supported'] and geometry['sampled_geometry_pass']
                                                       and past['sampled_geometry_pass']),
                            previous_steering_rad=previous, one_tick_rate_possible=abs(steer-previous) <= .06))
                rows.append(row)
    counts = Counter(c['observation']['reason'] for row in rows for c in row['cases'])
    report = dict(scope='static_truth_generated_sensor_grid_not_driving', calibration=calibration,
        rows=rows, grid_positions=len(rows), on_road_positions=sum(bool(r['cases']) for r in rows),
        observation_reasons=dict(counts), measurement_age_s=age, lidar_plane_z_m=z, wall_segments=len(segments),
        distance_m=distance, history_mode=args.history_mode,
        sensor_history_assumption='one_stationary_scan' if args.history_mode == 'stopped' else 'ideal_constant_rear_speed_arc',
        rendered_noise_or_materials=False,
        controller_received_truth=False, runtime_modified=False, movement_between_poses_verified=False,
        source_sha256=source_hashes, source_unchanged=all(digest(ROOT/p) == h for p, h in source_hashes.items()),
        input_sha256=hashes, input_unchanged=all(digest(p) == h for p, h in hashes.items()))
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','input_sha256','source_sha256')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
