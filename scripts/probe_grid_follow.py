#!/usr/bin/env python3
"""공식 그리드 정지 장면의 G1/M3 첫 경로 진단. 주행/출발 허가가 아니다.

지도는 합성 광선 생성에만 쓴다. 제어기는 표적 이름·그리드·정답 자세를 받지 않는다.
벽 visual의 수평 단면과 정지 상자만 포함하므로 실제 Gazebo/반사/시설 가림 시험은 아니다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import yaml

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.m3_execution_trial import ExecutionConfig, M3ExecutionFollow
from audit_grid_start import CAMERA, RUNTIME, VEHICLE, registry
from diagnose_front_unknown import compose, pose
from g0_geometry import Footprint, TruthBox, ray_box_distance
from probe_corner_visibility_grid import cast_segments

ROOT = Path(__file__).resolve().parents[1]


def wall_sections(raw, height):
    """현재 원본의 개별 box visual만 읽는다. mesh/tilt를 생략해 빈 공간으로 만들지 않는다."""
    if not math.isfinite(height):
        raise ValueError('finite height required')
    model=ET.fromstring(raw).find("./world/model[@name='it_arena_track_static']")
    walls=model.find("link[@name='walls']") if model is not None else None
    if walls is None:
        raise ValueError('known walls link required')
    transform=compose(pose(model),pose(walls))
    segments=[]
    for visual in walls.findall('visual'):
        size=visual.findtext('geometry/box/size')
        if size is None:
            raise ValueError('box wall visual required')
        dims=list(map(float,size.split()))
        if len(dims)!=3 or not all(math.isfinite(v) and v>0 for v in dims):
            raise ValueError('positive finite box dimensions required')
        xyz=compose(transform,pose(visual))
        if xyz[2]-dims[2]/2<=height<=xyz[2]+dims[2]/2:
            points=Footprint(*dims[:2]).polygon(xyz[0],xyz[1],xyz[5])
            segments.extend(zip(points,points[1:]+points[:1]))
    if not segments:
        raise ValueError('no wall visual at measurement plane')
    return np.asarray(segments)


def stationary_ranges(slot, others, segments, sensor, samples, minimum, maximum):
    """원점/방향을 world로 보낸 뒤 가장 가까운 표면만 반환. 근접 가림도 투과하지 않는다."""
    x, y, yaw = slot['x'], slot['y'], slot['yaw_rad']
    sx, sy, z = sensor['xyz_m']
    c, s = math.cos(yaw), math.sin(yaw)
    origin = (x+c*sx-s*sy, y+s*sx+c*sy, z)
    local_angles = -math.pi+np.arange(samples)*math.tau/samples
    distances = cast_segments(segments, origin[:2], local_angles+yaw)
    labels = ['wall' if math.isfinite(d) else None for d in distances]
    for i, angle in enumerate(local_angles+yaw):
        direction = (math.cos(angle), math.sin(angle), 0.)
        for box in others:
            hit = ray_box_distance(origin, direction, box, 0., maximum)
            if hit is not None and hit < distances[i]:
                distances[i], labels[i] = hit, box.label
    # minimum 미만은 NaN, maximum 밖은 Inf. 라벨은 평가측에만 보존한다.
    ranges = np.where(distances < minimum, np.nan, np.where(distances <= maximum, distances, np.inf))
    return ranges, labels


def stationary_history(wheelbase, sensor):
    h = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-wheelbase/2,
                      lidar_x_m=sensor['xyz_m'][0], lidar_y_m=sensor['xyz_m'][1]))
    # 합성 정지 측정. 실제 센서 정확도를 증명하는 입력이 아니다.
    for i in range(201):
        h.add_mean_wheel(i*.01, 0.)
        h.add_gyro(i*.01, 0.)
    return h


def diagnose(ranges, sensor, lidar, wheelbase, mode):
    if mode not in ('g1', 'm3') or sensor['rate_hz'] != 10. or len(ranges) != lidar['samples_per_scan']:
        raise ValueError('registered G1/M3, 10 Hz and complete scan required')
    config = FollowConfig(corridor_fallback_enabled=True, boundary_evidence_mode='corner_distance_candidate',
                          occupancy_timing_mode='per_ray_candidate', wheelbase_m=wheelbase)
    controller = (LocalFollow(config) if mode == 'g1' else M3ExecutionFollow(config,
        ExecutionConfig(command_fallback=True, release_to_baseline=True, small_center_straight=True,
                        straight_surface_commitment=True), allow_simulation_trial=True))
    history = stationary_history(wheelbase, sensor)
    n, period = len(ranges), 1./sensor['rate_hz']
    step = period/n
    rows = []
    # 4개의 fresh scan, 각 5회 호출. 실제 차체/바퀴는 끝까지 정지해 있다.
    for frame in range(4):
        first = .5+frame*period
        stamps = first+np.arange(n)*step
        for repeat in range(5):
            now = float(stamps[-1]+.002+repeat*.02)
            scene = scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=math.tau/n,
                range_min=lidar['range_min_m'], range_max=lidar['range_max_m'], acquired_at_s=stamps.tolist(),
                t_ref_s=now, sensor_xy_m=tuple(sensor['xyz_m'][:2]),
                rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])
            speed, steering, meta = controller.command(scene, history, 0., 'left', 0., .02)
            rows.append(dict(t_s=now, scan_first_s=float(first), requested_speed_mps=speed,
                             requested_steering_rad=steering, meta=meta))
    final = rows[-1]
    return dict(mode=mode, calls=rows, reason_counts=dict(Counter(r['meta']['reason'] for r in rows)),
                positive_request_calls=sum(r['requested_speed_mps']>0 for r in rows),
                final_reason=final['meta']['reason'], final_speed_mps=final['requested_speed_mps'],
                final_steering_rad=final['requested_steering_rad'], driving_authorized=False)


def json_value(value):
    if isinstance(value, np.ndarray):
        return json_value(value.tolist())
    if isinstance(value, np.generic):
        return json_value(value.item())
    if isinstance(value, float) and not math.isfinite(value):
        return {'nonfinite': 'nan' if math.isnan(value) else ('+inf' if value>0 else '-inf')}
    if isinstance(value, dict):
        return {k:json_value(v) for k,v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(v) for v in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor = yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    lidar = vehicle['sensors']['lidar_2d']
    wheelbase = vehicle['drivetrain']['wheelbase_m']
    slots = registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),
                     (RUNTIME/'world.sdf').read_bytes(), vehicle)
    segments = wall_sections((RUNTIME/'world.sdf').read_bytes(), sensor['xyz_m'][2])
    paths = {Path(__file__), VEHICLE, CAMERA, RUNTIME/'scene.json', RUNTIME/'world.sdf',
             ROOT/'tests/test_grid_follow_probe.py'}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            path = Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                paths.add(path)
    sources = {p.relative_to(ROOT).as_posix(): p.read_bytes() for p in sorted(paths)}
    hashes = {name:hashlib.sha256(data).hexdigest() for name,data in sources.items()}
    with zipfile.ZipFile(output/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for name,data in sources.items():
            bundle.writestr(name, data)
    scenarios = [('solo', slot, []) for slot in slots]
    for count in (2,3,5):
        for slot in slots[:count]:
            scenarios.append((f'grid_{count}', slot, [p for p in slots[:count] if p['index']!=slot['index']]))
    results = []
    footprint = Footprint(vehicle['footprint']['length_m'], vehicle['footprint']['width_m'])
    for group, slot, other_slots in scenarios:
        others = [TruthBox(s['x'],s['y'],.12,footprint,s['yaw_rad'],label=f"grid_{s['index']}") for s in other_slots]
        ranges, labels = stationary_ranges(slot, others, segments, sensor, lidar['samples_per_scan'],
                                           lidar['range_min_m'], lidar['range_max_m'])
        result = dict(group=group, ego_grid_index=slot['index'], other_grid_indices=[s['index'] for s in other_slots],
                      ranges=json_value(ranges), evaluation_only_nearest_labels=labels,
                      modes=[diagnose(ranges,sensor,lidar,wheelbase,mode) for mode in ('g1','m3')])
        results.append(result)
    report = dict(scope='stationary synthetic official-grid LiDAR cold start, not departure',
                  scenarios=results, source_sha256=hashes,
                  sources_unchanged=all((ROOT/name).read_bytes()==data for name,data in sources.items()),
                  sensor=sensor, driving_authorized=False,
                  limits=['Walls visual cross-section and nominal stationary 20x15x12cm boxes only; other facilities excluded.',
                          'No reflectance, actual rendering, noise, motion, optical start signal or MCU/independent guard execution.',
                          'Positive planner request alone does not authorize driving or prove clearance/continuous safety.',
                          'Four fresh scans at fixed pose; commands do not move the car. Not a closed-loop trajectory.',
                          'Map/slot/actor labels are evaluator-only; planner receives LocalScene and zero motion measurements.'])
    (output/'report.json').write_text(json.dumps(json_value(report),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    compact=[dict(group=r['group'],slot=r['ego_grid_index'],modes=[{k:m[k] for k in
             ('mode','final_reason','final_speed_mps','final_steering_rad','positive_request_calls')} for m in r['modes']]) for r in results]
    (output/'summary.json').write_text(json.dumps(compact,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(scenarios=len(results),sources_unchanged=report['sources_unchanged'],driving_authorized=False)))
    return 0 if report['sources_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
