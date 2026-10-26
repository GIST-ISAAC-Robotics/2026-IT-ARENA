#!/usr/bin/env python3
"""M5 출발의 경계/간격 원인 분리. 명령 발행·위험 예산 변경 없는 평가 전용 도구."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np
import yaml
from shapely.geometry import Polygon

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path, swept_limit
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.occupancy_timing import per_ray_padding
from probe_grid_follow import (ROOT, CAMERA, VEHICLE, RUNTIME, diagnose, json_value,
                               registry, stationary_ranges, wall_sections)
from g0_geometry import Footprint, TruthBox

PRIOR = ROOT/'artifacts/validation/2026-10-26/m5_grid_follow_v1/run2/data/report.json'


def decode_ranges(values):
    codes = {'nan': math.nan, '+inf': math.inf, '-inf': -math.inf}
    return np.array([codes[v['nonfinite']] if isinstance(v, dict) else v for v in values])


def stationary_scene(ranges, sensor, lidar, repeat):
    if repeat not in (0, 4):
        raise ValueError('registered fresh or last repeat required')
    n = len(ranges)
    stamps = .8+np.arange(n)/sensor['rate_hz']/n
    return scene_from_scan(ranges.tolist(), angle_min=-math.pi, angle_increment=math.tau/n,
        range_min=lidar['range_min_m'], range_max=lidar['range_max_m'], acquired_at_s=stamps.tolist(),
        t_ref_s=float(stamps[-1]+.002+repeat*.02), sensor_xy_m=tuple(sensor['xyz_m'][:2]),
        rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])


def constraints(scene, cfg):
    """순수 센서 분석. truth label 없이 초기 점유와 좌우 경계 조건을 분리한다."""
    rays = [r for r in scene.rays if r.status == 'return']
    points = np.array([r.hit_xy_m for r in rays]).reshape(-1, 2)
    if not len(points):
        return dict(reason='no_returns', driving_authorized=False)
    base = .025+cfg.measurement_padding_m
    padding = per_ray_padding(scene, base, cfg.object_motion_bound_mps)
    # 축 정렬 명목 footprint부터의 L-infinity 여유. 유클리드 최단거리가 아니다.
    residual = np.max(np.abs(points)-[cfg.length_m/2, cfg.width_m/2], axis=1)
    overlaps = residual <= padding
    ordered = sorted(np.flatnonzero(overlaps), key=lambda i: residual[i]-padding[i])
    boundary = {}
    for side in ('left', 'right'):
        path, info = local_path(points, side, cfg.wall_offset_m)
        ahead = info.get('boundary_observed_ahead_m')
        distance = info.get('wall_distance_m')
        boundary[side] = dict(reason=info['reason'], ahead_m=ahead, anchor_distance_m=distance,
            length_ok=bool(path is not None and ahead >= cfg.boundary_min_ahead_m),
            anchor_ok=bool(path is not None and abs(distance-cfg.wall_offset_m) <= cfg.boundary_anchor_error_m))
    diagnostic = []
    for name, margin in (('raw_geometry_only', 0.), ('fixed_padding_only', base),
                         ('existing_per_ray_padding', padding)):
        diagnostic.append(dict(component=name, steering_clearance_m={str(angle): swept_limit(
            points, angle, cfg.wheelbase_m, horizon=2.5, margin=margin,
            length=cfg.length_m, width=cfg.width_m) for angle in (0., -.06, .06)},
            policy_alternative=False))
    return dict(reason='diagnostic_only', boundary=boundary,
        initial_inflated_overlap_count=int(overlaps.sum()),
        minimum_raw_linf_clearance_m=float(residual.min()),
        minimum_existing_margin_residual_m=float(np.min(residual-padding)),
        worst_initial_returns=[dict(source_index=rays[i].source_index, xy_m=points[i].tolist(),
             ray_age_s=scene.t_ref_s-rays[i].acquired_at_s, fixed_padding_m=base,
             motion_padding_m=float(padding[i]-base), raw_linf_clearance_m=float(residual[i]),
             existing_margin_residual_m=float(residual[i]-padding[i])) for i in ordered[:5]],
        swept_components=diagnostic, driving_authorized=False)


def translated_peers(ego, peers, distance, selection):
    """평가용 정지 반사실. 차량을 지우지 않고 지정한 앞 방향 위치로 옮긴다."""
    if selection not in ('all', 'ahead_only') or not math.isfinite(distance) or distance < 0:
        raise ValueError('registered nonnegative peer displacement required')
    c, s = math.cos(ego['yaw_rad']), math.sin(ego['yaw_rad'])
    result = []
    for peer in peers:
        ahead = (peer['x']-ego['x'])*c+(peer['y']-ego['y'])*s > 0
        shift = distance if selection == 'all' or ahead else 0.
        result.append(dict(peer, x=peer['x']+shift*math.cos(peer['yaw_rad']),
                           y=peer['y']+shift*math.sin(peer['yaw_rad'])))
    return result


def placement_checks(ego, peers, footprint):
    """이동 반사실의 겹침을 평가한다. 제어 입력에는 전달하지 않는다."""
    slots = [ego, *peers]
    polygons = [Polygon(footprint.polygon(s['x'], s['y'], s['yaw_rad'])) for s in slots]
    pairs = []
    for i, a in enumerate(polygons):
        for j in range(i+1, len(polygons)):
            area = a.intersection(polygons[j]).area
            if area > 1e-12:
                pairs.append(dict(indices=[slots[i]['index'],slots[j]['index']], overlap_area_m2=area))
    return dict(nonoverlapping=not pairs, overlaps=pairs, trajectory_validated=False,
                road_validated=False, evaluation_only=True)


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
    cfg = FollowConfig(wheelbase_m=vehicle['drivetrain']['wheelbase_m'])
    slots = registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),
                     (RUNTIME/'world.sdf').read_bytes(), vehicle)
    segments = wall_sections((RUNTIME/'world.sdf').read_bytes(), sensor['xyz_m'][2])
    footprint = Footprint(vehicle['footprint']['length_m'], vehicle['footprint']['width_m'])
    prior = json.loads(PRIOR.read_text(encoding='utf-8'))
    paths = {Path(__file__), PRIOR, CAMERA, VEHICLE, RUNTIME/'scene.json', RUNTIME/'world.sdf',
             ROOT/'tests/test_grid_start_constraints.py'}
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
    results = []
    for ego in slots[:5]:
        peers = [s for s in slots[:5] if s['index'] != ego['index']]
        for selection, distances in (('all', (0., .25, .5, 1., 1.5)), ('ahead_only', (.5, 1.5))):
            for distance in distances:
                moved = translated_peers(ego, peers, distance, selection)
                boxes = [TruthBox(p['x'],p['y'],.12,footprint,p['yaw_rad'],label=f"grid_{p['index']}") for p in moved]
                ranges, labels = stationary_ranges(ego, boxes, segments, sensor, lidar['samples_per_scan'],
                                                   lidar['range_min_m'], lidar['range_max_m'])
                diagnosis = diagnose(ranges, sensor, lidar, cfg.wheelbase_m, 'g1')
                row = dict(ego_grid_index=ego['index'], selection=selection, peer_displacement_m=distance,
                           ranges=json_value(ranges), diagnosis=diagnosis, constraints={},
                           placement=placement_checks(ego,moved,footprint))
                if selection == 'ahead_only' and ego['index'] in (5,4):
                    row['existing_m3'] = diagnose(ranges,sensor,lidar,cfg.wheelbase_m,'m3')
                for repeat in (0, 4):
                    data = constraints(stationary_scene(ranges, sensor, lidar, repeat), cfg)
                    for point in data.get('worst_initial_returns', []):
                        point['evaluation_only_label'] = labels[point['source_index']]
                    row['constraints'][str(repeat)] = data
                if distance == 0:
                    old = next(r for r in prior['scenarios'] if r['group']=='grid_5' and r['ego_grid_index']==ego['index'])
                    old_diagnosis = next(m for m in old['modes'] if m['mode']=='g1')
                    row['saved_input_equal'] = bool(np.array_equal(ranges, decode_ranges(old['ranges']), equal_nan=True))
                    row['saved_decisions_equal'] = diagnosis == old_diagnosis
                results.append(row)
    report = dict(scope='stationary counterfactual positions, not moving actors or ego driving',
                  scenarios=results, source_sha256=hashes, driving_authorized=False,
                  sources_unchanged=all((ROOT/name).read_bytes()==data for name,data in sources.items()),
                  limits=['No risk budget or runtime/controller changes.',
                          'Peer translations are independent fixed scenes, not trajectories or sequential departure proof.',
                          'No optical signal, independent guard, MCU, actor rotation/cut-in or real sensor effects.',
                          'Raw/fixed-only collision distances are causal decompositions, never driving alternatives.'])
    (output/'report.json').write_text(json.dumps(json_value(report),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    compact = [dict(slot=r['ego_grid_index'], selection=r['selection'], shift=r['peer_displacement_m'],
                    reason=r['diagnosis']['final_reason'], positive=r['diagnosis']['positive_request_calls'],
                    initial_overlap=r['constraints']['0'].get('initial_inflated_overlap_count'),
                    boundary=r['constraints']['0'].get('boundary'),
                    nonoverlapping=r['placement']['nonoverlapping'],
                    m3_reason=r.get('existing_m3',{}).get('final_reason'),
                    saved_input_equal=r.get('saved_input_equal'), saved_decisions_equal=r.get('saved_decisions_equal')) for r in results]
    (output/'summary.json').write_text(json.dumps(compact,ensure_ascii=False,indent=2),encoding='utf-8')
    preserved = all(r.get('saved_input_equal',True) and r.get('saved_decisions_equal',True) for r in results)
    print(json.dumps(dict(scenarios=len(results),sources_unchanged=report['sources_unchanged'],
                         originals_equal=preserved, reasons=dict(Counter(r['reason'] for r in compact)))))
    return 0 if report['sources_unchanged'] and preserved else 1


if __name__ == '__main__':
    raise SystemExit(main())
