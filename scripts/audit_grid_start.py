#!/usr/bin/env python3
"""M5 평가 전용 그리드 registry·정적 기하·시야. 운영 spawn/주행 허가가 아니다."""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

import numpy as np
import yaml

from audit_local_pursuit_road import road_polygon
from audit_start_signal_geometry import lamps_from_sdf, pose, project, rotation, REVISION
from build_experimental_track import obstacle_polygons, rectangle
from validate_start_signal_render import put

ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT/'artifacts/validation/2026-10-13/race_start_geometry_v1/result'
RUNTIME = ROOT/'src/arena_gazebo/worlds/it_arena_official'
VEHICLE = ROOT/'src/arena_description/config/vehicle.yaml'
CAMERA = ROOT/'src/arena_description/config/b0183_c1.yaml'


def registry(scene, raw_world, vehicle):
    """scene의 칸 중심 = model origin = 축간 중앙 지면. 후축으로 이동하지 않는다."""
    if vehicle['reference_frame'] != 'model_origin':
        raise ValueError('unsupported vehicle reference frame')
    grid = scene['starting_grid']
    slots = grid['slots']
    if len(slots) != 6 or {s['index'] for s in slots} != set(range(6)):
        raise ValueError('six unique official slots required')
    model = ET.fromstring(raw_world).find("world/model[@name='it_arena_track_static']")
    if model is None:
        raise ValueError('missing track')
    origin, orientation = pose(model)
    if not np.allclose(origin, 0) or not np.allclose(orientation, np.eye(3)):
        raise ValueError('world-origin model required')
    rows = []
    for slot in sorted(slots, key=lambda s: -s['s_m']):
        if not all(math.isfinite(slot[k]) for k in ('x', 'y', 'yaw_rad', 's_m')):
            raise ValueError('nonfinite grid pose')
        # 실행 변환은 grid 링크를 합치되 visual pose에 원래 pose를 옮긴다.
        # 어느 표현이든 실제 부모/visual 좌표를 합성해 대조한다. 이름만 믿지 않는다.
        index = slot['index']
        candidates = [(link, visual) for link in model.findall('link') for visual in link.findall('visual')
                      if (link.get('name') == f'grid_slot_{index}' and visual.get('name') == f'grid_slot_{index}_vis')
                      or visual.get('name') == f'grid_slot_{index}__grid_slot_{index}_vis']
        if len(candidates) != 1:
            raise ValueError('grid paint missing or ambiguous')
        link, visual = candidates[0]
        local, local_basis = pose(link)
        vxyz, vbasis = pose(visual)
        xyz, basis = local+local_basis@vxyz, local_basis@vbasis
        size = [float(x) for x in visual.findtext('geometry/box/size').split()]
        if (not np.allclose(xyz[:2], [slot['x'], slot['y']], atol=1e-6, rtol=0)
                or not np.allclose(basis, rotation(0, 0, slot['yaw_rad']), atol=1e-6, rtol=0)
                or not np.allclose(size[:2], [grid['slot_length_m'], grid['slot_width_m']], atol=1e-6, rtol=0)):
            raise ValueError('scene/SDF grid mismatch')
        rows.append(dict(slot, front_to_back_order=len(rows)+1,
                         spawn_model_origin=[slot['x'], slot['y'], 0., 0., 0., slot['yaw_rad']],
                         paint_z_m=float(xyz[2]), footprint=vehicle['footprint']))
    return rows


def measure(rows, road, obstacles, length, width, comparison_gap=.15):
    polygons = {r['index']: rectangle(r['x'], r['y'], r['yaw_rad'], length, width) for r in rows}
    per_slot = []
    for r in rows:
        p = polygons[r['index']]
        per_slot.append(dict(grid_index=r['index'], outside_road_area_m2=p.difference(road).area,
            wall_intersection_area_m2=sum(p.intersection(o).area for o in obstacles),
            road_edge_distance_m=p.distance(road.boundary)))
    subsets = {}
    for count in (2, 3, 5):
        chosen = rows[:count]
        pairs = []
        for a, b in itertools.combinations(chosen, 2):
            first, second = polygons[a['index']], polygons[b['index']]
            relative = rotation(0, 0, a['yaw_rad']).T@np.array([b['x']-a['x'], b['y']-a['y'], 0.])
            pairs.append(dict(slots=[a['index'], b['index']], body_gap_m=first.distance(second),
                overlap_area_m2=first.intersection(second).area,
                center_longitudinal_m=float(relative[0]), center_lateral_m=float(relative[1])))
        subset_slots = [s for s in per_slot if s['grid_index'] in [r['index'] for r in chosen]]
        subsets[str(count)] = dict(slot_indices=[r['index'] for r in chosen], pairs=pairs,
            minimum_gap_m=min(p['body_gap_m'] for p in pairs),
            no_overlap=all(p['overlap_area_m2'] <= 1e-9 for p in pairs),
            contained_on_road=all(s['outside_road_area_m2'] <= 1e-9 for s in subset_slots),
            no_low_wall_overlap=all(s['wall_intersection_area_m2'] <= 1e-9 for s in subset_slots),
            meets_existing_traffic_gap=all(p['body_gap_m'] >= comparison_gap for p in pairs))
    return dict(slots=per_slot, subsets=subsets, comparison_gap_m=comparison_gap,
                interpretation='nominal stationary straight-wheel footprints; not steering sweep or departure clearance')


def camera_model(world, name, xyz, rpy, camera):
    model = ET.SubElement(world, 'model', name='m5_camera_'+name)
    put(model, 'static', 'true')
    put(model, 'pose', ' '.join(map(str, [*xyz, *rpy])))
    sensor = ET.SubElement(ET.SubElement(model, 'link', name='link'), 'sensor', name='rgb', type='camera')
    put(sensor, 'always_on', 'true')
    put(sensor, 'update_rate', 2)
    topic = '/m5_grid/'+name
    put(sensor, 'topic', topic)
    cam = ET.SubElement(sensor, 'camera')
    put(cam, 'horizontal_fov', math.radians(camera['horizontal_fov_deg']))
    img = ET.SubElement(cam, 'image')
    for key, value in (('width', camera['width_px']), ('height', camera['height_px']), ('format', 'R8G8B8')):
        put(img, key, value)
    clip = ET.SubElement(cam, 'clip')
    put(clip, 'near', .01)
    put(clip, 'far', 40)
    return dict(topic=topic, xyz=list(map(float, xyz)), rpy=list(map(float, rpy)), camera=camera)


def build_fixture(raw_world, rows, vehicle, camera, actor):
    """원본 기하/램프는 그대로, 정적 상자 5개와 평가용 카메라만 추가. 제어 플러그인 없음."""
    root = ET.fromstring(raw_world)
    world = root.find('world')
    world.set('name', 'm5_grid_fixture')
    if world.findall('plugin'):
        raise ValueError('official input unexpectedly has world plugins')
    for plugin, name in (('physics', 'Physics'), ('user-commands', 'UserCommands'),
                         ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')):
        p = ET.SubElement(world, 'plugin', filename=f'gz-sim-{plugin}-system', name='gz::sim::systems::'+name)
        if plugin == 'sensors':
            put(p, 'render_engine', 'ogre2')
    # 원본 URI의 데이터는 기존 공식 파생본과 같은 PNG. 원본 파일 자체는 건드리지 않는다.
    for texture in world.findall('.//albedo_map'):
        texture.text = str((RUNTIME/'aruco'/Path(texture.text).name).resolve())
    chosen = rows[:5]
    for r in chosen:
        model = ET.SubElement(world, 'model', name=f"eval_grid_{r['index']}")
        put(model, 'static', 'true')
        put(model, 'pose', f"{r['x']} {r['y']} {actor['bottom_m']+actor['height_m']/2} 0 0 {r['yaw_rad']}")
        link = ET.SubElement(model, 'link', name='body')
        visual = ET.SubElement(link, 'visual', name='body')
        box = ET.SubElement(ET.SubElement(visual, 'geometry'), 'box')
        put(box, 'size', f"{vehicle['footprint']['length_m']} {vehicle['footprint']['width_m']} {actor['height_m']}")
        material = ET.SubElement(visual, 'material')
        color = '.15 .55 .9 1' if r is chosen[0] else '.85 .3 .15 1'
        put(material, 'ambient', color)
        put(material, 'diffuse', color)
    views = {}
    for r in (chosen[0], chosen[-1]):
        xyz = np.array([r['x'], r['y'], 0.])+rotation(0, 0, r['yaw_rad'])@camera['xyz_m']
        views[f"slot{r['index']}"] = camera_model(world, f"slot{r['index']}", xyz,
            [*camera['rpy_rad'][:2], r['yaw_rad']+camera['rpy_rad'][2]], camera)
    center = np.mean([[r['x'], r['y'], 0.] for r in chosen], axis=0)
    xyz = center+[-1.6, -.8, 2.2]
    delta = center-xyz
    rpy = [0., math.atan2(-delta[2], math.hypot(*delta[:2])), math.atan2(delta[1], delta[0])]
    views['overview'] = camera_model(world, 'overview', xyz, rpy, dict(camera, horizontal_fov_deg=60.))
    return root, views


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('fresh artifacts subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    vehicle = yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    camera = yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['camera']
    actor_path = ROOT/'config/tests/g1_traffic_v1.json'
    actor = json.loads(actor_path.read_text(encoding='utf-8'))['actor']
    scene = json.loads((OFFICIAL/'scene.json').read_text(encoding='utf-8'))
    rows = registry(scene, (OFFICIAL/'world.sdf').read_bytes(), vehicle)
    current_scene = json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8'))
    runtime_rows = registry(current_scene, (RUNTIME/'world.sdf').read_bytes(), vehicle)
    lamps = lamps_from_sdf((OFFICIAL/'world.sdf').read_bytes())
    paths = [Path(__file__), ROOT/'scripts/capture_grid_start.py', ROOT/'scripts/lidar_shutdown.py',
             ROOT/'tests/test_grid_start.py', VEHICLE, CAMERA, actor_path,
             OFFICIAL/'scene.json', OFFICIAL/'world.sdf', RUNTIME/'scene.json', RUNTIME/'world.sdf',
             ROOT/'scripts/audit_start_signal_geometry.py', ROOT/'scripts/audit_local_pursuit_road.py',
             ROOT/'scripts/build_experimental_track.py', ROOT/'scripts/validate_start_signal_render.py',
             *sorted((RUNTIME/'aruco').glob('*.png'))]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    report = dict(scope='M5 static grid preflight; no driving or hardware validation', official_revision=REVISION,
        source_sha256=hashes, registry=rows, registry_matches_runtime=rows == runtime_rows,
        order_basis='descending s on official starting straight, not an official qualifying assignment',
        geometry=measure(rows, road_polygon(OFFICIAL/'world.sdf'), obstacle_polygons(OFFICIAL/'world.sdf'),
                         vehicle['footprint']['length_m'], vehicle['footprint']['width_m']),
        official_light_views=[project(lamps, r, camera) for r in rows],
        nominal_base_z_m=0., simulation_initial_spawn_z_m=.01,
        spawn_height_sensitivity=[project(lamps, r, camera, base_z=.01) for r in rows],
        limitations=['nominal base at ground; suspension settled pose unmeasured',
            'pinhole FOV, no distortion/real exposure; projection does not check occlusion',
            'low wall planar check covers vertical interval 0.02..0.10 m only',
            'render boxes use E3 0.12m height; real competitor geometry unknown',
            'initial gap below 0.15m is not a new waiver or a contact claim'])
    report['departure_ready'] = False  # 아직 동적 출발 검증 없음. 정적 합격을 주행 허가로 승격하지 않는다.
    root, views = build_fixture((OFFICIAL/'world.sdf').read_bytes(), rows, vehicle, camera, actor)
    ET.indent(root)
    ET.ElementTree(root).write(output/'fixture.sdf', encoding='utf-8', xml_declaration=True)
    report['render_views'] = views
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in paths:
            archive.write(p, str(p.relative_to(ROOT)))
    (output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(dict(registry_matches_runtime=report['registry_matches_runtime'],
        subsets=report['geometry']['subsets'], visible_grids=sum(r['all_inside_image'] for r in report['official_light_views']))))


if __name__ == '__main__':
    main()
