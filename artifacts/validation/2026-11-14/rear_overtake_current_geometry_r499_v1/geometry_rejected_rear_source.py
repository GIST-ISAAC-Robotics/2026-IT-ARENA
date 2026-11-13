"""현재 CAD/시설의 원 피추월 경로 정적 대조. Windows, ROS 초기화·실행 없음."""
import ast
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
from shapely.affinity import rotate, translate

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
PARENT = A.parent / 'traffic_current_geometry_r496_v1'
ORIGINAL = ROOT / 'artifacts/validation/2026-11-08/rear_overtake_measured_v1'
sys.path[:0] = [str(ROOT / 'scripts'), str(PARENT)]
import g1_traffic_eval as ev
from audit_local_pursuit_road import road_polygon
from build_experimental_track import rectangle, obstacle_polygons
from traffic_road_route import check_onroad_route
from facility_geometry import load, slab_projection

read = lambda p: json.loads(p.read_bytes())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    world = ROOT / 'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
    current = read(PARENT / 'config.json')
    original = read(ORIGINAL / 'new_track_course.json')
    case = original['cases'][0]
    cfg = dict(current, cases=[case])
    cfg['actor'] = dict(current['actor'], wheelbase_m=original['actor']['wheelbase_m'],
                        track_m=original['actor']['track_m'])
    source_path = ROOT / 'scripts/validate_g1_traffic.py'
    selected = {'load_centerline', 'actor_start_s', 'check_straight', 'actor_pose_at'}
    source = ast.parse(source_path.read_bytes())
    functions = [n for n in source.body if isinstance(n, ast.FunctionDef) and n.name in selected]
    assert len(functions) == len(selected)
    # 기존 순수 함수 본문 그대로 추출: ROS 평가기 모듈 import만 피한다.
    space = dict(np=np, json=json, math=math, ev=ev, DEMO_WORLD=world)
    exec(compile(ast.Module(body=functions, type_ignores=[]), str(source_path), 'exec'), space)
    line, scene = space['load_centerline'](world)
    s0 = space['actor_start_s'](cfg, scene, 0, case)
    road = road_polygon(world / 'world.sdf')
    walls = obstacle_polygons(world / 'world.sdf')
    fixture = ROOT / 'artifacts/validation/2026-11-13/old_signal_610_r326_v1'
    fixture_cfg = read(fixture / 'fixture.json')
    mesh = fixture / 'support_frame_m.stl'
    assert sha(mesh) == fixture_cfg['mesh_sha256']
    _, _, triangles, _ = load(mesh)
    # 상대 충돌 상자 전체 높이와 지주 겹침을 확인한다. 자차 하부 slab로 대신하지 않는다.
    height_shape = slab_projection(triangles, cfg['actor']['bottom_m'],
                                   cfg['actor']['bottom_m'] + cfg['actor']['height_m'])
    facilities = [translate(rotate(height_shape, p['yaw'], origin=(0, 0), use_radians=True),
                            p['x'], p['y']) for p in fixture_cfg['locations']]
    route = check_onroad_route(cfg, line, case, s0, road, walls,
                               space['check_straight'], space['actor_pose_at'])
    travel = ev.actor_travel_m(case['schedule'], case['duration_s'])
    steps = max(1, math.ceil(travel / .01))
    collisions = []
    min_facility_gap = math.inf
    max_body_outside = 0.
    for i in range(steps + 1):
        along = s0 + travel * i / steps
        pose = space['actor_pose_at'](line, along, [], 0., case['actor_lateral_offset_m'])
        body = rectangle(*pose, cfg['actor']['length_m'], cfg['actor']['width_m'])
        max_body_outside = max(max_body_outside, body.difference(road).area)
        for index, shape in enumerate(facilities):
            area = body.intersection(shape).area
            min_facility_gap = min(min_facility_gap, body.distance(shape))
            if area > 1e-6:
                collisions.append(dict(sample=i, along_s_m=along, facility_index=index,
                                       overlap_area_m2=area, actor_pose=pose))
    slot = next(s for s in scene['starting_grid']['slots'] if s['index'] == cfg['launch']['grid_slot'])
    vehicle_path = ROOT / 'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
    resolved_vehicle = read(PARENT / 'run1/report.json')['vehicle_config']
    assert sha(vehicle_path) == resolved_vehicle['sha256']
    geometry = resolved_vehicle['ego_geometry']
    ego = rectangle(slot['x'], slot['y'], slot['yaw_rad'], geometry['length_m'], geometry['width_m'])
    first_actor = rectangle(*space['actor_pose_at'](line, s0, [], 0., case['actor_lateral_offset_m']),
                            cfg['actor']['length_m'], cfg['actor']['width_m'])
    initial_gap = ego.distance(first_actor)
    result = dict(passed=not collisions and initial_gap >= .08, original_case=case,
                  route=route, actor_height_m=cfg['actor']['height_m'], original_actor_height_m=original['actor']['height_m'],
                  actor_travel_m=travel, actor_path_samples=steps + 1,
                  maximum_sample_step_m=travel / steps, initial_gap_m=initial_gap,
                  initial_overlap_m2=ego.intersection(first_actor).area,
                  facility_count=len(facilities), actor_full_height_facility_gap_m=min_facility_gap,
                  facility_collision_sample_count=len(collisions), collisions=collisions,
                  actor_max_outside_body_area_m2=max_body_outside, current_ego_geometry=geometry,
                  reused_pure_function_source_sha256=sha(source_path),
                  source_hashes={str(p.relative_to(ROOT)).replace('\\', '/'):sha(p) for p in
                    (ORIGINAL / 'new_track_course.json', PARENT / 'config.json', vehicle_path,
                     world / 'world.sdf', world / 'centerline.csv', world / 'scene.json', mesh)},
                  scope='static sampled original actor route versus current track and full-height facility slab; no closed-loop or hardware claim',
                  platform=sys.platform, static_calculation_only=True, simulator_started=False)
    out = A / 'geometry.json'
    with out.open('x', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
        f.write('\n')
    print(json.dumps({k:result[k] for k in ('passed','actor_travel_m','actor_path_samples','initial_gap_m',
          'actor_full_height_facility_gap_m','facility_collision_sample_count')}, ensure_ascii=False))


if __name__ == '__main__':
    main()
