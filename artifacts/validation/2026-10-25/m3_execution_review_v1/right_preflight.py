"""등록된 오른쪽 자극의 정적 검사. 주행·보호·간격 판정 변경 없음."""
import hashlib
import json
from pathlib import Path
import sys

import validate_g1_traffic as runner
from traffic_actors import overlay_case, actor_specs, check_additional_routes
from g1_traffic_eval import rectangle


def main():
    out = Path(sys.argv[1])
    if out.exists():
        raise FileExistsError(out)
    cfg_path = Path('config/tests/e3_m3_right_grid_v1.json')
    overlay_path = Path('config/tests/e3_onroad_right_encounter_v1.json')
    cfg = runner.load_config(cfg_path)
    parent = runner.load_config()
    for key in parent:
        if key in ('registered_at', 'scope', 'launch'):
            continue
        assert cfg[key] == parent[key], key
    assert {k:v for k,v in cfg['launch'].items() if k != 'grid_slot'} == {
        k:v for k,v in parent['launch'].items() if k != 'grid_slot'}
    assert cfg['launch']['grid_slot'] == 0
    meta = cfg['parent_config']
    assert hashlib.sha256(Path(meta['path']).read_bytes()).hexdigest() == meta['sha256']
    overlay = json.loads(overlay_path.read_text())
    original = json.loads(Path(overlay['parent_overlay']['path']).read_text())
    assert hashlib.sha256(Path(overlay['parent_overlay']['path']).read_bytes()).hexdigest() == overlay['parent_overlay']['sha256']
    a, b = dict(overlay['additional_actors'][0]), dict(original['additional_actors'][0])
    assert a.pop('lateral_offset_m') == -b.pop('lateral_offset_m') == -.13
    a.pop('gap_basis'); b.pop('gap_basis')
    assert a == b
    case = overlay_case(runner.case_named(cfg, 'slow_lead'), overlay_path)
    specs = actor_specs(cfg, case)
    line, scene = runner.load_centerline()
    road = runner.road_polygon(runner.DEMO_WORLD/'world.sdf')
    obstacles = runner.obstacle_polygons(runner.DEMO_WORLD/'world.sdf')
    starts, routes = check_additional_routes(cfg, case, specs, scene, 0, line, road, obstacles,
        runner.actor_start_s, runner.check_route, runner.actor_pose_at)
    slot = next(x for x in scene['starting_grid']['slots'] if int(x['index']) == 0)
    cx, cy, heading = line.pose(slot['s_m'])
    import math
    lateral = -(slot['x']-cx)*math.sin(heading)+(slot['y']-cy)*math.cos(heading)
    ego = rectangle(slot['x'], slot['y'], slot['yaw_rad'], .20, .15)
    assert ego.difference(road).area < 1e-6
    assert .085 < lateral < .095
    assert routes['actors']['eval_e3_side']['minimum_sampled_road_edge_distance_m'] > .01
    result = dict(scope='static right-side geometry only; not a drive or a new acceptance threshold',
        config_sha256=hashlib.sha256(cfg_path.read_bytes()).hexdigest(),
        overlay_sha256=hashlib.sha256(overlay_path.read_bytes()).hexdigest(),
        config_parent_preserved=True, overlay_parent_preserved=True, grid_slot=0,
        grid_lateral_m=lateral, nominal_parallel_gap_m=lateral+.13-.15,
        ego_initial_road_outside_area_m2=ego.difference(road).area, routes=routes,
        caveat='slot 0 is 0.20 m further ahead than slot 3; scan acquisition order and left preferred wall unchanged')
    out.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
