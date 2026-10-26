"""도로 안 옆 접근의 실행 전 기하. 차체 운동/회피 가능성을 증명하지 않음."""
import json
from pathlib import Path
import sys
import hashlib
import math

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import validate_g1_traffic as runner
from traffic_actors import actor_specs, overlay_case, check_additional_routes
from traffic_road_route import check_onroad_route


if __name__ == '__main__':
    config = runner.load_config()
    overlay = ROOT/'config/tests/e3_onroad_encounter_v1.json'
    case = overlay_case(runner.case_named(config, 'slow_lead'), overlay)
    specs = actor_specs(config, case)
    line, scene = runner.load_centerline()
    road = runner.road_polygon(runner.DEMO_WORLD/'world.sdf')
    obstacles = runner.obstacle_polygons(runner.DEMO_WORLD/'world.sdf')
    def route(cfg, line, case, start, road, obstacles):
        if case.get('route_kind') == 'side_road_straight':
            return check_onroad_route(cfg, line, case, start, road, obstacles, runner.check_straight, runner.actor_pose_at)
        return runner.check_route(cfg, line, case, start, road, obstacles)
    starts, checks = check_additional_routes(config, case, specs, scene, 0, line, road, obstacles,
        runner.actor_start_s, route, runner.actor_pose_at)
    slot = scene['starting_grid']['slots'][config['launch']['grid_slot']]
    ego = runner.rectangle(slot['x'], slot['y'], slot['yaw_rad'], .20, .15)
    side_pose = runner.actor_pose_at(line, starts[1], specs[1]['schedule'], 0., specs[1]['lateral_offset_m'])
    initial_gap = ego.distance(runner.rectangle(*side_pose, .20, .15))
    heading = line.pose(slot['s_m'])
    lateral = -(slot['x']-heading[0])*math.sin(heading[2])+(slot['y']-heading[1])*math.cos(heading[2])
    result = {'scope': 'preregistered geometric feasibility and threat; no controller or motion proof',
        'actor_routes': checks, 'initial_ego_actor_gap_m': initial_gap,
        'ego_initial_lateral_m': lateral, 'nominal_parallel_gap_at_start_line_m': .13-lateral-.15,
        'nominal_parallel_gap_if_ego_centers_m': .13-.15,
        'registered_gap_retained_m': .15,
        'sources': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in (Path(__file__), overlay, ROOT/'scripts/traffic_road_route.py')}}
    with (OUT/'report.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
