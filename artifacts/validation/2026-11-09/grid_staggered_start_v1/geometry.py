"""공식 칸·12초 경로·상대끼리 일정 간격을 평가 전용으로 사전 검사한다. 제어기 입력이 아니다."""
import itertools
import json
import math
from pathlib import Path

from g1_traffic_eval import rectangle, actor_travel_m
import validate_g1_traffic as ev
from traffic_actors import overlay_case, actor_specs, check_additional_routes

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
CASE = 'five_vehicle_grid_staggered'
SLOTS = {'eval_g1b_actor': 4, 'eval_grid_0': 0, 'eval_grid_1': 1, 'eval_grid_2': 2}
STARTS = {'eval_grid_2': .20, 'eval_grid_1': .80, 'eval_grid_0': 1.40, 'eval_g1b_actor': 2.00}


def main():
    cfg = ev.load_config(HERE/'new_track_course.json')
    case = overlay_case(ev.case_named(cfg, CASE), HERE/'additional_actors.json')
    specs = actor_specs(cfg, case)
    line, scene = ev.load_centerline(WORLD)
    road = ev.road_polygon(WORLD/'world.sdf')
    walls = ev.obstacle_polygons(WORLD/'world.sdf')
    s0 = ev.actor_start_s(cfg, scene, 0, case)
    primary = ev.check_route(cfg, line, case, s0, road, walls)
    starts, other = check_additional_routes(cfg, case, specs, scene, 0, line, road, walls,
                                            ev.actor_start_s, ev.check_route, ev.actor_pose_at)
    slots = {s['index']: s for s in scene['starting_grid']['slots']}
    ego = rectangle(slots[3]['x'], slots[3]['y'], slots[3]['yaw_rad'], .20, .15)
    placement = {}
    for actor, start in zip(specs, starts):
        name = actor['model_name']
        target = slots[SLOTS[name]]
        pose = ev.actor_pose_at(line, start, actor['schedule'], 0., actor['lateral_offset_m'])
        assert abs(pose[0]-target['x']) < 1e-6 and abs(pose[1]-target['y']) < 1e-6, name
        body = rectangle(*pose, actor['length_m'], actor['width_m'])
        assert ego.intersection(body).area < 1e-9 and body.difference(road).area < 1e-6, name
        moving = [r for r in actor['schedule'] if r['speed_mps'] > 0]
        assert len(moving) == 1 and abs(moving[0]['from_s']-STARTS[name]) < 1e-9 and moving[0]['speed_mps'] == .5, name
        assert actor['height_m'] == .18 and (actor['length_m'], actor['width_m']) == (.2, .15), name
        placement[name] = dict(slot=SLOTS[name], pose=pose, initial_ego_gap_m=ego.distance(body),
                               height_m=actor['height_m'], scheduled_start_s=STARTS[name],
                               travel_m=actor_travel_m(actor['schedule'], case['duration_s']))
    # 상대끼리 쌍별 최소 간격(10 ms 표본). check_additional_routes의 겹침 거절과 별도로 쌍을 남긴다.
    count = math.ceil(case['duration_s']/.01)
    pairs = {f'{a["model_name"]}|{b["model_name"]}': math.inf for a, b in itertools.combinations(specs, 2)}
    ego_static = {a['model_name']: math.inf for a in specs}
    for i in range(count+1):
        t = case['duration_s']*i/count
        bodies = {a['model_name']: rectangle(*ev.actor_pose_at(line, s, a['schedule'], t, a['lateral_offset_m']),
                                             a['length_m'], a['width_m']) for a, s in zip(specs, starts)}
        for key in pairs:
            first, second = key.split('|')
            pairs[key] = min(pairs[key], bodies[first].distance(bodies[second]))
        for name, body in bodies.items():
            ego_static[name] = min(ego_static[name], ego.distance(body))
    order = sorted(STARTS, key=STARTS.get)
    out = dict(case=CASE, ego_slot=3, actor_count=len(specs), duration_s=case['duration_s'], start_order=order,
               placements=placement, primary_route=primary, extra_routes=other,
               pair_minimum_gap_m=pairs, minimum_pair_gap_m=min(pairs.values()),
               ego_at_slot_minimum_gap_m=ego_static,
               scope='registered schedule 10 ms samples; not continuous collision proof; ego motion not modeled')
    assert out['minimum_pair_gap_m'] > 0
    (HERE/'geometry.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(dict(ok=True, pairs=pairs, order=order,
                          ends={k: v['actor_end_s_m'] for k, v in other['actors'].items()}), ensure_ascii=False))


if __name__ == '__main__':
    main()
