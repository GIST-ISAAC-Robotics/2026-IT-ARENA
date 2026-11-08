"""중앙 배우18초 경로의 외곽을1cm 간격으로 새 도로/벽과 대조한다. 평가 전용."""
import json
import math
from pathlib import Path
import validate_g1_traffic as ev
from g1_traffic_eval import rectangle,actor_travel_m

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
cfg=ev.load_config(HERE/'new_track_course.json'); case=cfg['cases'][0]
line,scene=ev.load_centerline(world)
s0=ev.actor_start_s(cfg,scene,0,case)
road=ev.road_polygon(world/'world.sdf'); walls=ev.obstacle_polygons(world/'world.sdf')
route=ev.check_route(cfg,line,case,s0,road,walls)
travel=actor_travel_m(case['schedule'],case['duration_s'])
steps=max(1,math.ceil(travel/.01))
footprints=[rectangle(*ev.actor_pose_at(line,s0+travel*i/steps,[],0.,case['actor_lateral_offset_m']),cfg['actor']['length_m'],cfg['actor']['width_m']) for i in range(steps+1)]
slot=next(x for x in scene['starting_grid']['slots'] if int(x['index'])==cfg['launch']['grid_slot'])
ego=rectangle(slot['x'],slot['y'],slot['yaw_rad'],.20,.15)
result=dict(initial_actor_ego_gap_m=ego.distance(footprints[0]),initial_actor_ego_overlap_m2=ego.intersection(footprints[0]).area,route=route,body_outside_area_m2=max(car.difference(road).area for car in footprints),
            wall_overlap_area_m2=max(sum(car.intersection(w).area for w in walls) for car in footprints),
            road_edge_distance_m=min(car.distance(road.boundary) for car in footprints),
            actor_road_wall_samples=len(footprints),maximum_sample_step_m=travel/steps)
(HERE/'geometry.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
assert result['body_outside_area_m2']<1e-6 and result['wall_overlap_area_m2']<1e-6

assert result['initial_actor_ego_gap_m']>=.08 and result['initial_actor_ego_overlap_m2']<1e-9
