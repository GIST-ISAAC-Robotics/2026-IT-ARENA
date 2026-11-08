"""출발 중앙 정지 배우의 고정 외곽을 새 도로/벽과 대조한다. 평가 전용."""
import json
from pathlib import Path
import validate_g1_traffic as ev
from g1_traffic_eval import rectangle

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
cfg=ev.load_config(HERE/'new_track_course.json'); case=cfg['cases'][0]
line,scene=ev.load_centerline(world)
s0=ev.actor_start_s(cfg,scene,0,case)
road=ev.road_polygon(world/'world.sdf'); walls=ev.obstacle_polygons(world/'world.sdf')
route=ev.check_route(cfg,line,case,s0,road,walls)
car=rectangle(*line.pose(s0),cfg['actor']['length_m'],cfg['actor']['width_m'])
result=dict(route=route,body_outside_area_m2=car.difference(road).area,
            wall_overlap_area_m2=sum(car.intersection(w).area for w in walls),
            road_edge_distance_m=car.distance(road.boundary))
(HERE/'geometry.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
assert result['body_outside_area_m2']<1e-6 and result['wall_overlap_area_m2']<1e-6
