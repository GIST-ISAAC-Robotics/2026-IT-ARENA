"""두 배우의 전체 경로·표본 겹침·자차 초기 간격. 지도는 평가 전용."""
import json
from pathlib import Path
import validate_g1_traffic as ev
from traffic_actors import overlay_case,actor_specs,check_additional_routes
from g1_traffic_eval import rectangle

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
cfg=ev.load_config(HERE/'new_track_course.json');case=overlay_case(cfg['cases'][0],HERE/'additional_actors.json')
specs=actor_specs(cfg,case);line,scene=ev.load_centerline(world)
road=ev.road_polygon(world/'world.sdf');walls=ev.obstacle_polygons(world/'world.sdf')
s0=ev.actor_start_s(cfg,scene,0,case)
primary=ev.check_route(cfg,line,case,s0,road,walls)
starts,others=check_additional_routes(cfg,case,specs,scene,0,line,road,walls,ev.actor_start_s,ev.check_route,ev.actor_pose_at)
slot=next(s for s in scene['starting_grid']['slots'] if s['index']==3)
ego=rectangle(slot['x'],slot['y'],slot['yaw_rad'],.20,.15)
initial={a['model_name']:ego.distance(rectangle(*ev.actor_pose_at(line,s,[],0.,0.),.20,.15)) for a,s in zip(specs,starts)}
assert all(gap>=.08 for gap in initial.values())
result=dict(primary=primary,additional=others,initial_gaps_m=initial,scope='sampled route, not closed-loop result')
(HERE/'geometry.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8');print(json.dumps(result))
