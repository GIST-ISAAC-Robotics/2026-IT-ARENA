"""공식 칸과 고정 경로·차량 간 초기/일정 겹침을 평가 전용으로 검사한다."""
import json
from pathlib import Path

from g1_traffic_eval import rectangle
import validate_g1_traffic as ev
from traffic_actors import overlay_case, actor_specs, check_additional_routes

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
WORLD=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
cfg=ev.load_config(HERE/'new_track_course.json')
case=overlay_case(cfg['cases'][0],HERE/'additional_actors.json')
specs=actor_specs(cfg,case)
line,scene=ev.load_centerline(WORLD)
road=ev.road_polygon(WORLD/'world.sdf')
walls=ev.obstacle_polygons(WORLD/'world.sdf')
s0=ev.actor_start_s(cfg,scene,0,case)
primary=ev.check_route(cfg,line,case,s0,road,walls)
starts,other=check_additional_routes(cfg,case,specs,scene,0,line,road,walls,ev.actor_start_s,ev.check_route,ev.actor_pose_at)
slots={s['index']:s for s in scene['starting_grid']['slots']}
slot=slots[3]
ego=rectangle(slot['x'],slot['y'],slot['yaw_rad'],.20,.15)
placement={}
for actor,start,slot_index in zip(specs,starts,(4,0,1,2)):
    target=slots[slot_index]
    pose=ev.actor_pose_at(line,start,actor['schedule'],0.,actor['lateral_offset_m'])
    assert abs(pose[0]-target['x'])<1e-6 and abs(pose[1]-target['y'])<1e-6
    body=rectangle(*pose,actor['length_m'],actor['width_m'])
    assert ego.intersection(body).area<1e-9 and body.difference(road).area<1e-6
    placement[actor['model_name']]=dict(slot=slot_index,pose=pose,initial_ego_gap_m=ego.distance(body),
                                      height_m=actor['height_m'])
out=dict(ego_slot=3,actor_count=len(specs),placements=placement,primary_route=primary,extra_routes=other)
(HERE/'geometry.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(out,ensure_ascii=False))
