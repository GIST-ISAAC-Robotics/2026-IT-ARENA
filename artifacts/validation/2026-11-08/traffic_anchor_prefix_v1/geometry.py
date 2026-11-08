"""두 배우의 등록 코너 구간 경로·표본 겹침·자차 초기 간격. 지도는 평가 전용."""
import json
from pathlib import Path
import validate_g1_traffic as ev
from traffic_actors import overlay_case,actor_specs,check_additional_routes
from g1_traffic_eval import rectangle

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
cfg=ev.load_config(HERE/'new_track_course.json');case=overlay_case(cfg['cases'][0],HERE/'additional_actors.json')
assert case['name']=='two_actor_corner_prefix' and case['route_kind']=='first_curve' and case['duration_s']==44.
specs=actor_specs(cfg,case);line,scene=ev.load_centerline(world)
road=ev.road_polygon(world/'world.sdf');walls=ev.obstacle_polygons(world/'world.sdf')
s0=ev.actor_start_s(cfg,scene,0,case)
primary=ev.check_route(cfg,line,case,s0,road,walls)
starts,others=check_additional_routes(cfg,case,specs,scene,0,line,road,walls,ev.actor_start_s,ev.check_route,ev.actor_pose_at)
# 추가 actor도 같은 case 이름의 공통 규칙(first_curve 출구/한계)으로 검사됐는지 확인한다.
extra=others['actors']['eval_second_lead']
assert primary['kind']==extra['kind']=='first_curve'
close=lambda a,b:abs(a-b)<1e-6
assert close(primary['actor_start_s_m'],43.26) and close(primary['actor_end_s_m'],65.26)
assert close(extra['actor_start_s_m'],49.26) and close(extra['actor_end_s_m'],71.26)
assert close(line.lap,44.64)
for r in (primary,extra):
    assert close(r['required_exit_s_m'],18.5+line.lap) and close(r['maximum_actor_s_m'],27.+line.lap)
slot=next(s for s in scene['starting_grid']['slots'] if s['index']==3)
ego=rectangle(slot['x'],slot['y'],slot['yaw_rad'],.20,.15)
initial={a['model_name']:ego.distance(rectangle(*ev.actor_pose_at(line,s,[],0.,0.),.20,.15)) for a,s in zip(specs,starts)}
assert all(gap>=.08 for gap in initial.values())
result=dict(primary=primary,additional=others,initial_gaps_m=initial,ego_grid_s_m=slot['s_m'],
    wrapped_end_s_m={a['model_name']:(s+.5*44.)%line.lap for a,s in zip(specs,starts)},
    common_case_rules_applied_to_additional=True,scope='sampled route, not closed-loop result')
(HERE/'geometry.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8');print(json.dumps(result))
