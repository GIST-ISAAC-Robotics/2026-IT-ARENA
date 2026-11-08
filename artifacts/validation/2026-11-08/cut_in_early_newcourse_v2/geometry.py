"""새 코스에서 기존 합류 각본의 벽·도로/잔디·조향 표본을 검사한다. 평가 전용."""
import json
from pathlib import Path
import validate_g1_traffic as ev
A=Path(__file__).resolve().parent;R=A.parents[3];world=R/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
cfg=ev.load_config(A/'new_track_course.json');case=cfg['cases'][0]
line,scene=ev.load_centerline(world);s0=ev.actor_start_s(cfg,scene,0,case)
route=ev.check_route(cfg,line,case,s0,ev.road_polygon(world/'world.sdf'),ev.obstacle_polygons(world/'world.sdf'))
(A/'geometry.json').write_text(json.dumps(route,indent=2)+'\n');print(json.dumps(route))
