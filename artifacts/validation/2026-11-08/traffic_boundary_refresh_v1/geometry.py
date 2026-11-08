"""사전 등록 도로/벽/초기 간격과 각본 연속성. Gazebo 실행은 하지 않는다."""
import json
import math
from pathlib import Path
import numpy as np
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons, rectangle
from validate_g1_traffic import load_config, load_centerline, actor_start_s, primary_pose_at, actor_pose_at, check_route

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
cfg = load_config(HERE/'new_track_course.json')
case = cfg['cases'][0]
line, scene = load_centerline(WORLD)
s0 = actor_start_s(cfg, scene, 0, case)
check = check_route(cfg, line, case, s0, road_polygon(WORLD/'world.sdf'), obstacle_polygons(WORLD/'world.sdf'))
slot = scene['starting_grid']['slots'][cfg['launch']['grid_slot']]
ego = rectangle(slot['x'], slot['y'], slot['yaw_rad'], .20, .15)
start = primary_pose_at(cfg, line, s0, case, 0.)
gap = ego.distance(rectangle(*start, .20, .15))
assert gap >= .08
times = np.linspace(0., case['duration_s'], int(case['duration_s']*100)+1)
rows = np.array([primary_pose_at(cfg, line, s0, case, t) for t in times])
old = np.array([actor_pose_at(line, s0, case['schedule'], t, case['actor_lateral_offset_m']) for t in times])
speed = np.linalg.norm(np.diff(rows[:, :2], axis=0), axis=1)/.01
yaw = np.abs(np.diff(np.unwrap(rows[:, 2])))/.01
result = dict(route=check, initial_ego_actor_gap_m=gap, actual_grid_slot=slot,
    time_samples=len(times), maximum_speed_mps=float(max(speed)), minimum_speed_mps=float(min(speed)),
    maximum_yaw_rate_rad_s=float(max(yaw)), maximum_old_offset_jump_speed_mps=float(max(np.linalg.norm(np.diff(old[:, :2], axis=0), axis=1)/.01)),
    scope='static scenario geometry, not actual parallel driving or vehicle dynamics')
(HERE/'geometry.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result, ensure_ascii=False))
