"""등록 전 실제 코스·평가 좌표와 새 래퍼의 형식을 확인한다."""
import ast,json,math
from pathlib import Path
from race_ops_eval import Course
from g1_traffic_eval import rectangle
from audit_local_pursuit_road import road_polygon
A=Path(__file__).resolve().parent;R=A.parents[3];world=R/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
c=Course(world);road=road_polygon(world/'world.sdf')
poses={s:c.pose(s) for s in (41.66,42.26,43.26,44.)}
for p in poses.values():assert rectangle(*p,.2,.15).difference(road).area<1e-6
for name in ('run.py','register.py','assess.py','settle.py'):ast.parse((A/name).read_text(encoding='utf-8'))
assert math.dist(poses[41.66][:2],poses[43.26][:2])>1.59
(A/'preflight_review.json').write_text(json.dumps(dict(poses=poses,initial_center_gap_m=math.dist(poses[41.66][:2],poses[43.26][:2]),scope='static geometry and syntax only; live NOT run'),indent=2)+'\n')
print('geometry/syntax checked')
