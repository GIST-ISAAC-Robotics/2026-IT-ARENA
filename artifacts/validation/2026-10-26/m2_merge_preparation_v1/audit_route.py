"""고정 전방 합류 경로의 공식 도로 감사. 제어 입력·Gazebo 없음."""
import hashlib
import json
from pathlib import Path
import sys

import validate_g1_traffic as runner
from traffic_merge import validate_merge_route, scheduled_merge_pose

root = runner.REPO
config_path = root/'config/tests/m2_merge_stop_restart_v1.json'
config = json.loads(config_path.read_text(encoding='utf-8'))
case = config['cases'][0]
line, scene = runner.load_centerline()
start = runner.actor_start_s(config, scene, 0, case)
result = validate_merge_route(config, line, case, start,
    runner.road_polygon(runner.DEMO_WORLD/'world.sdf'),
    runner.obstacle_polygons(runner.DEMO_WORLD/'world.sdf'), runner.check_straight)
result['config_sha256'] = hashlib.sha256(config_path.read_bytes()).hexdigest()
result['scope'] += '; initial lateral body spans overlap ego width: not a complete cut-in from a separate lane'
out = Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
(out/'route_audit.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
print(json.dumps(result, indent=2, ensure_ascii=False))
