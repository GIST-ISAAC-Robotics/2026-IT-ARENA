"""설명용 사후 검산: 무반환 방향의 벽 교차가 라이다 최대거리 밖인지 확인. 제어 입력이 아니다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for p in ('scripts', 'src/arena_autonomy', 'src/arena_vehicle_interface'):
    sys.path.insert(0, str(ROOT/p))
from probe_grid_follow import wall_sections, RUNTIME
from probe_corner_visibility_grid import cast_segments
from arena_autonomy.follow_trace import thaw

run = HERE/'run1_corner'
world = RUNTIME/'world.sdf'
digest = hashlib.sha256(world.read_bytes()).hexdigest()
manifest = json.loads((HERE/'run1_source_manifest.json').read_text())
expected = next(r['sha256'] for r in manifest if r['path'] == world.relative_to(ROOT).as_posix())
if expected != digest:
    raise ValueError('world differs from run1 snapshot')
walls = wall_sections(world.read_bytes(), .095)
poses = np.asarray([json.loads(l) for l in (run/'ego_poses.jsonl').open()])
poses = poses[np.r_[True, np.diff(poses[:, 0]) > 0]]
for line in (run/'follow_decisions.jsonl').open():
    e = json.loads(line)
    if e['kind'] == 'header':
        motion = e['data']['motion_config']
    if e['kind'] == 'call' and abs(e['data']['input']['reference_s']-32.) < 1e-7:
        data = thaw(e['data']['input'])
        break
raw = data['scan']
rows = []
for i in range(448, 457):
    t = raw['sec']+raw['nanosec']*1e-9+i*raw['time_increment']
    x, y = (float(np.interp(t, poses[:, 0], poses[:, j])) for j in (1, 2))
    yaw = float(np.interp(t, poses[:, 0], np.unwrap(poses[:, 3])))
    sx, sy = motion['lidar_x_m'], motion['lidar_y_m']
    origin = (x+math.cos(yaw)*sx-math.sin(yaw)*sy, y+math.sin(yaw)*sx+math.cos(yaw)*sy)
    angle = raw['angle_min']+i*raw['angle_increment']+motion['lidar_yaw_rad']+yaw
    distance = float(cast_segments(walls, origin, np.array([angle]))[0])
    observed = raw['ranges'][i]
    rows.append({'index': i, 'acquired_s': t, 'recorded_m': observed if math.isfinite(observed) else None,
                 'wall_planar_distance_m': distance if math.isfinite(distance) else None,
                 'beyond_range_max': distance > raw['range_max']})
result = {'scope': 'evaluation-only planar wall cross-check; ground truth is not sent to controller; not a full Gazebo renderer reconstruction',
          'world_unchanged': True, 'world_sha256': digest, 'range_max_m': raw['range_max'], 'rays': rows}
out = HERE/'run1_wall_ray_check.json'
if out.exists():
    raise FileExistsError(out)
out.write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result))
