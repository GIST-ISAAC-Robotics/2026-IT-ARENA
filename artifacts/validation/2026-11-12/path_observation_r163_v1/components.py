"""현재 입력만으로 실제 무반환 성분의 이웃/이동 외곽 거리를 기록한다."""
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.rear_fringe_trial import _query_brackets
from diagnose import TRACE, A

events = [json.loads(s) for s in TRACE.open()]
event = next(e for e in events if e['seq'] == 632)
data, scan, history = restore_call(event['data']['input'], MotionConfig(**events[0]['data']['motion_config']))
scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
    epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
geometry = json.loads((A/'geometry632.json').read_text())
q = np.asarray(geometry['queries'])[~np.asarray(geometry['free']) & ~np.asarray(geometry['masked'])]
origins = np.asarray([r.ray_origin_xy_m for r in scene.rays])
dirs = np.asarray([r.direction_xy for r in scene.rays])
status = [r.status for r in scene.rays]
touch = set()
for _, _, lo, hi in _query_brackets(q, origins, dirs):
    touch.update(i for i in (lo, hi) if status[i] != 'return')
groups = []
covered = set()
for i in sorted(touch):
    if i in covered:
        continue
    start = end = i
    while start > 0 and status[start-1] != 'return':
        start -= 1
    while end+1 < len(status) and status[end+1] != 'return':
        end += 1
    covered.update(range(start, end+1))
    neighbors = []
    for n in (start-1, end+1):
        if 0 <= n < len(status) and status[n] == 'return':
            ray = scene.rays[n]
            neighbors.append(dict(index=n, direction_deg=math.degrees(math.atan2(*dirs[n, ::-1])),
                depth_m=math.dist(ray.ray_origin_xy_m, ray.hit_xy_m), hit=ray.hit_xy_m,
                closest_query_m=float(np.min(np.linalg.norm(q-np.asarray(ray.hit_xy_m), axis=1))),
                farthest_query_from_origin_m=float(np.max(np.linalg.norm(q-origins[n], axis=1)))))
    groups.append(dict(first=start, last=end, status=status[start:end+1],
        angles_deg=[math.degrees(math.atan2(*v[::-1])) for v in dirs[start:end+1]], neighbors=neighbors))
result = dict(seq=632, components=groups, unknown_queries=len(q),
    physical_exterior_m=float(np.max(abs(q[:,1])-geometry['body_width_m']/2)))
(A/'components632.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result))
