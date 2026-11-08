"""새 증속 주행의 첫/마지막 지속 정지에서 미관측 방향만 조사. 출력 변경 없음."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.known_structure import classify,parse_sectors

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
SOURCE=ROOT/'artifacts/validation/2026-11-07/solo_straight_newcourse_v1/run1/follow_decisions.jsonl'
targets=[]
with SOURCE.open(encoding='utf8') as f:
    header=json.loads(next(f))['data']
    for l in f:
        q=json.loads(l)
        if q.get('kind')=='call' and q['data']['output'][2].get('reason')=='follow_lateral_metric_outside_trial':
            targets.append(q['data'])
rows=[]
for q in [targets[0],targets[-1]]:
    d,scan,h=restore_call(q['input'],MotionConfig(**header['motion_config']))
    scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],
                       epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    valid,excused,sectors=classify(scan,parse_sectors(header['follow_config']['known_structure_sectors_json']))
    angles=(scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment+h.config.lidar_yaw_rad+math.pi)%math.tau-math.pi
    missing=np.flatnonzero(~valid & ~excused & (abs(angles)<=math.radians(100)+1e-6))
    rays={r.source_index:r for r in scene.rays}
    rows.append(dict(t=d['reference_s'],speed=d['speed_mps'],previous_steer=d['previous_steering_rad'],
        output=q['output'][:2],partial=q['output'][2].get('partial_observation'),
        unknown=[dict(i=int(i),raw_range=freeze(scan.ranges[i]),angle_at_source_deg=math.degrees(angles[i]),
                      current_angle_deg=math.degrees(math.atan2(*rays[i].direction_xy[::-1])),
                      status=rays[i].status,origin_xy=rays[i].ray_origin_xy_m,
                      previous_range=freeze(scan.ranges[i-1]),next_range=freeze(scan.ranges[(i+1)%len(scan.ranges)]))
                 for i in missing]))
result=dict(scope='saved sensor evidence only, unchanged output',rows=rows,
            inputs={str(SOURCE.relative_to(ROOT)):hashlib.sha256(SOURCE.read_bytes()).hexdigest()})
(HERE/'diagnosis.json').write_text(json.dumps(freeze(result),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(freeze(result),ensure_ascii=False))

