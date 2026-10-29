"""고정된 후반 입력에서 양쪽 벽/짧은 경로의 거절 사유를 분리한다. 출력 연결 없음."""
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_corridor import corridor_evidence, short_prefix_evidence
from arena_autonomy.corridor_observer import current_proposal
BASE = Path(__file__).resolve().parent
out = BASE/'boundary_hold_geometry.json'
if out.exists():
    raise FileExistsError(out)
wanted, results = (129.4, 129.5, 129.74, 130., 136.), []
for line in (BASE/'late_hold_prefix.jsonl').open(encoding='utf-8'):
    row = json.loads(line)
    if row['kind'] == 'header':
        config = FollowConfig(**row['data']['follow_config'])
        motion = MotionConfig(**row['data']['motion_config'])
    if row['kind'] != 'call' or not any(abs(row['data']['input']['reference_s']-t) < 1e-7 for t in wanted):
        continue
    call = row['data']
    data, scan, history = restore_call(call['input'], motion)
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                         epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
    result = dict(reference_s=data['reference_s'], side=data['side'], speed=data['speed_mps'],
                  previous_steering=data['previous_steering_rad'], original_command=call['output'][:2],
                  original_reason=call['output'][2]['reason'], candidates={})
    for side in ('left','right'):
        path, info = local_path(points, side, config.wall_offset_m)
        result['candidates'][side] = dict(local_info=info, path=None if path is None else path.tolist(),
            corridor=corridor_evidence(points,side,config), short_prefix=short_prefix_evidence(points,side,config),
            current_proposal=current_proposal(scene,data['speed_mps'],side,data['previous_steering_rad'],
                                             data['dt_s'],config,partial_candidate=True))
    if abs(data['reference_s']-130.) < 1e-7:
        result['points'] = points.tolist()
    results.append(result)
report = dict(scope='current-frame geometry only; opposite-side comparison is not a steering decision',
              runtime_changed=False, results=results)
out.write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps([dict(t=r['reference_s'],original=r['original_reason'],
    candidates={s:dict(info=v['local_info'],corridor_reason=v['corridor']['reason'],
                       prefix_reason=v['short_prefix']['reason'],proposal=v['current_proposal']['reason'])
                for s,v in r['candidates'].items()}) for r in results],ensure_ascii=False))
