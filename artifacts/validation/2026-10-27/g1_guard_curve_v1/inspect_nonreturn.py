"""선택된 거절 시점만 기록된 스캔/운동으로 복원해 공백의 주변 반환을 확인한다."""
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'src/arena_autonomy'))
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene


if __name__ == '__main__':
    rows = []
    wanted = {round(s['t'], 6) for s in json.loads((HERE/'run1_region_audit.json').read_text())['samples']}
    for line in (HERE/'run1_corner/follow_decisions.jsonl').open():
        e = json.loads(line)
        if e['kind'] == 'header':
            config = MotionConfig(**e['data']['motion_config'])
        if e['kind'] != 'call' or round(e['data']['input']['reference_s'], 6) not in wanted:
            continue
        data, scan, history = restore_call(e['data']['input'], config)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        neighborhood = []
        for i in range(435, 461):
            r = scene.rays[i]
            neighborhood.append({'index': i, 'status': r.status, 'raw_range_m': scan.ranges[i] if math.isfinite(scan.ranges[i]) else None,
                                 'angle_deg': math.degrees(math.atan2(r.direction_xy[1], r.direction_xy[0])),
                                 'hit': r.hit_xy_m})
        rows.append({'t': data['reference_s'], 'neighborhood': neighborhood})
    out = HERE/'run1_nonreturn_neighborhood.json'
    if out.exists():
        raise FileExistsError(out)
    out.write_text(json.dumps({'scope': 'recorded sensor evidence only, no free-space imputation', 'samples': rows}, indent=2)+'\n')
    print(json.dumps([{'t': r['t'], 'rays': [(p['index'], p['status'], p['raw_range_m']) for p in r['neighborhood']]} for r in rows]))
