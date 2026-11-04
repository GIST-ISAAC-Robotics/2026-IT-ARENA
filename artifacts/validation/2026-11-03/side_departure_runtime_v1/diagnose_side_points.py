"""최초 재정지와 마지막 정지의 실제 반환 좌표. 제어 입력을 바꾸지 않는다."""
import json
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'build/side_departure_runtime_v1'))
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.occupancy_timing import per_ray_padding


def main():
    chosen = []
    for line in (HERE/'run1/follow_decisions.jsonl').open(encoding='utf-8'):
        e = json.loads(line); d = e['data']
        if e['kind'] == 'header':
            motion = MotionConfig(**d['motion_config']); cfg = d['follow_config']
        if e['kind'] != 'call':
            continue
        normal = d['output'][2].get('normal') or d['output'][2]
        dep = (normal.get('parallel_hold_trial') or {}).get('side_departure') or {}
        if dep.get('reason') == 'overlap_not_only_side_timing':
            chosen = [chosen[0], e] if chosen else [e]
    result = []
    for e in chosen:
        raw, scan, history = restore_call(e['data']['input'], motion)
        scene = motion_scene(scan, history, raw['reference_s'], timing_verified=raw['timing_verified'],
                             epoch=raw['epoch'], rear_half_angle_deg=raw['rear_half_angle_deg'])
        points = np.asarray(scene.occupied_endpoints_xy_m)
        fixed = .025+cfg['measurement_padding_m']
        padding = per_ray_padding(scene, fixed, cfg['object_motion_bound_mps'])
        overlap = ((abs(points[:, 0]) <= cfg['length_m']/2+padding)
                   & (abs(points[:, 1]) <= cfg['width_m']/2+padding))
        side = overlap & (abs(points[:, 0]) <= cfg['length_m']/2+.025) & (abs(points[:, 1]) > cfg['width_m']/2+fixed)
        bad = points[overlap & ~side]
        result.append(dict(seq=e['seq'], reference_s=raw['reference_s'], rejected_points=bad.tolist(),
                           all_behind_side_window=bool(len(bad)) and bool(np.all((bad[:, 0] < -cfg['length_m']/2-.025)
                               & (abs(bad[:, 1]) > cfg['width_m']/2+fixed)))))
    (HERE/'run1_side_points.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
