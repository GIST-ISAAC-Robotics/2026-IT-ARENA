"""정지 직전 현재 반대 벽과 유효 경로 이력의 연결 조건을 같은 입력에서 확인한다."""
import json
import math
from pathlib import Path
import sys

import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, replay_reset, restore_call, freeze
from arena_autonomy.scene_motion import motion_scene, transport_points
from arena_autonomy.local_path import local_path
from arena_autonomy.local_follow import path_agrees, path_agrees_arc
from arena_autonomy.numeric_kernels import configure


def main():
    with (A.parent/'old610_drive_r328_v1/run1/follow_decisions.jsonl').open() as f:
        header = json.loads(next(f))['data']
        events = [json.loads(line) for line in f]
    configure(header.get('numeric_execution', 'python'))
    follow, motion = make_controller(header)
    cfg = follow.config
    rows = []
    for event in events:
        if event['kind'] == 'reset':
            replay_reset(follow, event['data'])
        if event['kind'] != 'call':
            continue
        call = event['data']
        now = call['input']['reference_s']
        if now > 31.35:
            break
        if now >= 30.7:
            info, scan, history = restore_call(call['input'], motion)
            scene = motion_scene(scan, history, now, timing_verified=info['timing_verified'],
                                 epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
            if follow.last_source != scene.source_first_s:
                points = np.asarray(scene.occupied_endpoints_xy_m)
                rays = [r.source_index for r in scene.rays if r.status == 'return']
                age = now-scene.source_first_s
                v = cfg.open_route_speed_mps
                horizon = max(1.8, .06+cfg.standstill_gap_m+v*(age+.1)+v**2/4)
                p, d = local_path(points, 'right', cfg.wall_offset_m, horizon=horizon,
                    rays=rays if cfg.ray_wall_continuity else None,
                    piecewise_corner=cfg.piecewise_wall_trial,
                    minimum_radius=cfg.wheelbase_m/math.tan(cfg.max_steering_rad),
                    narrow_occluder_bridge=cfg.narrow_occluder_bridge)
                carried = None if follow.path is None else transport_points(follow.path, history, follow.path_ref, now)
                row = dict(time=now, old_source=follow.path_source, old_wall=follow.path_wall,
                           model=d.get('wall_model'), ahead=d.get('boundary_observed_ahead_m'),
                           anchor=d.get('path_anchor_error_m'),
                           shared_x=bool(p is not None and carried is not None and path_agrees(p, carried, cfg.boundary_agreement_m)),
                           shared_arc=bool(p is not None and carried is not None and path_agrees_arc(p, carried, cfg.boundary_agreement_m)))
                if p is not None and carried is not None:
                    row['candidate'] = p.tolist()
                    row['carried'] = carried.tolist()
                rows.append(row)
        output = freeze(replay_call(follow, call['input'], motion))
        if rows and rows[-1]['time'] == now:
            rows[-1]['speed'] = output[0]
            rows[-1]['reason'] = output[2]['reason']
            rows[-1]['selection'] = output[2].get('boundary_selection')
    (A/'timeline.json').write_text(json.dumps(rows, indent=2)+'\n')
    print(json.dumps([{k: v for k, v in row.items() if k not in ('candidate', 'carried')} for row in rows], indent=2))


if __name__ == '__main__':
    main()
