"""run32 입력에서 옆차 위협이 끊긴 장면의 측방 반환 조건별 개수(읽기 전용). 정답 자세는 사후 대조에만 쓴다."""
from bisect import bisect_right
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
    sys.path.insert(0, str(ROOT/part))

import numpy as np  # noqa: E402
import arena_autonomy.m3_execution_trial as mx  # noqa: E402
from arena_autonomy.follow_trace import restore_call  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig  # noqa: E402
from arena_autonomy.local_follow import FollowConfig, LocalFollow  # noqa: E402
from arena_autonomy.pass_release_trial import lateral_config  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402

RUN = ROOT/'artifacts/validation/2026-10-25/m3_execution_review_v1/run32_corner_after_threat/run'
CONFIG = ROOT/'artifacts/validation/2026-10-27/g1_pass_release_integration_v1/run4_left/follow_decisions.jsonl'
TIMES = (12.52, 12.62, 12.72, 12.82, 12.92, 20.92, 21.02, 21.12)


def main():
    cfg = FollowConfig(**json.loads(CONFIG.open(encoding='utf-8').readline())['data']['follow_config'])
    lat = lateral_config(cfg)
    ego = [json.loads(s) for s in (RUN/'ego_poses.jsonl').open(encoding='utf-8')]
    side = json.loads((RUN/'additional_actors.json').read_text(encoding='utf-8'))['poses']['eval_e3_side']
    et, st = [r[0] for r in ego], [r[0] for r in side]
    off = LocalFollow(FollowConfig(**{**vars(cfg), 'parallel_hold_trial': False, 'pass_release_trial': False}))
    motion, out = None, []
    with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                continue
            if event['kind'] == 'reset':
                off.reset()
                continue
            if event['kind'] != 'call':
                continue
            data, scan, history = restore_call(event['data']['input'], motion)
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                 epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            base = off.command(scene, history, data['speed_mps'], data['side'], data['previous_steering_rad'],
                               data['dt_s'], raw_scan=scan)
            if not any(abs(data['reference_s']-t) < 1e-6 for t in TIMES):
                continue
            path = np.asarray(base[2].get('path_points') or [], dtype=float).reshape(-1, 2)
            row = {'t': data['reference_s'], 'baseline_reason': base[2].get('reason'), 'path_points': len(path)}
            if len(path) >= 2:
                frame = mx.ExecutionFrame(path)
                state = mx.ego_state(frame, lat)
                grid = mx.ReturnGrid(scene, lat)
                s, l = frame.project(grid.returns)
                ds = s-state['s0']
                window = (ds >= -lat.band_behind_m) & (ds <= lat.band_ahead_m)
                wall = (np.abs(l-lat.wall_offset_m) <= lat.wall_tolerance_m) | (np.abs(l+lat.wall_offset_m) <= lat.wall_tolerance_m)
                interior = np.abs(l) < lat.wall_offset_m-lat.wall_tolerance_m
                lateral = (l-state['d0']) >= state['half_span_m']
                near_car = lateral & (l < .6) & (ds > -1.) & (ds < 1.)
                e = ego[max(0, bisect_right(et, data['reference_s'])-1)]
                a = side[max(0, bisect_right(st, data['reference_s'])-1)]
                c, sn = math.cos(e[3]), math.sin(e[3])
                dx, dy = a[1]-e[1], a[2]-e[2]
                row.update(s0=state['s0'], d0=state['d0'], half_span=state['half_span_m'],
                           left_candidates=int(np.sum(near_car)),
                           in_window=int(np.sum(near_car & window)), interior=int(np.sum(near_car & window & interior)),
                           not_wall=int(np.sum(near_car & window & interior & ~wall)),
                           compact=int(np.sum(near_car & window & interior & ~wall & grid.compact_returns)),
                           ds_range=[float(np.min(ds[near_car])), float(np.max(ds[near_car]))] if np.any(near_car) else None,
                           l_range=[float(np.min(l[near_car])), float(np.max(l[near_car]))] if np.any(near_car) else None,
                           eval_side_actor_ego_xy=[c*dx+sn*dy, -sn*dx+c*dy])
            out.append(row)
    (HERE/'vanish_probe.json').write_text(json.dumps({'scope': 'run32 input probe; truth actor pose evaluation only',
                                                      'rows': out}, indent=1)+'\n', encoding='utf-8', newline='\n')
    for row in out:
        print(json.dumps(row))


main()
