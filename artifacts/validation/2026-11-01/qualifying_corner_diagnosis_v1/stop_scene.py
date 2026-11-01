"""정지 뒤 스캔의 좌/우 경로 후보를 세계 좌표로 옮겨 실제 통과 궤적과 비교한다(진단 전용).

실행: python3 -I stop_scene.py <snapshot_root> <out.json>
"""
import gzip
import json
import math
import sys
from pathlib import Path

snapshot, out = sys.argv[1], sys.argv[2]
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
for pkg in ('arena_autonomy', 'arena_vehicle_interface'):
    sys.path.insert(0, str(Path(snapshot) / 'src' / pkg))
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene

RUN = ROOT / 'qualifying_profile_v1/run1'
PICK = (46.52, 47.52, 50.52, 52.52)


def truth():
    with gzip.open(HERE / 'extract_combined_failed.json.gz', 'rt', encoding='utf-8') as f:
        return json.load(f)['trajectory']


def reference_line():
    with gzip.open(HERE / 'extract_turn_only.json.gz', 'rt', encoding='utf-8') as f:
        rows = json.load(f)['trajectory']
    return np.array([[r['x'], r['y'], r['progress_m']] for r in rows if r['light'] == 'green' and 42.5 <= r['progress_m'] <= 46.5])


traj = truth()
ref = reference_line()
motion = None
result = []
with (RUN / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
    for line in f:
        r = json.loads(line)
        if r['kind'] == 'header':
            motion = MotionConfig(**r['data']['motion_config'])
            cfg = r['data']['follow_config']
        if r['kind'] != 'call':
            continue
        t = r['data']['input']['reference_s']
        if not any(abs(t - p) < .005 for p in PICK):
            continue
        data, scan, history = restore_call(r['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
        pose = min(traj, key=lambda x: abs((x['sim_time_s'] or 0) - t))
        c, s = math.cos(pose['truth_yaw_rad']), math.sin(pose['truth_yaw_rad'])
        row = dict(t=t, x=pose['truth_x_m'], y=pose['truth_y_m'], yaw=pose['truth_yaw_rad'], progress=pose['progress_m'],
                   centerline_error=pose['centerline_error_m'])
        horizon = max(1.8, .06 + cfg['standstill_gap_m'] + cfg['open_route_speed_mps'] * (.12 + .10) + cfg['open_route_speed_mps'] ** 2 / 4)
        for side in ('right', 'left'):
            try:
                path, info = local_path(points, side, cfg['wall_offset_m'], horizon=horizon)
            except Exception as e:
                row[side] = dict(error=str(e))
                continue
            if path is None:
                row[side] = dict(path=None, info={k: v for k, v in info.items() if not isinstance(v, (list, np.ndarray))})
                continue
            world = np.column_stack((row['x'] + c * path[:, 0] - s * path[:, 1], row['y'] + s * path[:, 0] + c * path[:, 1]))
            d = np.linalg.norm(world[:, None, :] - ref[None, :, :2], axis=2)
            nearest = d.argmin(axis=1)
            row[side] = dict(n=len(path), path_end_local=path[-1].tolist(), info={k: (float(v) if isinstance(v, (float, np.floating)) else v) for k, v in info.items() if not isinstance(v, (list, np.ndarray))},
                             max_offset_from_reference_line=float(d.min(axis=1).max()), median_offset=float(np.median(d.min(axis=1))),
                             reference_progress_span=[float(ref[nearest[0], 2]), float(ref[nearest[-1], 2])],
                             sample_local=path[::max(1, len(path) // 8)].round(3).tolist())
        ranges = points
        result.append(row)
with open(out, 'w', encoding='utf-8', newline='\n') as f:
    json.dump(dict(scope='평가 자세로 좌표만 옮긴 사후 진단. 기준선은 코너 보완 단독의 실제 통과 궤적.', rows=result), f, ensure_ascii=False, indent=1)
print(json.dumps(result, ensure_ascii=False)[:3500])
