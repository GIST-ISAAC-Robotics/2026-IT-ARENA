"""저장 스캔 전체에서 호길이 비교만 통과한 경우가 실제 주행 경로와 맞는지 사후 대조한다.

평가 자세로 좌표만 옮기는 진단이다. 실행: python3 -I arc_audit.py <snap_candA> <out.json>
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
from arena_autonomy.local_follow import path_agrees, path_agrees_arc
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene

RUNS = {'rise_only': ROOT / 'race_acceleration_v1/run1', 'turn_only': ROOT / 'turn_preview_v1/run4',
        'combined_failed': ROOT / 'qualifying_profile_v1/run1'}
result, flagged = {}, []
for name, folder in RUNS.items():
    with gzip.open(HERE / f'extract_{name}.json.gz', 'rt', encoding='utf-8') as f:
        rows = [r for r in json.load(f)['trajectory'] if r['sim_time_s'] is not None and r['light'] == 'green']
    times = np.array([r['sim_time_s'] for r in rows])
    xy = np.array([[r['truth_x_m'], r['truth_y_m']] for r in rows])
    yaw = np.array([r['truth_yaw_rad'] for r in rows])
    counts = dict(scans=0, both=0, x_only=0, arc_only=0, neither=0, low_speed_scans=0, low_speed_arc_only=0,
                  arc_only_checked=0, arc_only_offset_gt_0p15=0)
    motion = cfg = None
    last = None
    with (folder / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
        for line in f:
            r = json.loads(line)
            if r['kind'] == 'header':
                motion = MotionConfig(**r['data']['motion_config'])
                cfg = r['data']['follow_config']
            if r['kind'] != 'call':
                continue
            i = r['data']['input']
            first = i['scan']['sec'] + i['scan']['nanosec'] * 1e-9
            t = i['reference_s']
            if first == last or t < times[0]:
                continue
            last = first
            data, scan, history = restore_call(i, motion)
            try:
                scene = motion_scene(scan, history, t, timing_verified=data['timing_verified'], epoch=data['epoch'],
                                     rear_half_angle_deg=data['rear_half_angle_deg'])
            except Exception:
                continue
            points = np.asarray(scene.occupied_endpoints_xy_m).reshape(-1, 2)
            horizon = max(1.8, .06 + cfg['standstill_gap_m'] + cfg['open_route_speed_mps'] * .22 + cfg['open_route_speed_mps'] ** 2 / 4)
            right, _ = local_path(points, 'right', cfg['wall_offset_m'], horizon=horizon)
            left, _ = local_path(points, 'left', cfg['wall_offset_m'], horizon=horizon)
            if right is None or left is None:
                continue
            x_ok = path_agrees(right, left, cfg['boundary_agreement_m'])
            arc_ok = path_agrees_arc(right, left, cfg['boundary_agreement_m'])
            low = abs(i['speed_mps']) <= .10
            counts['scans'] += 1
            counts['low_speed_scans'] += low
            counts['both' if x_ok and arc_ok else 'x_only' if x_ok else 'arc_only' if arc_ok else 'neither'] += 1
            if arc_ok and not x_ok:
                counts['low_speed_arc_only'] += low
                k = int(np.searchsorted(times, t))
                if k >= len(rows) or times[-1]-t < 3.:
                    continue
                c, s = math.cos(yaw[k]), math.sin(yaw[k])
                offsets = []
                # 실제 이후 궤적과 비교할 수 있는 정지 전 구간만 센다.
                future = xy[k:k+90]
                if np.linalg.norm(future[-1]-future[0]) < .5:
                    continue
                for path in (right, left):
                    p = np.asarray(path)
                    origin = int(np.argmin(np.linalg.norm(p, axis=1)))
                    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p[origin:], axis=0), axis=1))]
                    q = p[origin:][arc <= 1.]
                    world = xy[k]+np.column_stack((c*q[:, 0]-s*q[:, 1], s*q[:, 0]+c*q[:, 1]))
                    offsets.append(float(np.linalg.norm(world[:, None]-future[None], axis=2).min(axis=1).max()))
                counts['arc_only_checked'] += 1
                if max(offsets) > .15:
                    counts['arc_only_offset_gt_0p15'] += 1
                    flagged.append(dict(run=name, t=round(t, 2), meas=i['speed_mps'], offsets=[round(v, 3) for v in offsets],
                                        progress=rows[k]['progress_m']))
    result[name] = counts
    print(name, counts)
with open(out, 'w', encoding='utf-8', newline='\n') as f:
    json.dump(dict(scope='평가 자세 기반 사후 대조. 궤적이 충분히 이어지는 스캔만 비교했다.', runs=result, flagged=flagged[:30]), f, ensure_ascii=False, indent=1)
print('flagged', len(flagged), flagged[:5])
