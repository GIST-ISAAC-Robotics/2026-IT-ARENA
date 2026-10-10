"""탐색용: 특정 tick의 기억 경로·지원점 선택을 출력한다(제어 변경 없음)."""
import json, math, sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow, path_agrees, boundary_support
from arena_autonomy.local_path import local_path, mask_scan
from arena_autonomy.scene_motion import motion_scene, transport_points

run = Path(sys.argv[1]); t0, t1 = float(sys.argv[2]), float(sys.argv[3]); debug_t = [float(v) for v in sys.argv[4].split(',')]
scans = [json.loads(l) for l in (run/'scans_received.jsonl').read_text().splitlines()]
act = json.load(open(run/'actuation_trace.json'))
fb = sorted({r['feedback_capture_us']: r['mean_wheel_mps'] for r in act if r['feedback_valid'] and r['feedback_capture_us'] is not None}.items())
ego = np.array([json.loads(l) for l in (run/'ego_poses.jsonl').read_text().splitlines()])
yaw = np.unwrap(ego[:, 3]); rate = np.gradient(yaw, ego[:, 0])
gyro_t = np.arange(0, t1+.5, .005); gyro_v = np.interp(gyro_t, ego[:, 0], rate)


def scan_message(row):
    nanos = round(row['stamp_s']*1e9)
    return SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame', stamp=SimpleNamespace(sec=nanos//10**9, nanosec=nanos % 10**9)),
                           ranges=[math.inf if r is None else r for r in row['ranges']], angle_min=row['angle_min'],
                           angle_increment=row['angle_increment'], range_min=.05, range_max=12., time_increment=row['time_increment'])


cfg = FollowConfig(wheelbase_m=.135, max_steering_rad=.37, max_speed_mps=1.0, wall_offset_m=.425)
ctrl = LocalFollow(cfg); hist = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
fi = gi = 0; steer = 0.
for k in range(int(round(t0/.02)), int(round(t1/.02))+1):
    t = k*.02
    while fi < len(fb) and fb[fi][0]*1e-6 <= t-.005:
        hist.wheels.add(fb[fi][0]*1e-6, fb[fi][1]); fi += 1
    while gi < len(gyro_t) and gyro_t[gi] <= t-.004:
        hist.add_gyro(gyro_t[gi], gyro_v[gi]); gi += 1
    avail = [s for s in scans if s['stamp_s']+len(s['ranges'])*s['time_increment'] <= t+1e-9]
    if not avail:
        continue
    scan = mask_scan(scan_message(avail[-1]))
    speed = hist.wheels.values[-1] if hist.wheels.values else 0.
    try:
        scene = motion_scene(scan, hist, t, timing_verified=True)
    except MotionUnavailable as e:
        continue
    if any(abs(t-d) < 1e-6 for d in debug_t) and ctrl.path is not None:
        pts = np.asarray(scene.occupied_endpoints_xy_m)
        carried = transport_points(ctrl.path, hist, ctrl.path_ref, t)
        tang = np.gradient(carried, axis=0); tang /= np.linalg.norm(tang, axis=1)[:, None]
        wall = carried + .425*np.column_stack((-tang[:, 1], tang[:, 0]))
        print(f'== t={t:.2f} scan={avail[-1]["stamp_s"]} path_ref={ctrl.path_ref} src={ctrl.path_source}')
        print('   carried x', round(carried[0, 0], 3), '..', round(carried[-1, 0], 3), ' expected wall y@x=.35:', round(float(np.interp(.35, wall[:, 0], wall[:, 1])), 3))
        sel = boundary_support(pts, carried, 'left', cfg)
        L, li = local_path(sel, 'left', .425)
        print('   support', len(sel), li.get('reason'), li.get('boundary_observed_ahead_m'), 'agree' if L is not None and path_agrees(L, carried, .08) else 'no-agree')
        keep = set(map(tuple, np.round(sel, 7)))
        m = (pts[:, 0] > .15) & (pts[:, 0] < .8) & (pts[:, 1] > .2)
        for p in pts[m]:
            d = np.min(np.linalg.norm(wall-p, axis=1))
            print('     pt', np.round(p, 3), 'd_wall', round(float(d), 3), 'kept' if tuple(np.round(p, 7)) in keep else 'drop')
        # 선택 결과의 연결 끊김
        q = sel[(sel[:, 1] > .1)]
        q = q[np.argsort(-np.arctan2(q[:, 1], q[:, 0]))]
        gaps = np.linalg.norm(np.diff(q, axis=0), axis=1)
        lim = .10+.035*np.minimum(np.linalg.norm(q[:-1], axis=1), np.linalg.norm(q[1:], axis=1))
        for j in np.flatnonzero(gaps > lim):
            print('   BREAK between', np.round(q[j], 3), np.round(q[j+1], 3), round(float(gaps[j]), 3), '>', round(float(lim[j]), 3))
    sp, steer, d = ctrl.command(scene, hist, speed, 'left', steer, .02)
    if d.get('boundary_candidate_reason') is not None or d['reason'] != 'local_follow':
        print(round(t, 2), d['reason'], d.get('boundary_selection'), d.get('boundary_candidate_ahead_m'), d.get('fresh_boundary_accepted'), d.get('boundary_from_memory'))
