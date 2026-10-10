"""탐색용: 기록 scan + 기록 MCU 피드백(+gyro 근사)으로 LocalFollow 결정을 재구성."""
import json, math, sys
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow, path_agrees
from arena_autonomy.local_path import local_path, mask_scan
from arena_autonomy.scene_motion import motion_scene

run = Path(sys.argv[1])
t0, t1 = float(sys.argv[2]), float(sys.argv[3])
gyro_mode = sys.argv[4] if len(sys.argv) > 4 else 'zero'
scans = [json.loads(l) for l in (run/'scans_received.jsonl').read_text().splitlines()]
act = json.load(open(run/'actuation_trace.json'))
fb = {}
for r in act:
    if r['feedback_valid'] and r['feedback_capture_us'] is not None:
        fb[r['feedback_capture_us']] = r['mean_wheel_mps']
fb = sorted(fb.items())
ego = np.array([json.loads(l) for l in (run/'ego_poses.jsonl').read_text().splitlines()])
yaw = np.unwrap(ego[:, 3])

def scan_message(row):
    nanos = round(row['stamp_s']*1e9)
    return SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame', stamp=SimpleNamespace(sec=nanos//10**9, nanosec=nanos % 10**9)),
                           ranges=[math.inf if r is None else r for r in row['ranges']], angle_min=row['angle_min'],
                           angle_increment=row['angle_increment'], range_min=.05, range_max=12.,
                           time_increment=row['time_increment'])

cfg = FollowConfig(wheelbase_m=.135, max_steering_rad=.37, max_speed_mps=1.0, wall_offset_m=.425)
mc = MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06)
ctrl = LocalFollow(cfg)
hist = MotionHistory(mc)
fi = 0
gyro_t = np.arange(0, t1+.5, .005)
if gyro_mode == 'truth':
    rate = np.gradient(yaw, ego[:, 0])
    gyro_v = np.interp(gyro_t, ego[:, 0], rate)
else:
    gyro_v = np.zeros_like(gyro_t)
gi = 0
steer = 0.
for k in range(int(round(t0/.02)), int(round(t1/.02))+1):
    t = k*.02
    while fi < len(fb) and fb[fi][0]*1e-6 <= t-.005:
        hist.wheels.add(fb[fi][0]*1e-6, fb[fi][1]); fi += 1
    while gi < len(gyro_t) and gyro_t[gi] <= t-.004:
        hist.add_gyro(gyro_t[gi], gyro_v[gi]); gi += 1
    avail = [s for s in scans if s['stamp_s']+len(s['ranges'])*s['time_increment'] <= t+1e-9]
    if not avail:
        continue
    row = avail[-1]
    scan = mask_scan(scan_message(row))
    speed = hist.wheels.values[-1] if hist.wheels.values else 0.
    try:
        scene = motion_scene(scan, hist, t, timing_verified=True)
    except MotionUnavailable as e:
        print(round(t,3), 'motion', e); continue
    sp, st, d = ctrl.command(scene, hist, speed, 'left', steer, .02)
    steer = st
    pts = np.asarray(scene.occupied_endpoints_xy_m)
    if d.get('boundary_candidate_reason') is not None or d['reason'] != 'local_follow':
        L, li = local_path(pts, 'left', .425); R, ri = local_path(pts, 'right', .425)
        print(round(t,3), 'scan', row['stamp_s'], d['reason'], 'sel', d.get('boundary_selection'), 'cand_ahead', None if d.get('boundary_candidate_ahead_m') is None else round(d['boundary_candidate_ahead_m'],3),
              'acc', d.get('fresh_boundary_accepted'), 'mem', d.get('boundary_from_memory'), 'age', None if d.get('boundary_age_s') is None else round(d['boundary_age_s'],3), 'wall', d.get('path_wall'),
              '| L', li['reason'], round(li.get('boundary_observed_ahead_m', -1),3), round(li.get('wall_distance_m',-1),3), '| R', ri['reason'], round(ri.get('boundary_observed_ahead_m',-1),3), round(ri.get('wall_distance_m',-1),3), 'cmd', round(sp,3))
