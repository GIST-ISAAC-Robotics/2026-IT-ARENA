"""초기 G2 후보의 기하 경로와 실제 PP 추종 예측을 비교한다. 제어 연결 없음."""
import hashlib
import json
import math
from pathlib import Path
import sys
import time

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'scripts')]
import numpy as np
from arena_autonomy.bypass_entry_trial import EntryConfig, EntryGuide, _compare_entries, current_outer_wall_padding
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene
from bypass_probe_sensors import ProbeSensors
from probe_bypass_entry_sequence import render_scan
from g2_tracking_probe import bicycle_step


def initial_scene(profile='combined', seed=17, target_y=0.):
    config = FollowConfig()
    entry = EntryConfig(offsets_m=(0., -.16, .16, -.20, .20, -.24, .24, -.27, .27))
    guide = EntryGuide(config, entry, stationary_outer_boundary=True, rejoin_enabled=True,
                       wall_reanchor=True, tracked_wall_fit=True)
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675,
                                         lidar_x_m=.06, history_s=5.))
    sensors = ProbeSensors(profile, seed)
    physical = []
    for k in range(25):
        at = k*.02
        physical.append(dict(t=at, rear_pose=[-.0675, 0., 0.], speed=0., steering=0.))
        mean, gyro, _ = sensors.measure_motion(at, 0., 0., 0.)
        history.add_mean_wheel(at, mean)
        history.add_gyro(at, gyro)
    for k in range(6):
        at = round(.5+k*.02, 9)
        physical.append(dict(t=at, rear_pose=[-.0675, 0., 0.], speed=0., steering=0.))
        if sensors.motion_enabled:
            mean, gyro, _ = sensors.measure_motion(at, 0., 0., 0.)
            history.add_mean_wheel(at, mean)
            history.add_gyro(at, gyro)
        if k%5 == 0:
            scan = render_scan(physical, at, target_y, False, sensors)
        scene = motion_scene(scan, history, at, timing_verified=True, rear_half_angle_deg=30.)
        result = guide.evaluate(scene, history, 0., 0., .02)
        if k < 5 and result['selected'] is not None:
            raise AssertionError('unexpected earlier movement')
        if not sensors.motion_enabled:
            history.add_mean_wheel(at, 0.)
            history.add_gyro(at, 0.)
    compare = _compare_entries(scene, at, 0., 0., .02, config=config, entry=entry,
                               stationary_boundaries=True)
    if result['selected'] != compare['selected']:
        raise AssertionError('initial guide comparison mismatch')
    return config, scene, compare


def predict(path, points, padding, config, age, dt=.02):
    """같은 시각의 반환점에 대한 순위 진단. 충돌/관측 승인이나 실차 예측이 아니다."""
    rear = np.array([-config.wheelbase_m/2, 0., 0.])
    speed, steering = 0., 0.
    rows = []
    horizon = min(1.25, float(path[-1, 0])-.15)
    stop_reason = 'duration_bound'
    began = time.perf_counter()
    for k in range(round(6./dt)+1):
        yaw = rear[2]
        center = rear[:2]+config.wheelbase_m/2*np.array([math.cos(yaw),math.sin(yaw)])
        rot = np.array([[math.cos(yaw),math.sin(yaw)],[-math.sin(yaw),math.cos(yaw)]])
        local_points = (points-center)@rot.T
        gap = float(np.min(np.maximum(abs(local_points[:,0])-config.length_m/2-padding,
                                      abs(local_points[:,1])-config.width_m/2-padding)))
        rows.append(dict(t=k*dt, center_pose=[*center.tolist(), float(yaw)],
                         speed_mps=speed, steering_rad=steering, padded_point_gap_m=gap))
        if center[0] >= horizon:
            stop_reason = 'observed_preview_horizon'
            break
        local_path = (path-center)@rot.T
        cap, requested, _ = pursue_path(local_path, speed, config, age)
        if cap <= 0.:
            stop_reason = 'path_pursuit_unavailable'
            break
        steering = float(np.clip(requested, steering-3*dt, steering+3*dt))
        speed = float(np.clip(min(.35, cap), max(0., speed-2*dt), speed+dt))
        rear = np.asarray(bicycle_step(*rear, speed, steering, dt, config.wheelbase_m))
    hits = [i for i,r in enumerate(rows) if r['padded_point_gap_m'] <= 0.]
    idx = max(0,hits[0]-1) if hits else len(rows)-1
    return dict(first_padded_interference_progress_m=rows[idx]['center_pose'][0],
                padded_interference=bool(hits), minimum_padded_point_gap_m=min(r['padded_point_gap_m'] for r in rows),
                stop_reason=stop_reason, horizon_m=horizon, computation_ms=(time.perf_counter()-began)*1000,
                rows=rows)


def inspect(profile, target_y):
    config, scene, comparison = initial_scene(profile=profile, target_y=target_y)
    points = np.asarray(scene.occupied_endpoints_xy_m)
    candidates = []
    for row in comparison['candidates']:
        if not row['eligible']:
            continue
        side = row['current_outer_wall']
        wall = comparison['reference']['walls'][side]
        padding, _ = current_outer_wall_padding(scene, config, side, wall)
        result = predict(np.asarray(row['path_points']), points, padding, config,
                         scene.t_ref_s-scene.source_first_s)
        candidates.append(dict(offset_m=row['offset_m'], transition_m=row['transition_m'],
            old_ideal_progress_m=row['preview_return_progress_m'],
            old_ideal_gap_m=row['preview_point_gap_m'],
            reference_path=row['path_points'], prediction=result))
    return dict(profile=profile, target_y=target_y, selected=comparison['selected'],
                reference=comparison['reference'], points=points.tolist(), candidates=candidates,
                excluded=[{k:r.get(k) for k in ('offset_m','transition_m','reason')} for r in comparison['candidates'] if not r['eligible']])


if __name__ == '__main__':
    output = BASE/'initial_comparison.json'
    if output.exists():
        raise RuntimeError('preserve prior diagnostic')
    sources = [Path(__file__), ROOT/'src/arena_autonomy/arena_autonomy/bypass_entry_trial.py',
               ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py', ROOT/'scripts/probe_bypass_entry_sequence.py',
               ROOT/'scripts/bypass_probe_sensors.py',ROOT/'scripts/g2_tracking_probe.py']
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    cases = [inspect(profile,y) for profile,y in (('ideal',0.),('combined',0.),('combined',-.15),('combined',.15))]
    result = dict(scope='initial frozen-scan ranking diagnostic only; no observation or future collision permission, no ROS/Gazebo or moving actor proof',
                  source_sha256=hashes, cases=cases,
                  source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items()))
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    for case in cases:
        print(case['profile'],case['target_y'],'selected',case['selected'])
        for c in case['candidates']:
            p = c['prediction']
            print('  offset=%+.2f L=%.1f old_progress=%.3f old_gap=%.3f predicted=%.3f gap=%.3f cost=%.1fms' %
                  (c['offset_m'],c['transition_m'],c['old_ideal_progress_m'],c['old_ideal_gap_m'],p['first_padded_interference_progress_m'],p['minimum_padded_point_gap_m'],p['computation_ms']))
