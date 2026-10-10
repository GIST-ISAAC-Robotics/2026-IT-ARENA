#!/usr/bin/env python3
"""G1-a 순차 광선/이상적 운동 피드백의 소규모 합성 폐루프. Gazebo/실차 아님."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from types import SimpleNamespace
import zipfile

import numpy as np
from shapely.geometry import Polygon

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable, advance_local
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import pursuit_command, swept_limit, braking_speed, mask_scan
from arena_autonomy.lidar_observation import audit_front_observation
from g0_geometry import TruthBox, Footprint, ray_box_distance

CONFIG = ROOT/'config/tests/g1_follow_v1.json'


def longest_hold_s(trace, first, last, interval):
    longest = current = 0
    for row in trace:
        current = current+1 if first <= row['t'] <= last and abs(row['speed']) < .05 and row['command'] == 0. else 0
        longest = max(longest, current)
    # 표본 개수가 아니라 첫 관측과 마지막 관측 사이의 시간이다. 한 점은 0초다.
    return max(0, longest-1)*interval


def actor_x(name, t):
    if name == 'empty': return None
    if name == 'stopped': return 1.6
    if name == 'moving': return 1.6+.35*t
    if name == 'stop_restart': return 1.6+.35*min(t, 2.)+.45*max(0., t-5.)
    raise ValueError('unknown scenario')


def make_scan(case, now, truth, rng, suite):
    plant, geo = suite['synthetic_plant'], suite['geometry']
    n = plant['points_per_scan']
    first, increment = now-1/plant['scan_hz'], 1/(plant['scan_hz']*n)
    stamps = first+np.arange(n)*increment
    poses = np.column_stack([np.interp(stamps, truth[:, 0], truth[:, axis]) for axis in (1, 2, 3)])
    walls = [TruthBox(3., sign*(geo['wall_y_m']+.025), .4, Footprint(10., .05)) for sign in (-1, 1)]
    walls += [TruthBox(x, 0., .4, Footprint(.05, .95)) for x in geo['end_caps_x_m']]
    ranges = []
    for i, ((x, y, yaw), stamp) in enumerate(zip(poses, stamps)):
        origin = (x+.06*math.cos(yaw), y+.06*math.sin(yaw), .095)
        bearing = -math.pi+i*math.tau/n+yaw
        direction = (math.cos(bearing), math.sin(bearing), 0.)
        target_x = actor_x(case, stamp)
        boxes = walls if target_x is None else walls+[TruthBox(target_x, 0., .12)]
        hits = [hit for box in boxes if (hit := ray_box_distance(origin, direction, box, 0., 12.)) is not None]
        nearest = min(hits, default=math.inf)
        # 최근접이 min range 안이면 뒤벽을 대신 반환하지 않는다.
        if nearest < .05:
            ranges.append(math.nan)
        elif math.isfinite(nearest):
            ranges.append(nearest+rng.uniform(-plant['lidar_range_noise_uniform_m'], plant['lidar_range_noise_uniform_m']))
        else:
            ranges.append(nearest)
    nanos = round(first*1e9)
    return SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame',
        stamp=SimpleNamespace(sec=nanos//10**9, nanosec=nanos%10**9)), ranges=ranges,
        angle_min=-math.pi, angle_increment=math.tau/n, range_min=.05, range_max=12., time_increment=increment)


def run_case(case, seed, mode, suite):
    plant, gates = suite['synthetic_plant'], suite['acceptance']
    cfg = FollowConfig()
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-cfg.wheelbase_m/2,
                                         lidar_x_m=.06))
    controller = LocalFollow(cfg)
    rng = np.random.default_rng(seed)
    ego = np.array([0., 0., 0.])  # 평가기 전용 정답이며 제어기에 전달하지 않는다.
    speed = steer = requested = commanded_steer = 0.
    truth, trace, timings = [], [], []
    scan = None
    detail = {'behavior': 'HOLD', 'reason': 'startup'}
    footprint = Footprint(cfg.length_m, cfg.width_m)
    min_gap, collisions, offroad = math.inf, 0, 0
    dt = plant['dt_s']
    steps_scan = round(1/(plant['scan_hz']*dt))
    steps_control = round(1/(plant['control_hz']*dt))
    for index in range(round(case['duration_s']/dt)+1):
        now = index*dt
        truth.append([now, *ego])
        yaw_rate = speed*math.tan(steer)/cfg.wheelbase_m
        history.add_mean_wheel(now, speed/.0325)
        history.add_gyro(now, yaw_rate)
        if index >= steps_scan and index % steps_scan == 0:
            scan = mask_scan(make_scan(case['name'], now, np.array(truth), rng, suite))
        if index % steps_control == 0 and scan is not None:
            began = time.perf_counter()
            first = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            try:
                observation = audit_front_observation(scan.ranges, scan.angle_min, scan.angle_increment, .05, 12.)
                points, _, _ = history.project(scan.ranges, scan.angle_min, scan.angle_increment, .05, 12.,
                                               first, scan.time_increment, now, 'both', True)
                if mode == 'g1':
                    value, angle, detail = controller.command(motion_scene(scan, history, now, timing_verified=True),
                                                             history, speed, 'left', commanded_steer, 1/plant['control_hz'])
                else:
                    value, angle, detail = pursuit_command(points, 'left', speed, wheelbase=cfg.wheelbase_m,
                        max_steering=cfg.max_steering_rad, max_speed=cfg.max_speed_mps, scan_age=now-first)
                    detail = {**detail, 'behavior': 'R0'}
                angle = float(np.clip(angle, commanded_steer-3/plant['control_hz'], commanded_steer+3/plant['control_hz']))
                # 두 방식에 같은 기존 보호 계산을 적용한다. ROS 독립 보호 프로세스 시험은 아니다.
                clearance = swept_limit(points, angle, cfg.wheelbase_m, length=cfg.length_m, width=cfg.width_m)
                safe = braking_speed(clearance, now-first) if observation.reason == 'ok' else 0.
                requested = min(value, safe, requested+plant['acceleration_mps2']/plant['control_hz'])
                commanded_steer = angle
            except MotionUnavailable as error:
                requested = 0.
                detail = {'behavior': 'HOLD', 'reason': str(error)}
            timings.append((time.perf_counter()-began)*1000)
        polygon = Polygon(footprint.polygon(*ego))
        tx = actor_x(case['name'], now)
        body_gap = None
        if tx is not None:
            target = Polygon(footprint.polygon(tx, 0.))
            body_gap = polygon.distance(target)
            min_gap = min(min_gap, body_gap)
            collisions += int(polygon.intersection(target).area > 1e-10)
        offroad += int(max(abs(p[1]) for p in polygon.exterior.coords) > suite['geometry']['road_half_width_m'])
        if index % steps_control == 0:
            trace.append({'t': now, 'x': float(ego[0]), 'y': float(ego[1]), 'yaw': float(ego[2]),
                          'speed': speed, 'command': requested, 'steering': commanded_steer,
                          'body_gap_m': body_gap, 'behavior': detail['behavior'], 'reason': detail['reason']})
        speed += float(np.clip(requested-speed, -plant['deceleration_mps2']*dt, plant['acceleration_mps2']*dt))
        steer += float(np.clip(commanded_steer-steer, -plant['steering_slew_rad_s']*dt, plant['steering_slew_rad_s']*dt))
        dx, dy, da = advance_local(speed, speed*math.tan(steer)/cfg.wheelbase_m, dt)
        c, s = math.cos(ego[2]), math.sin(ego[2])
        # advance_local은 후륜축 기준이다. 차체 중앙의 회전 오프셋을 되돌린다.
        rx, ry = ego[0]-cfg.wheelbase_m/2*c, ego[1]-cfg.wheelbase_m/2*s
        ego = np.array([rx+c*dx-s*dy+cfg.wheelbase_m/2*math.cos(ego[2]+da),
                        ry+s*dx+c*dy+cfg.wheelbase_m/2*math.sin(ego[2]+da), ego[2]+da])
    stopped = [r for r in trace if 3.5 <= r['t'] <= (4.9 if case['name'] == 'stop_restart' else 7.)
               and abs(r['speed']) < .05 and r['command'] == 0.]
    restarted = [r['t'] for r in trace if r['t'] >= 5. and r['speed'] > .1]
    checks = {'no_sampled_collision': collisions == 0, 'inside_road_samples': offroad == 0,
              'lateral_path_held': max(abs(r['y']) for r in trace) <= gates['maximum_lateral_center_error_m'],
              'body_gap': min_gap >= gates['minimum_body_gap_m']}
    if case['name'] == 'empty': checks['progress'] = trace[-1]['x'] >= gates['minimum_empty_progress_m']
    if case['name'] in ('moving', 'stop_restart'): checks['progress'] = trace[-1]['x'] >= gates['minimum_follow_progress_m']
    if case['name'] in ('stopped', 'stop_restart'):
        checks['hold'] = longest_hold_s(trace, 3.5, 4.9 if case['name'] == 'stop_restart' else 7.,
                                       1/plant['control_hz']) >= gates['minimum_hold_duration_s']
    if case['name'] == 'stop_restart': checks['restart'] = bool(restarted) and restarted[0]-5. <= gates['maximum_restart_delay_s']
    return {'case': case['name'], 'mode': mode, 'seed': seed, 'checks': checks,
        'passed': all(checks.values()), 'progress_m': trace[-1]['x'], 'min_body_gap_m': min_gap if math.isfinite(min_gap) else None,
        'collision_samples': collisions, 'offroad_samples': offroad,
        'max_lateral_error_m': max(abs(r['y']) for r in trace),
        'hold_samples': len(stopped),
        'longest_hold_s': longest_hold_s(trace, 3.5, 4.9 if case['name'] == 'stop_restart' else 7., 1/plant['control_hz']),
        'restart_delay_s': restarted[0]-5. if restarted and case['name'] == 'stop_restart' else None,
        'processing_ms': {'p50': float(np.percentile(timings, 50)), 'p99': float(np.percentile(timings, 99)), 'max': max(timings)},
        'sampling_scope': 'synthetic kinematic plant at 100 Hz; not continuous collision or actuator proof',
        'trace': trace}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--case', choices=['all', 'empty', 'stopped', 'moving', 'stop_restart'], default='all')
    parser.add_argument('--seed', type=int, choices=[0, 1, 2])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    suite = json.loads(CONFIG.read_text())
    sources = [Path(__file__), CONFIG, ROOT/'scripts/g0_geometry.py'] + [ROOT/'src/arena_autonomy/arena_autonomy'/f'{n}.py'
        for n in ('local_follow', 'scene_motion', 'local_scene', 'local_path', 'lidar_motion', 'lidar_observation')]
    hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'w', zipfile.ZIP_DEFLATED) as bundle:
        for p in sources: bundle.write(p, str(p.relative_to(ROOT)))
    results = []
    for case in suite['cases']:
        if args.case not in ('all', case['name']): continue
        for seed in ([args.seed] if args.seed is not None else suite['synthetic_plant']['seeds']):
            for mode in ('r0', 'g1'):
                result = run_case(case, seed, mode, suite)
                trace = result.pop('trace')
                path = args.output/f'{case["name"]}_{seed}_{mode}.json'
                path.write_text(json.dumps(trace, ensure_ascii=False, allow_nan=False))
                result['trace'] = path.name
                result['trace_sha256'] = hashlib.sha256(path.read_bytes()).hexdigest()
                results.append(result)
                print(json.dumps({k: result[k] for k in ('case', 'seed', 'mode', 'passed', 'checks', 'progress_m', 'min_body_gap_m')}), flush=True)
    unchanged = all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest for name, digest in hashes.items())
    report = {'created_at': datetime.now(timezone.utc).isoformat(), 'source_sha256': hashes, 'source_unchanged': unchanged,
              'configuration': suite, 'controller_config': asdict(FollowConfig()), 'results': results,
              'passed': unchanged and bool(results) and all(r['passed'] for r in results if r['mode'] == 'g1'),
              'scope': 'synthetic closed-loop only; R0 comparison is not required to pass G1 headway gates',
              'gazebo_or_real_vehicle_verified': False}
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
