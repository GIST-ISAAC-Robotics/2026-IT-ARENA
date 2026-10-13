#!/usr/bin/env python3
"""G2 평가 전용: 실제 pursue_path의 조향을 이상적 평면 bicycle에 적용.

입력 reference는 평가용 정답 곡선이다. 노드/토픽/실차 출력은 없다.
명령 지연은 시험 가정이며 모터 토크 40 ms 응답이나 서보 실측이 아니다.
"""
import argparse
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
from shapely.geometry import Polygon

from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.lidar_motion import MotionUnavailable
from g0_geometry import Footprint, TruthBox
from g2_geometry_probe import ROOT, ProbeConfig, sample_path, assess, numeric


def bicycle_step(x, y, yaw, speed, steering, dt, wheelbase):
    values = [numeric(v) for v in (x, y, yaw, speed, steering, dt, wheelbase)]
    x, y, yaw, speed, steering, dt, wheelbase = values
    if dt <= 0 or wheelbase <= 0 or abs(steering) >= math.pi/2:
        raise ValueError('invalid bicycle integration input')
    distance = speed*dt
    curvature = math.tan(steering)/wheelbase
    turn = curvature*distance
    if abs(curvature) < 1e-10:
        return x+distance*math.cos(yaw), y+distance*math.sin(yaw), yaw
    return (x+(math.sin(yaw+turn)-math.sin(yaw))/curvature,
            y-(math.cos(yaw+turn)-math.cos(yaw))/curvature, yaw+turn)


def local_reference(reference, rear_pose, wheelbase, nearest_floor):
    """평가 곡선을 base_link로 변환. 뒤로 되돌리지 않고 기존 1.8 m/36점을 사용한다."""
    rx, ry, yaw = rear_pose
    distances = np.linalg.norm(reference[nearest_floor:]-[rx, ry], axis=1)
    nearest = nearest_floor+int(np.argmin(distances))
    steps = np.linalg.norm(np.diff(reference[nearest:], axis=0), axis=1)
    arc = np.r_[0., np.cumsum(steps)]
    stop = min(1.8, arc[-1])
    selected = np.column_stack([np.interp(np.linspace(0., stop, 36), arc,
                                         reference[nearest:, axis]) for axis in (0, 1)])
    cx, cy = rx+wheelbase/2*math.cos(yaw), ry+wheelbase/2*math.sin(yaw)
    delta = selected-[cx, cy]
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((c*delta[:, 0]+s*delta[:, 1], -s*delta[:, 0]+c*delta[:, 1])), nearest


def rollout(offset, transition, *, speed_cap=.4, delay_steps=0, target_y=.24, cfg=ProbeConfig()):
    speed_cap = numeric(speed_cap)
    if not 0 < speed_cap <= 1 or isinstance(delay_steps, bool) or not isinstance(delay_steps, int) or not 0 <= delay_steps <= 10:
        raise ValueError('bounded tracking probe exceeded')
    path = sample_path(offset, transition, cfg)
    reference = np.array([(r['rear_x_m'], r['rear_y_m']) for r in path['samples']])
    target = TruthBox(1.5, target_y, .12)
    obstacle = Polygon(target.footprint.polygon(target.x_m, target.y_m, target.yaw_rad))
    shape = Footprint(cfg.length_m, cfg.width_m)
    follow = FollowConfig(wheelbase_m=cfg.wheelbase_m, max_steering_rad=cfg.max_steering_rad)
    dt, steps_limit = .02, 1250  # 25 s cap, not extended after failure
    pose, steering, speed, cursor = (-cfg.wheelbase_m/2, 0., 0.), 0., 0., 0
    delayed = deque([(0., 0.)]*delay_steps)
    rows = []
    reached, fault = False, None
    rear_radius = math.hypot((cfg.length_m+cfg.wheelbase_m)/2, cfg.width_m/2)
    for step in range(steps_limit):
        local, cursor = local_reference(reference, pose, cfg.wheelbase_m, cursor)
        try:
            cap, desired, _ = pursue_path(local, speed, follow, .1)
        except MotionUnavailable as error:
            fault = str(error)
            break
        command = float(np.clip(desired, steering-3*dt, steering+3*dt))
        requested_speed = min(speed_cap, cap)
        delayed.append((requested_speed, command))
        applied_speed, applied_steering = delayed.popleft()
        x, y, yaw = pose
        polygon = Polygon(shape.polygon(x+cfg.wheelbase_m/2*math.cos(yaw),
                                         y+cfg.wheelbase_m/2*math.sin(yaw), yaw))
        bound = abs(applied_speed)*dt*(1+rear_radius*abs(math.tan(applied_steering)/cfg.wheelbase_m))
        gap = polygon.distance(obstacle)
        edge = cfg.road_width_m/2-max(abs(polygon.bounds[1]), abs(polygon.bounds[3]))
        rows.append(dict(t_s=step*dt, rear_x_m=x, rear_y_m=y, yaw_rad=yaw,
                         command_steering_rad=command, applied_steering_rad=applied_steering,
                         applied_speed_mps=applied_speed, body_gap_lower_m=max(0., gap-bound),
                         road_edge_lower_m=edge-bound,
                         nearest_reference_error_m=float(np.linalg.norm(reference[cursor]-[x, y]))))
        pose = bicycle_step(*pose, applied_speed, applied_steering, dt, cfg.wheelbase_m)
        steering, speed = command, applied_speed
        if pose[0] >= 1.85+transition+.02:
            reached = True
            break
    minimum_gap = min((r['body_gap_lower_m'] for r in rows), default=None)
    minimum_edge = min((r['road_edge_lower_m'] for r in rows), default=None)
    checks = dict(checkpoint_reached=reached, no_path_fault=fault is None,
                  body_gap=minimum_gap is not None and minimum_gap >= cfg.minimum_body_gap_m,
                  road_margin=minimum_edge is not None and minimum_edge >= cfg.edge_margin_m,
                  rejoins_center=abs(pose[1]) <= .02, rejoins_heading=abs(pose[2]) <= .05)
    return dict(offset_m=offset, transition_m=transition, speed_cap_mps=speed_cap,
                delay_steps=delay_steps, delay_s=delay_steps*dt,
                ideal_geometry=assess(path, target, cfg), checks=checks,
                tracking_probe_conditions_met=all(checks.values()), driving_authorized=False,
                minimum_body_gap_lower_m=minimum_gap, minimum_road_edge_lower_m=minimum_edge,
                final_rear_pose=list(pose), fault=fault, steps=len(rows), rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('new validation subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    names = ['scripts/g2_tracking_probe.py', 'scripts/g2_geometry_probe.py', 'scripts/g0_geometry.py',
             'tests/test_g2_tracking_probe.py', 'src/arena_autonomy/arena_autonomy/local_follow.py',
             'src/arena_autonomy/arena_autonomy/local_path.py', 'src/arena_description/config/vehicle.yaml']
    raw = {name:(ROOT/name).read_bytes() for name in names}
    hashes = {name:hashlib.sha256(value).hexdigest() for name,value in raw.items()}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for name,data in raw.items():
            bundle.writestr(name,data)
    results = []
    for offset in (-.075, -.10):
        for length in (.6, .9, 1.2):
            for speed in (.4, .8):
                for delay in (0, 5):
                    result = rollout(offset, length, speed_cap=speed, delay_steps=delay)
                    name = f'offset{offset}_length{length}_speed{speed}_delay{delay}'
                    rows = result.pop('rows')
                    (output/(name+'.json')).write_text(json.dumps(rows, allow_nan=False), encoding='utf-8')
                    results.append(dict(name=name, **result))
    unchanged = all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == value for name,value in hashes.items())
    report = dict(scope='evaluation_reference_ideal_bicycle_actual_pursue_path', config=asdict(ProbeConfig()),
                  cases=results, source_sha256=hashes, sources_unchanged=unchanged, driving_authorized=False,
                  limits=['Exact reference is evaluation truth, not available to production controller.',
                          'No mass/tire/slip/longitudinal motor or measured servo dynamics.',
                          '0/100 ms command delay is a test assumption, not the motor torque 40 ms constant.',
                          'No scan/occlusion/dynamic target/independent protection integration.',
                          'Tracking thresholds are pre-registered experiment criteria, not hardware tolerances.'])
    (output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False),encoding='utf-8')
    print(json.dumps(dict(cases=len(results), met=sum(r['tracking_probe_conditions_met'] for r in results),
                         sources_unchanged=unchanged, driving_authorized=False)))
    return 0 if unchanged else 1


if __name__ == '__main__':
    raise SystemExit(main())
