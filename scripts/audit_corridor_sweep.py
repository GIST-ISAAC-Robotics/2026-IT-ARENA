#!/usr/bin/env python3
"""현재 자세 출발의 정곡률 외곽 깊이 감사. 경로 접선에 즉시 정렬하지 않는다."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_visibility import bracket_depth
from probe_corner_corridor import ROOT, digest, shadow_scene
from probe_corner_temporal import baseline_matches


def sweep_poses(steering, wheelbase, horizon, step=.04):
    """현행 swept_limit과 동일한 후륜축 거리와 차체 기준점. yaw를 따로 반환한다."""
    if not all(math.isfinite(v) for v in (steering, wheelbase, horizon, step)) or not (
            abs(steering) <= .37 and wheelbase > 0 and 0 < horizon <= 2.5 and 0 < step <= .04):
        raise ValueError('bounded steering and sweep required')
    distance = np.r_[np.arange(0., horizon, step), horizon]
    curvature = math.tan(steering)/wheelbase
    yaw = curvature*distance
    if abs(curvature) < 1e-5:
        rear_x, rear_y = distance-wheelbase/2, np.zeros_like(distance)
    else:
        rear_x = np.sin(yaw)/curvature-wheelbase/2
        rear_y = (1-np.cos(yaw))/curvature
    center_x = rear_x+wheelbase/2*np.cos(yaw)
    center_y = rear_y+wheelbase/2*np.sin(yaw)
    return np.column_stack((distance, center_x, center_y, yaw))


def sweep_visibility(scene, steering, required_distance, config):
    poses = sweep_poses(steering, config.wheelbase_m, required_distance)
    lx, wy = config.length_m/2, config.width_m/2
    border = np.unique(np.vstack(([(x,y) for x in (-lx,lx) for y in np.linspace(-wy,wy,7)],
                                  [(x,y) for x in np.linspace(-lx,lx,9) for y in (-wy,wy)])), axis=0)
    age = scene.t_ref_s-scene.source_first_s
    padding = [.025+config.measurement_padding_m+config.object_motion_bound_mps*(scene.t_ref_s-r.acquired_at_s
                if config.occupancy_timing_mode == 'per_ray_candidate' else age) for r in scene.rays]
    previous = prefix = 0.
    counts = Counter()
    checked = omitted = 0
    first_failure = None
    for distance, x, y, yaw in poses:
        c, s = math.cos(yaw), math.sin(yaw)
        q = border@np.array([[c,s],[-s,c]])+[x,y]
        inside = (np.abs(q[:,0]) <= lx+1e-10) & (np.abs(q[:,1]) <= wy+1e-10)
        omitted += int(inside.sum())
        results = bracket_depth(scene, q[~inside], padding)
        checked += len(results)
        counts.update(r['reason'] for r in results)
        failure = next((r for r in results if not r['supported']), None)
        if failure is not None and first_failure is None:
            first_failure = dict(rear_axle_distance_m=float(distance), center_xy_m=[float(x),float(y)],
                                 yaw_rad=float(yaw), ray_evidence=failure)
            prefix = max(0., previous-.04)  # 기존 이산 점유 검사와 같은 한 표본 간격을 별도 차감.
        if first_failure is None:
            prefix = float(distance)
        previous = float(distance)
    return dict(steering_rad=float(steering), required_distance_m=required_distance,
                poses=len(poses), query_samples=checked, current_body_queries_excluded=omitted,
                depth_supported_prefix_m=prefix, stopping_sweep_depth_supported=first_failure is None,
                first_unsupported=first_failure, reasons=dict(counts),
                between_ray_volume_observed=False, continuous_sweep_verified=False,
                intermediate_steering_angles_verified=False, free_area_certified=False,
                actual_braking_verified=False, driving_authorized=False)


def required_stopping_distance(measured, proposed, age, config):
    if not all(math.isfinite(v) for v in (measured, proposed, age)) or min(measured, proposed, age) < 0:
        raise ValueError('finite forward estimated speeds and nonnegative age required')
    speed = max(measured, proposed)
    # 현행 제동식 역산. 단일 모터축 측정값이 차체 속도 상한임을 보장하는 식은 아니다.
    return config.standstill_gap_m+.06+speed*(age+.10)+speed*speed/4


def inspect_events(events, temporal):
    selected = {r['seq']:r for r in temporal['rows'] if r['shadow']['temporal_candidate']}
    config = motion = None
    rows = []
    for event in events:
        data = event['data']
        if event['kind'] == 'header':
            config, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
        if event['kind'] != 'call' or event['seq'] not in selected:
            continue
        inputs, scan, history = restore_call(data['input'], motion)
        scene = motion_scene(scan, history, inputs['reference_s'], timing_verified=inputs['timing_verified'],
                             epoch=inputs['epoch'], rear_half_angle_deg=inputs['rear_half_angle_deg'])
        result = shadow_scene(scene, inputs['speed_mps'], inputs['side'], inputs['previous_steering_rad'], inputs['dt_s'], config)
        if not result['preliminary_motion_supported']:
            raise ValueError('saved temporal proposal not reproduced')
        required = required_stopping_distance(inputs['speed_mps'], result['proposed_speed_mps'],
                                              scene.t_ref_s-scene.source_first_s, config)
        detail = dict(seq=event['seq'], time_s=inputs['reference_s'], original_hold=data['output'][0] == 0,
            measured_motor_equivalent_speed_mps=inputs['speed_mps'], proposed_speed_mps=result['proposed_speed_mps'],
            required_stopping_distance_m=required, driving_authorized=False)
        if required > 2.5:
            rows.append({**detail, 'evaluated':False, 'reason':'existing_sweep_horizon_exceeded'})
            continue
        audits = [sweep_visibility(scene, angle, required, config)
                  for angle in (inputs['previous_steering_rad'], result['steering_rad'])]
        rows.append({**detail, 'evaluated':True, 'sweeps':audits,
                     'both_sweeps_depth_supported':all(a['stopping_sweep_depth_supported'] for a in audits)})
    if len(rows) != len(selected):
        raise ValueError('saved calls missing')
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--stateless', type=Path, required=True)
    parser.add_argument('--temporal', type=Path, required=True)
    parser.add_argument('--visibility', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = {str(p):digest(p) for p in (args.input, args.stateless, args.temporal, args.visibility)}
    stateless, temporal, visibility = [json.loads(p.read_text()) for p in (args.stateless, args.temporal, args.visibility)]
    baseline_matches(stateless, args.input)
    if (temporal['input_sha256'] != digest(args.input) or temporal['baseline_sha256'] != digest(args.stateless) or
            not all(visibility[k] for k in ('input_unchanged','source_unchanged')) or visibility['runtime_changed']):
        raise ValueError('preceding report provenance mismatch')
    sources = {**visibility['source_sha256'], 'scripts/audit_corridor_sweep.py':digest(Path(__file__)),
               'tests/test_corridor_sweep.py':digest(ROOT/'tests/test_corridor_sweep.py')}
    for p, value in sources.items():
        if digest(ROOT/p) != value:
            raise ValueError(f'source mismatch: {p}')
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in sources:
            archive.write(ROOT/p, p)
    with args.input.open() as stream:
        rows = inspect_events((json.loads(line) for line in stream), temporal)
    summary = dict(candidates=len(rows), evaluated=sum(r['evaluated'] for r in rows),
                   both_sweeps_depth_supported=sum(r.get('both_sweeps_depth_supported',False) for r in rows),
                   original_hold_candidates=sum(r['original_hold'] for r in rows),
                   original_hold_supported=sum(r['original_hold'] and r.get('both_sweeps_depth_supported',False) for r in rows))
    result = dict(scope='current_pose_constant_curvature_depth_audit_only', summary=summary, rows=rows,
                  source_sha256=sources, input_sha256=inputs,
                  source_unchanged=all(digest(ROOT/p) == h for p,h in sources.items()),
                  input_unchanged=all(digest(p) == h for p,h in inputs.items()),
                  runtime_changed=False, driving_authorized=False, closed_loop_verified=False,
                  between_ray_volume_observed=False, actual_braking_verified=False)
    with (args.output/'report.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({**summary, 'input_unchanged':result['input_unchanged'], 'source_unchanged':result['source_unchanged']}))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
