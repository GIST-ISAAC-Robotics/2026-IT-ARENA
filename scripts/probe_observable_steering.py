#!/usr/bin/env python3
"""저장 입력에서 관측 가능한 정곡률 선택지 비교. 명령/실행 정책이 아니다."""
import argparse
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path, swept_limit
from arena_autonomy.observation_support import RecentScenes, sweep_queries, source_valid
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest
from audit_corridor_sweep import required_stopping_distance
from audit_observation_history import lateral_support
from probe_observation_envelope import envelope_poses

ROOT = Path(__file__).resolve().parents[1]


def path_deviation(path, steering, distance, config):
    """현재 경로 polyline의 실제 범위 안에서만 중심 이동과 비교한다.

    .16 m 정지 간격은 관측 길이에 남기되 실제 움직임의 경로 비교에서 분리한다.
    맨 앞/뒤 끝점을 넘어 투영되는 질의는 미지원으로 남긴다. 노면 판정이 아니다.
    """
    if path is None:
        return dict(supported=False, reason='no_path')
    p = np.asarray(path, dtype=float)
    if p.ndim != 2 or p.shape[1] != 2 or len(p) < 5 or not np.all(np.isfinite(p)):
        return dict(supported=False, reason='invalid_path')
    movement = max(.01, distance-config.standstill_gap_m)
    queries = envelope_poses(steering, movement, config, step=.01)[:, 1:3]
    # 관측 경로가 원점까지 항상 있지는 않으므로 첫 관측 구간 전은 외삽하지 않는다.
    queries = queries[queries[:, 0] >= p[0, 0]]
    if len(queries) < 3:
        return dict(supported=False, reason='path_overlap_too_short')
    delta, steps = queries[:, None]-p[:-1][None], np.diff(p, axis=0)
    length2 = np.sum(steps*steps, axis=1)
    if np.any(length2 <= 1e-12):
        return dict(supported=False, reason='degenerate_path')
    raw = np.sum(delta*steps[None], axis=2)/length2
    projections = p[:-1][None] + np.clip(raw, 0., 1.)[:, :, None]*steps[None]
    distances = np.linalg.norm(queries[:, None]-projections, axis=2)
    nearest = np.argmin(distances, axis=1)
    idx = np.arange(len(queries))
    outside = ((nearest == 0) & (raw[idx, nearest] < 0)) | ((nearest == len(steps)-1) & (raw[idx, nearest] > 1))
    maximum = float(np.max(distances[idx, nearest]))
    return dict(supported=bool(not np.any(outside) and maximum <= config.boundary_agreement_m),
                reason='observed_polyline_distance_only', maximum_error_m=maximum,
                queries=len(queries), movement_horizon_m=movement, outside_path_extent=bool(np.any(outside)),
                road_verified=False)


def inspect(scene, available, speed, steering, previous, path, config):
    result = dict(supported=False, runtime_connected=False, driving_authorized=False,
                  continuous_sweep_verified=False, future_cut_in_verified=False)
    if not math.isfinite(speed) or not 0 <= speed <= 1.1 or not source_valid(scene):
        return {**result, 'reason':'invalid_state'}
    distance = required_stopping_distance(speed, .2, scene.t_ref_s-scene.source_first_s, config)
    agreement = path_deviation(path, steering, distance, config)
    result.update(path=agreement, required_distance_m=distance, rate_limited=abs(steering-previous)<=.06+1e-9)
    if not agreement['supported']:
        return {**result, 'reason':'path_disagrees'}
    padding = per_ray_padding(scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)
    angles = sorted(set((steering, previous)))
    clearance = min(swept_limit(scene.occupied_endpoints_xy_m, angle, config.wheelbase_m, horizon=2.5,
                               margin=padding, length=config.length_m, width=config.width_m) for angle in angles)
    result['clearance_m'] = clearance
    if clearance < distance:
        return {**result, 'reason':'current_occupancy'}
    pieces = [sweep_queries(scene, angle, distance, config)[0] for angle in angles]
    queries = np.unique(np.vstack(pieces), axis=0)
    support, current = np.zeros(len(queries), dtype=bool), None
    for number, source in enumerate(available):
        accepted, _, _ = lateral_support(source, queries, config)
        support |= accepted
        if number == 0:
            current = int(support.sum())
        if support.all():
            break
    return {**result, 'supported':bool(support.all()),
        'reason':'sampled_observable_choice' if support.all() else 'sweep_unobserved',
        'current_supported_queries':current, 'all_supported_queries':int(support.sum()), 'queries':len(queries)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = digest(args.capture)
    sources = {}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            path = Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                sources[path.relative_to(ROOT).as_posix()] = digest(path)
    sources['tests/test_observable_steering.py'] = digest(ROOT/'tests/test_observable_steering.py')
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for filename in sources:
            archive.write(ROOT/filename, filename)
    cache, last, rows = RecentScenes(), None, []
    with args.capture.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                config = FollowConfig(**event['data']['follow_config'])
            if event['kind'] != 'call':
                continue
            data, scan, history = restore_call(event['data']['input'], motion)
            key = (data['epoch'], scan.header.stamp.sec, scan.header.stamp.nanosec)
            if key == last:
                continue
            last = key
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                 epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            available, _ = cache.observe(scene, history)
            if event['data']['output'][2]['reason'] != 'follow_front_unobserved':
                continue
            path, info = local_path(scene.occupied_endpoints_xy_m, data['side'], config.wall_offset_m)
            cases = []
            for steering in (-.06, -.03, 0., .03, .06, .09, .12, .15, .18):
                result = inspect(scene, available, data['speed_mps'], steering,
                                 data['previous_steering_rad'], path, config)
                cases.append(dict(steering_rad=steering, result=result))
            rows.append(dict(seq=event['seq'], time_s=data['reference_s'], speed_mps=data['speed_mps'],
                previous_steering_rad=data['previous_steering_rad'], path_info=info, cases=cases))
            if len(rows) == 8:
                break
    report = dict(scope='saved_observable_steering_choices_only', rows=rows, input_sha256=before,
        source_sha256=sources, input_unchanged=before==digest(args.capture),
        source_unchanged=all(digest(ROOT/name)==h for name,h in sources.items()), runtime_changed=False,
        boundary_confirmation_verified=False, driving_authorized=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(rows=len(rows), supported_by_row=[
        [c['steering_rad'] for c in r['cases'] if c['result']['supported'] and c['result']['rate_limited']] for r in rows],
        input_unchanged=report['input_unchanged'], source_unchanged=report['source_unchanged'])))


if __name__ == '__main__':
    main()
