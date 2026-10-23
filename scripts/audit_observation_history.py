#!/usr/bin/env python3
"""이전 광선 깊이 지지의 측방 이동 반례 감사. 주행/보호에는 연결하지 않는다.

유한 반환의 깊이 여유만 줄이면 미관측 부채꼴의 옆 경계에서는 같은 여유가
생기지 않는다. 취득 뒤 object_motion_bound_mps로 이동 가능한 미관측 출발점을
따로 검사한다. 현재 명목 차체 내부는 출발점에서 제외하며 unknown은 유지한다.
센서 사이 물체/전체 자유 공간/미래 cut-in/실제 제동을 인증하지 않는다.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np
from shapely.geometry import Point, box
from shapely.ops import nearest_points

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import RecentScenes, depth_support, sweep_queries
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest
from audit_corridor_sweep import required_stopping_distance
from probe_observation_envelope import unknown_sector

ROOT = Path(__file__).resolve().parents[1]


def lateral_support(scene, queries, config):
    """깊이가 지지된 각 질의에 과거 unknown의 측방 도달 가능성을 추가 대조."""
    queries = np.asarray(queries, dtype=float).reshape(-1, 2)
    pad = np.array([.025 + config.measurement_padding_m + config.object_motion_bound_mps *
                    (scene.t_ref_s-r.acquired_at_s) for r in scene.rays])
    support = depth_support(scene, queries, pad)
    reachable = np.zeros(len(queries), dtype=bool)
    examples = []
    if not len(queries):
        return support, reachable, examples
    radius = config.object_motion_bound_mps * (scene.t_ref_s-scene.source_first_s)
    bounds = box(queries[:, 0].min()-radius-.01, queries[:, 1].min()-radius-.01,
                 queries[:, 0].max()+radius+.01, queries[:, 1].max()+radius+.01)
    body = box(-config.length_m/2, -config.width_m/2, config.length_m/2, config.width_m/2)
    points = [Point(q) for q in queries]
    for ray in scene.rays:
        if ray.status == 'return':
            continue
        unknown = unknown_sector(ray, math.pi/500, bounds).intersection(bounds).difference(body)
        if unknown.is_empty:
            continue
        travel = config.object_motion_bound_mps * (scene.t_ref_s-ray.acquired_at_s)
        for index in np.flatnonzero(support & ~reachable):
            distance = unknown.distance(points[index])
            if distance <= travel + 1e-12:
                reachable[index] = True
                if len(examples) < 4:
                    origin = nearest_points(unknown, points[index])[0]
                    examples.append(dict(query_xy_m=queries[index].tolist(), source_index=ray.source_index,
                        status=ray.status, acquired_at_s=ray.acquired_at_s,
                        possible_start_xy_m=[origin.x, origin.y], separation_m=distance,
                        age_motion_bound_m=travel))
    return support & ~reachable, reachable, examples


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-scenes', type=int, default=8, choices=range(1, 9))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = digest(args.capture)
    sources = {}
    for module in tuple(sys.modules.values()):
        filename = getattr(module, '__file__', None)
        if filename:
            path = Path(filename).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                sources[path.relative_to(ROOT).as_posix()] = digest(path)
    for filename in ('tests/test_observation_history.py',):
        sources[filename] = digest(ROOT/filename)
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
            distance = required_stopping_distance(max(0., data['speed_mps']), .2,
                                                  scene.t_ref_s-scene.source_first_s, config)
            cases = []
            for name, angle in [('recorded_previous', data['previous_steering_rad']), ('left_sensitivity', .175417)]:
                queries, _ = sweep_queries(scene, angle, distance, config)
                depth = np.zeros(len(queries), dtype=bool)
                stricter = depth.copy()
                evidence = []
                for source in available:
                    supported, reachable, examples = lateral_support(source, queries, config)
                    # reachable은 원래 깊이가 지지된 점만 포함하므로 합집합으로 원래 결과를 복원한다.
                    depth |= supported | reachable
                    stricter |= supported
                    evidence.append(dict(source_first_s=source.source_first_s,
                        supported=int(supported.sum()), reachable_from_unknown=int(reachable.sum()), examples=examples))
                rejected = queries[depth & ~stricter][:4]
                witnesses = []
                for source in available:
                    if len(rejected):
                        _, _, examples = lateral_support(source, rejected, config)
                        witnesses.append(dict(source_first_s=source.source_first_s, examples=examples))
                cases.append(dict(name=name, queries=len(queries), depth_supported=int(depth.sum()),
                    lateral_supported=int(stricter.sum()), all_depth_supported=bool(depth.all()),
                    all_lateral_supported=bool(stricter.all()), sources=evidence,
                    uncovered_queries_xy_m=rejected.tolist(), uncovered_witnesses=witnesses))
            rows.append(dict(seq=event['seq'], time_s=data['reference_s'], measured_speed_mps=data['speed_mps'], cases=cases))
            if len(rows) == args.max_scenes:
                break
    report = dict(scope='past_unknown_lateral_reachability_only', rows=rows, source_sha256=sources,
        input_sha256=before, input_unchanged=before == digest(args.capture),
        source_unchanged=all(digest(ROOT/name) == h for name, h in sources.items()),
        runtime_changed=False, actual_dynamic_collision_demonstrated=False, future_cut_in_verified=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(rows=len(rows), depth_all=sum(c['all_depth_supported'] for r in rows for c in r['cases']),
        lateral_all=sum(c['all_lateral_supported'] for r in rows for c in r['cases']),
        input_unchanged=report['input_unchanged'], source_unchanged=report['source_unchanged'])))


if __name__ == '__main__':
    main()
