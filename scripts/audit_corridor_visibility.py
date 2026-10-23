#!/usr/bin/env python3
"""회랑 경로 외곽과 인접 광선 깊이의 사후 대조. 자유 공간/주행 허가 생성 아님."""
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
from probe_corner_corridor import ROOT, digest, shadow_scene
from probe_corner_temporal import baseline_matches


def bracket_depth(scene, queries, padding, maximum_gap_rad=.025):
    """질의 방향 양옆의 원래 광선이 모두 더 멀리 반환됐는지 확인한다.

    서로 다른 취득 원점에서 질의 방향을 구하며 source_index 이웃만 대조한다.
    두 광선 사이 공간을 채우거나 가림 뒤를 free로 분류하지 않는다. 통과도
    '인접 광선 깊이 지지'일 뿐 표본 사이 작은 물체의 부재를 증명하지 않는다.
    .025 rad는 500광선 진단의 약 두 배 각 간격 상한이며 런타임 문턱이 아니다.
    """
    q = np.asarray(queries, dtype=float)
    pad = np.asarray(padding, dtype=float)
    if (q.ndim != 2 or q.shape[1] != 2 or not np.all(np.isfinite(q)) or
            not math.isfinite(maximum_gap_rad) or not 0 < maximum_gap_rad <= .025 or
            pad.shape != (len(scene.rays),) or not np.all(np.isfinite(pad)) or np.any(pad < 0)):
        raise ValueError('finite query pairs and nonnegative per-ray padding required')
    if not scene.rays:
        return [dict(supported=False, reason='no_rays') for _ in q]
    origins = np.asarray([r.ray_origin_xy_m for r in scene.rays])
    directions = np.asarray([r.direction_xy for r in scene.rays])
    delta = q[:, None, :] - origins[None, :, :]
    radial = np.linalg.norm(delta, axis=2)
    angle = np.arctan2(directions[None, :, 0]*delta[:, :, 1]-directions[None, :, 1]*delta[:, :, 0],
                       np.sum(directions[None]*delta, axis=2))
    lower = np.argmin(np.where(angle >= -1e-10, np.maximum(angle, 0.), np.inf), axis=1)
    upper = np.argmin(np.where(angle <= 1e-10, np.maximum(-angle, 0.), np.inf), axis=1)
    rows = []
    for k, (a, b) in enumerate(zip(lower, upper)):
        if np.any(radial[k] <= 1e-9):
            rows.append(dict(supported=False, reason='query_at_ray_origin',
                             query_xy_m=q[k].tolist(), free_area_certified=False))
            continue
        rays = [scene.rays[int(i)] for i in (a, b)]
        span = float(abs(angle[k, a])+abs(angle[k, b]))
        detail = dict(supported=False, source_indices=[r.source_index for r in rays],
                      acquired_at_s=[r.acquired_at_s for r in rays], angular_span_rad=span,
                      query_xy_m=q[k].tolist(), free_area_certified=False)
        if (angle[k, a] < -1e-10 or angle[k, b] > 1e-10 or span > maximum_gap_rad or
                abs(rays[0].source_index-rays[1].source_index) > 1 or np.any(radial[k, [a, b]] < 1e-9)):
            rows.append({**detail, 'reason':'angular_bracket_unavailable'})
            continue
        if any(r.status != 'return' for r in rays):
            rows.append({**detail, 'reason':'bracket_unknown_or_masked'})
            continue
        distances = [math.dist(r.ray_origin_xy_m, r.hit_xy_m) for r in rays]
        margins = [distances[j]-float(radial[k, i])-float(pad[i]) for j, i in enumerate((a, b))]
        supported = min(margins) > 0
        rows.append({**detail, 'supported':supported, 'depth_margins_m':margins,
                     'reason':'adjacent_ray_depth_supported' if supported else 'return_at_or_before_query_margin'})
    return rows


def footprint_queries(path, config):
    """경로 접선 방향 20x15 cm 외곽의 이산 경계 표본. 추종/연속 swept 모델 아님."""
    p = np.asarray(path, dtype=float)
    if (p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.all(np.isfinite(p))):
        raise ValueError('finite path required')
    tangent = np.gradient(p, axis=0)
    norm = np.linalg.norm(tangent, axis=1)
    if np.any(norm < 1e-9) or np.any(np.linalg.norm(np.diff(p, axis=0), axis=1) < 1e-9):
        raise ValueError('nondegenerate path required')
    tangent /= norm[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    lx, wy = config.length_m/2, config.width_m/2
    border = np.unique(np.vstack(([(x, y) for x in (-lx, lx) for y in np.linspace(-wy, wy, 7)],
                                  [(x, y) for x in np.linspace(-lx, lx, 9) for y in (-wy, wy)])), axis=0)
    origin = int(np.argmin(np.linalg.norm(p, axis=1)))
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    rows = []
    for i in range(origin, len(p)):
        q = p[i]+border[:, 0, None]*tangent[i]+border[:, 1, None]*normal[i]
        # 현재 명목 차체 내부는 센서 가시성 감사 대상이 아니다. free로 간주하지 않는다.
        existing_body = (np.abs(q[:, 0]) <= lx+1e-10) & (np.abs(q[:, 1]) <= wy+1e-10)
        rows.append((i, float(arc[i]-arc[origin]), q[~existing_body], int(existing_body.sum())))
    return rows


def path_visibility(scene, path, config):
    origin_age = scene.t_ref_s-scene.source_first_s
    # return 외 unknown에도 같은 나이/측정 여유를 계산하되 깊이 지지로 채택하지 않는다.
    padding = [.025+config.measurement_padding_m+config.object_motion_bound_mps*(scene.t_ref_s-r.acquired_at_s
                if config.occupancy_timing_mode == 'per_ray_candidate' else origin_age) for r in scene.rays]
    frames = footprint_queries(path, config)
    counts = Counter()
    prefix = 0.
    prefix_open = True
    first_failure = None
    checked = omitted = 0
    for index, distance, query, excluded in frames:
        omitted += excluded
        results = bracket_depth(scene, query, padding)
        checked += len(results)
        counts.update(r['reason'] for r in results)
        failure = next((r for r in results if not r['supported']), None)
        if failure is not None and first_failure is None:
            first_failure = dict(path_index=index, path_distance_m=distance, ray_evidence=failure)
        if failure is not None:
            prefix_open = False
        if prefix_open:
            prefix = distance
    return dict(path_frames=len(frames), query_samples=checked, current_body_queries_excluded=omitted,
                depth_supported_prefix_m=prefix, reasons=dict(counts), first_unsupported=first_failure,
                whole_path_depth_supported=first_failure is None,
                between_ray_volume_observed=False, free_area_certified=False,
                continuous_sweep_verified=False, tracking_dynamics_verified=False,
                driver_or_sensor_error_calibrated=False, driving_authorized=False)


def inspect_events(events, stateless, temporal):
    scenes = {r['seq']:r for r in stateless['rows'] if r['shadow']['geometry_supported']}
    positive = {r['seq']:r for r in temporal['rows'] if r['shadow']['temporal_candidate']}
    selected = set(scenes) | set(positive)
    config = motion = None
    results = []
    for event in events:
        data = event['data']
        if event['kind'] == 'header':
            config, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
        if event['kind'] != 'call' or event['seq'] not in selected:
            continue
        inputs, scan, history = restore_call(data['input'], motion)
        scene = motion_scene(scan, history, inputs['reference_s'], timing_verified=inputs['timing_verified'],
                             epoch=inputs['epoch'], rear_half_angle_deg=inputs['rear_half_angle_deg'])
        proposal = shadow_scene(scene, inputs['speed_mps'], inputs['side'], inputs['previous_steering_rad'],
                                inputs['dt_s'], config)
        if not proposal['geometry_supported']:
            raise ValueError('previous geometry is not reproduced')
        if event['seq'] in positive and not proposal['preliminary_motion_supported']:
            raise ValueError('previous temporal proposal is not reproduced')
        audit = path_visibility(scene, proposal['geometry']['path'], config)
        speed = proposal['proposed_speed_mps']
        # 현행 점유 제동식의 역산. 곡선 외곽 가시성과 rear-axle swept 거리는 다른 양이다.
        # 따라서 이 대조로 속도 cap을 만들지 않고 필요한 후속 판단만 표시한다.
        required = config.standstill_gap_m+.06+speed*(scene.t_ref_s-scene.source_first_s+.10)+speed*speed/4
        results.append(dict(seq=event['seq'], time_s=inputs['reference_s'],
            original_hold=data['output'][0] == 0, temporal_candidate=event['seq'] in positive,
            stateless_fresh_scene=event['seq'] in scenes, proposed_speed_mps=speed,
            visibility=audit, existing_stopping_distance_proxy_m=required,
            prefix_exceeds_stopping_proxy=audit['depth_supported_prefix_m'] >= required,
            distance_semantics_identical=False))
    if len(results) != len(selected):
        raise ValueError('selected saved calls missing')
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--stateless', type=Path, required=True)
    parser.add_argument('--temporal', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    inputs = {str(p):digest(p) for p in (args.input, args.stateless, args.temporal)}
    stateless, temporal = [json.loads(p.read_text()) for p in (args.stateless, args.temporal)]
    baseline_matches(stateless, args.input)
    if (temporal['input_sha256'] != digest(args.input) or not temporal['input_unchanged'] or
            not temporal['source_unchanged'] or temporal['runtime_changed'] is not False or
            temporal['baseline_sha256'] != digest(args.stateless)):
        raise ValueError('temporal provenance mismatch')
    sources = {**temporal['source_sha256'], 'scripts/audit_corridor_visibility.py':digest(Path(__file__)),
               'tests/test_corridor_visibility.py':digest(ROOT/'tests/test_corridor_visibility.py')}
    for path, expected in sources.items():
        if digest(ROOT/path) != expected:
            raise ValueError(f'source mismatch: {path}')
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in sources:
            archive.write(ROOT/path, path)
    with args.input.open() as stream:
        rows = inspect_events((json.loads(line) for line in stream), stateless, temporal)
    candidates = [r for r in rows if r['temporal_candidate']]
    summary = dict(calls=len(rows), temporal_candidates=len(candidates),
        temporal_whole_path_supported=sum(r['visibility']['whole_path_depth_supported'] for r in candidates),
        temporal_prefix_over_065=sum(r['visibility']['depth_supported_prefix_m'] >= .65 for r in candidates),
        temporal_prefix_over_stop_proxy=sum(r['prefix_exceeds_stopping_proxy'] for r in candidates),
        original_hold_candidates=sum(r['original_hold'] for r in candidates))
    result = dict(scope='adjacent_ray_depth_audit_not_free_space_certificate', summary=summary, rows=rows,
        input_sha256=inputs, source_sha256=sources,
        input_unchanged=all(digest(p) == h for p, h in inputs.items()),
        source_unchanged=all(digest(ROOT/p) == h for p, h in sources.items()),
        runtime_changed=False, unknown_gaps_filled=False, driving_authorized=False,
        between_ray_volume_observed=False, closed_loop_verified=False)
    with (args.output/'report.json').open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
    print(json.dumps({**summary, 'input_unchanged':result['input_unchanged'], 'source_unchanged':result['source_unchanged']}))
    return 0 if result['input_unchanged'] and result['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
