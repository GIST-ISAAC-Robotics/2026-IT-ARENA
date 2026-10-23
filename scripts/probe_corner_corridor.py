#!/usr/bin/env python3
"""긴 바깥 관측 + 의미 미확정 안쪽 표면의 shadow 대조. 주행 출력 연결 없음."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np

from arena_autonomy.follow_trace import replay_events, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.local_path import local_path, swept_limit, braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene
from inspect_corner_geometry import components
from probe_short_corner_support import point_support
from profile_follow_trace import SNAPSHOT_MODULES

ROOT = Path(__file__).resolve().parents[1]


def paired_geometry(points, side, config):
    """현재 관측의 경로 가설만 생성한다. 안쪽 조각을 벽으로 분류하지 않는다."""
    base = dict(geometry_supported=False, reason='invalid_input', path=None,
                inner_identity='unknown', wall_identity_certified=False,
                memory_refresh_authorized=False, free_area_certified=False,
                driving_authorized=False, unknown_gaps_filled=False)
    p = np.asarray(points, dtype=float)
    if (side not in ('left', 'right') or p.ndim != 2 or p.shape[1] != 2 or
            len(p) < 30 or not np.all(np.isfinite(p))):
        return base
    other = 'right' if side == 'left' else 'left'
    path, info = local_path(p, other, config.wall_offset_m)
    base.update(outer_side=other, outer_info=info)
    if (path is None or info['boundary_observed_ahead_m'] < config.boundary_min_ahead_m or
            abs(info['wall_distance_m']-config.wall_offset_m) > config.boundary_anchor_error_m):
        return {**base, 'reason': 'long_outer_reference_unavailable'}
    tangent = np.gradient(path, axis=0)
    if np.any(np.linalg.norm(tangent, axis=1) <= 1e-9):
        return {**base, 'reason': 'degenerate_outer_path'}
    angles = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    origin = int(np.argmin(np.linalg.norm(path, axis=1)))
    sign = 1 if side == 'left' else -1
    turn = float(sign*(angles[-1]-angles[origin]))
    opposing = float(np.maximum(0., -sign*np.diff(angles[origin:])).sum())
    if (not math.radians(30) <= turn <= math.radians(135) or opposing > math.radians(5)
            or tangent[origin, 0] <= 0):
        return {**base, 'reason': 'not_consistent_inward_corner'}
    witnesses, rejected = [], []
    for part in components(p, side):
        support = point_support(p[part['indices']], path, side,
                                config.wall_offset_m, config.boundary_agreement_m)
        end = origin+support['supported_prefix_samples']
        retained_turn = float(sign*(angles[end-1]-angles[origin])) if end > origin else 0.
        detail = dict(source_point_indices=part['indices'], observed_extent_m=support['observed_extent_m'],
                      prefix_m=support['path_prefix_m'], end_exclusive=end, turn_rad=retained_turn)
        if support['path_prefix_m'] < config.boundary_min_ahead_m:
            rejected.append({**detail, 'reason': 'short_geometric_support'})
        elif retained_turn < math.radians(30):
            rejected.append({**detail, 'reason': 'supported_prefix_not_corner'})
        else:
            witnesses.append(detail)
    if not witnesses:
        return {**base, 'reason': 'inner_surface_support_unavailable', 'rejected': rejected}
    # 두 조각이 모두 같은 경로를 지지하면 더 짧은 공통 prefix만 남긴다.
    # 조각을 합쳐 길이를 늘리거나 모호한 조각 하나를 벽으로 선택하지 않는다.
    end = min(w['end_exclusive'] for w in witnesses)
    retained = path[:end]
    return {**base, 'geometry_supported': True, 'reason': 'paired_geometry_only',
            'path': retained.tolist(), 'witnesses': witnesses, 'rejected': rejected,
            'inner_semantic_hypotheses': ['boundary_surface', 'other_occupied_object'],
            'common_prefix_m': min(w['prefix_m'] for w in witnesses),
            'candidate_count': 1, 'preferred_side': side}


def shadow_scene(scene, speed, side, previous_steering, dt, config):
    """현재 입력의 기하/기존 점유 제한을 분리한다. fresh 확인/기억 채택/명령 발행 없음."""
    base = dict(shadow_only=True, driving_authorized=False, runtime_connected=False,
                confirmation_evaluated=False, confirmations_required=config.confirmations,
                memory_refresh_authorized=False, free_area_certified=False,
                geometry_supported=False, preliminary_motion_supported=False,
                proposed_speed_mps=0., source_first_s=scene.source_first_s,
                reference_s=scene.t_ref_s, source_epoch=scene.source_epoch)
    if (not all(math.isfinite(v) for v in (speed, previous_steering, dt)) or not 0 < dt <= .2):
        return {**base, 'reason': 'invalid_motion_state'}
    if scene.source_first_s is None or not 0 <= scene.t_ref_s-scene.source_first_s < .3:
        return {**base, 'reason': 'scene_stale'}
    front = [r for r in scene.rays if abs(math.atan2(r.direction_xy[1], r.direction_xy[0])) <= math.radians(100)]
    if not front or any(r.status != 'return' for r in front):
        return {**base, 'reason': 'front_unobserved'}
    points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
    geometry = paired_geometry(points, side, config)
    base.update(geometry=geometry, geometry_supported=geometry['geometry_supported'],
                occupied_points=len(points), all_return_points_retained=True)
    if not geometry['geometry_supported']:
        return {**base, 'reason': geometry['reason']}
    age = scene.t_ref_s-scene.source_first_s
    cap, desired, ahead = pursue_path(geometry['path'], speed, config, age)
    steering = float(np.clip(desired, previous_steering-3*dt, previous_steering+3*dt))
    padding = (per_ray_padding(scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)
               if config.occupancy_timing_mode == 'per_ray_candidate' else
               .025+config.measurement_padding_m+config.object_motion_bound_mps*age)
    clearances = [swept_limit(points, angle, config.wheelbase_m, horizon=2.5, margin=padding,
                             length=config.length_m, width=config.width_m)
                  for angle in (steering, previous_steering)]
    clearance = min(clearances)
    available = max(0., clearance-config.standstill_gap_m)
    proposed = min(cap, braking_speed(available, age), available/config.headway_s)
    base.update(steering_rad=steering, previous_steering_rad=previous_steering,
                swept_clearance_m=clearance, steering_clearances_m=clearances,
                point_padding_min_m=float(np.min(padding)), point_padding_max_m=float(np.max(padding)),
                pursuit_cap_mps=cap, observed_path_ahead_m=ahead,
                speed_before_hold_mps=proposed)
    if speed < 0:
        return {**base, 'reason': 'reverse_motion'}
    # 이 가설은 매번 새 획득으로 취급한다. 기존 재출발 여유보다 작으면 진단도 HOLD다.
    if clearance < config.resume_clearance_m:
        return {**base, 'reason': 'existing_occupancy_margin_hold'}
    return {**base, 'reason': 'geometry_and_point_occupancy_only',
            'preliminary_motion_supported': proposed > 0, 'proposed_speed_mps': proposed}


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def inspect_events(events):
    config = motion = None
    last_source = None
    rows = []
    for event in events:
        kind, data = event['kind'], event['data']
        if kind == 'header':
            config, motion = FollowConfig(**data['follow_config']), MotionConfig(**data['motion_config'])
        elif kind == 'reset':
            last_source = None
        elif kind == 'call':
            inputs, scan, history = restore_call(data['input'], motion)
            try:
                scene = motion_scene(scan, history, inputs['reference_s'], timing_verified=inputs['timing_verified'],
                                     epoch=inputs['epoch'], rear_half_angle_deg=inputs['rear_half_angle_deg'])
                source = (scene.source_epoch, scene.source_first_s)
                if source == last_source:
                    continue  # 같은 스캔의 50 Hz 재사용을 새 관측 수로 세지 않는다.
                last_source = source
                result = shadow_scene(scene, inputs['speed_mps'], inputs['side'],
                                      inputs['previous_steering_rad'], inputs['dt_s'], config)
            except (ValueError, MotionUnavailable) as error:
                result = dict(reason='input_rejected', error=str(error), driving_authorized=False,
                              geometry_supported=False, preliminary_motion_supported=False)
            rows.append(dict(seq=event['seq'], time_s=inputs['reference_s'],
                             original_speed_mps=data['output'][0], original_reason=data['output'][2]['reason'],
                             shadow=result))
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = digest(args.input)
    sources = [f'src/{pkg}/{pkg}/{mod}.py' for pkg, mods in SNAPSHOT_MODULES.items() for mod in mods]
    sources += ['scripts/probe_corner_corridor.py', 'scripts/probe_short_corner_support.py',
                'scripts/probe_corner_observability.py', 'scripts/inspect_corner_geometry.py',
                'scripts/profile_follow_trace.py', 'tests/test_corner_corridor_shadow.py']
    hashes = {p: digest(ROOT/p) for p in sources}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in sources:
            archive.write(ROOT/p, p)
    with args.input.open() as stream:
        baseline = replay_events(json.loads(line) for line in stream)
    with (args.output/'baseline_replay.json').open('x') as stream:
        json.dump(baseline, stream, indent=2, allow_nan=False)
    if not baseline['passed']:
        raise RuntimeError('baseline replay mismatch; shadow not evaluated')
    with args.input.open() as stream:
        rows = inspect_events(json.loads(line) for line in stream)
    report = dict(scope='fixed_input_stateless_shadow_not_driving_policy', rows=rows,
                  summary=dict(baseline_calls_matched=baseline['matched'], distinct_scenes=len(rows),
                               geometry_supported=sum(r['shadow']['geometry_supported'] for r in rows),
                               preliminary_motion_supported=sum(r['shadow']['preliminary_motion_supported'] for r in rows),
                               reasons=dict(Counter(r['shadow']['reason'] for r in rows))),
                  input_sha256=before, input_unchanged=digest(args.input)==before,
                  source_sha256=hashes, source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()),
                  runtime_changed=False, memory_refresh_authorized=False, driving_authorized=False,
                  missing_contracts=['two fresh confirmation and memory integration', 'path visibility/road ambiguity',
                                     'closed-loop tracking and future observations', 'hardware braking and motion bounds'])
    with (args.output/'report.json').open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({**report['summary'], 'input_unchanged':report['input_unchanged'],
                      'source_unchanged':report['source_unchanged'], 'driving_authorized':False}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
