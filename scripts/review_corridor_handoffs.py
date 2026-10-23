#!/usr/bin/env python3
"""만료되어 인증하지 못한 전환을 설명용으로만 회수한다. TTL/출력 변경 없음."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import zipfile

import numpy as np

from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import transport_points
from audit_corridor_runtime import digest

ROOT = Path(__file__).resolve().parents[1]


def geometric_difference(first, second):
    """기존 path_agrees와 같은 공통 구간의 수치만. 시각 적합성은 보장하지 않는다."""
    paths = [np.asarray(p, dtype=float) for p in (first, second)]
    if any(p.ndim != 2 or p.shape[1] != 2 or len(p) < 2 or not np.all(np.isfinite(p)) for p in paths):
        raise ValueError('finite 2D paths required')
    paths = [p[(p[:, 0] >= .05) & (p[:, 0] <= .65)] for p in paths]
    if any(len(p) < 5 or np.ptp(p[:, 0]) < .25 for p in paths):
        return dict(evaluated=False, reason='insufficient_near_points_or_extent')
    lo, hi = max(p[:, 0].min() for p in paths), min(p[:, 0].max() for p in paths)
    paths = [p[(p[:, 0] >= lo-1e-12) & (p[:, 0] <= hi+1e-12)] for p in paths]
    if hi-lo < .25 or any(len(p) < 5 or np.ptp(p[:, 0]) < .25 for p in paths):
        return dict(evaluated=False, reason='insufficient_shared_observation')
    distances = np.linalg.norm(paths[0][:, None]-paths[1][None, :], axis=2)
    return dict(evaluated=True, common_x_extent_m=float(hi-lo),
                symmetric_p80_m=float(max(np.percentile(distances.min(axis=0), 80),
                                           np.percentile(distances.min(axis=1), 80))),
                symmetric_maximum_m=float(max(distances.min(axis=0).max(), distances.min(axis=1).max())))


def inspect_handoff(before, after, between, motion, config):
    old = thaw(before['data'])
    inputs, _, history = restore_call(after['data']['input'], motion)
    current = thaw(after['data']['output'])
    previous = old['output']
    age = inputs['reference_s']-previous[2]['path_source_s']
    carried = transport_points(previous[2]['path_points'], history, old['input']['reference_s'], inputs['reference_s'])
    steps = [thaw(event['data']) for event in between if event['kind'] == 'call']
    return dict(previous_seq=before['seq'], seq=after['seq'], previous_reference_s=old['input']['reference_s'],
        reference_s=inputs['reference_s'], previous_source_age_s=age, original_ttl_s=config.memory_ttl_s,
        within_original_ttl=0 <= age < config.memory_ttl_s,
        diagnostic_geometry=geometric_difference(carried, current[2]['path_points']),
        carried_old_path=carried.tolist(), current_path=current[2]['path_points'],
        previous_steering_rad=previous[1], current_steering_rad=current[1],
        direct_steering_difference_rad=current[1]-previous[1],
        intervening_command_count=len(steps), intervening_zero_commands=sum(row['output'][0] == 0 for row in steps),
        intervening_reasons=dict(Counter(row['output'][2]['reason'] for row in steps)),
        measured_speed_before_mps=old['input']['speed_mps'], measured_speed_after_mps=inputs['speed_mps'],
        expired_path_reauthorized=False, continuity_certified=False, actual_body_stop_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('audit', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {str(p): digest(p) for p in (args.capture, args.audit)}
    audit = json.loads(args.audit.read_text())
    if not audit['capture_valid'] or not audit['input_unchanged'] or audit['input_sha256'] != hashes[str(args.capture)]:
        raise ValueError('audit and complete original capture must agree')
    transitions = audit['handoffs']
    selected = {}
    with args.capture.open() as stream:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'header':
                motion = MotionConfig(**event['data']['motion_config'])
                config = FollowConfig(**event['data']['follow_config'])
            if any(row['previous_seq'] <= event['seq'] <= row['seq'] for row in transitions):
                selected[event['seq']] = event
    results = [inspect_handoff(selected[row['previous_seq']], selected[row['seq']],
        [selected[seq] for seq in sorted(selected) if row['previous_seq'] < seq < row['seq']],
        motion, config) for row in transitions]
    sources = {}
    for module in tuple(sys.modules.values()):
        location = getattr(module, '__file__', None)
        if location:
            path = Path(location).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                sources[str(path.relative_to(ROOT))] = digest(path)
    test = ROOT/'tests/test_corridor_handoff_review.py'
    sources[str(test.relative_to(ROOT))] = digest(test)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in sources:
            archive.write(ROOT/path, path)
    report = dict(scope='expired_transition_explanation_not_acceptance_or_runtime_policy', rows=results,
        input_sha256=hashes, input_unchanged=all(digest(p) == h for p, h in hashes.items()),
        source_sha256=sources, source_unchanged=all(digest(ROOT/p) == h for p, h in sources.items()),
        original_audit_verdict_unchanged=True, runtime_changed=False, ttl_changed=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps([{k: v for k, v in row.items() if k not in ('carried_old_path', 'current_path')} for row in results]))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
