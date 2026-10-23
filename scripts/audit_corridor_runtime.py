#!/usr/bin/env python3
"""실제 선택형 기록의 채택 조건·경로 전환·광선 깊이를 사후 대조한다.

판단 재생/공식 완주/연속 무충돌/실차 제동의 판정 도구가 아니다.
기존 이산 깊이 검사와 운동 이송을 재사용하며 운영 출력은 바꾸지 않는다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np

from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, path_agrees
from arena_autonomy.scene_motion import motion_scene, transport_points
from audit_corridor_sweep import required_stopping_distance, sweep_visibility

ROOT = Path(__file__).resolve().parents[1]


def selection_contract(output, inputs, config):
    speed, steering, detail = output
    meta = detail.get('corridor_candidate', {})
    age = inputs['reference_s'] - meta.get('source_first_s', math.inf)
    return {
        'explicit_option': config.corridor_fallback_enabled,
        'boundary_hold_only': detail.get('underlying_reason') == 'follow_boundary_unconfirmed',
        'selected_reason': detail.get('reason') == 'local_follow_corridor_candidate',
        'positive_confirmed': meta.get('positive') is True,
        'distinct_geometry': meta.get('confirmations', 0) >= config.confirmations,
        'distinct_clearance': meta.get('release_count', 0) >= config.confirmations,
        'source_age': math.isfinite(age) and 0 <= age < config.memory_ttl_s,
        'forward_bounded_output': 0 < speed <= config.max_speed_mps,
        'bounded_steering': abs(steering) <= config.max_steering_rad,
        'reported_all_occupancy': detail.get('unfiltered_occupancy') is True,
        'reported_no_wall_identity': detail.get('wall_identity_certified') is False,
        'reported_no_memory_refresh': detail.get('boundary_memory_from_corridor') is False,
    }


def handoff_check(previous, current, history, config):
    result = dict(from_mode=previous['mode'], to_mode=current['mode'],
                  previous_seq=previous['seq'], seq=current['seq'],
                  time_s=current['time_s'], evaluated=False, path_agrees=None)
    if previous['key'] != current['key']:
        return {**result, 'reason': 'frame_epoch_or_side_changed'}
    age = current['time_s'] - previous['source_s']
    if not 0 <= age < config.memory_ttl_s:
        return {**result, 'reason': 'previous_path_expired', 'previous_age_s': age}
    try:
        carried = transport_points(previous['path'], history, previous['time_s'], current['time_s'])
        displacement = float(np.max(np.linalg.norm(carried-previous['path'], axis=1)))
        if displacement > config.memory_motion_limit_m:
            return {**result, 'reason': 'transport_limit', 'displacement_m': displacement}
        # False에는 경로 간 큰 차이 외에도 공통 관측 길이/표본 부족이 포함된다.
        paths = [p[(p[:, 0] >= .05) & (p[:, 0] <= .65)] for p in (current['path'], carried)]
        shared = None
        if all(len(p) for p in paths):
            shared = max(0., min(float(p[:, 0].max()) for p in paths)-max(float(p[:, 0].min()) for p in paths))
        return {**result, 'evaluated': True, 'reason': 'measured_motion_comparison',
                'path_agrees': bool(path_agrees(current['path'], carried, config.boundary_agreement_m)),
                'previous_age_s': age, 'displacement_m': displacement,
                'near_path_points': [len(p) for p in paths], 'shared_x_extent_m': shared,
                'false_may_include_insufficient_overlap': True}
    except (MotionUnavailable, ValueError) as error:
        return {**result, 'reason': 'transport_unavailable', 'error': str(error)}


def inspect_events(events):
    iterator = iter(events)
    header = next(iterator, None)
    if (not header or header.get('kind') != 'header' or header.get('seq') != 0
            or header['data'].get('schema') != 1
            or header['data'].get('scope') != 'g1_decision_stage_only'):
        raise ValueError('complete known header required')
    config = FollowConfig(**header['data']['follow_config'])
    motion = MotionConfig(**header['data']['motion_config'])
    if not config.corridor_fallback_enabled:
        raise ValueError('audit requires explicitly enabled corridor capture')
    calls = 0
    previous = footer = None
    expected = 1
    reasons, candidates = Counter(), Counter()
    selected, transitions, state_changes = [], [], []
    last_state = None
    for event in iterator:
        if footer is not None or event.get('seq') != expected:
            raise ValueError('trace sequence gap or trailing records')
        expected += 1
        kind, data = event['kind'], thaw(event['data'])
        if kind == 'reset':
            previous = None
            continue
        if kind == 'footer':
            footer = data
            continue
        if kind != 'call':
            raise ValueError('unknown trace event')
        calls += 1
        inputs, output = data['input'], data['output']
        speed, steering, detail = output
        chosen = detail.get('corridor_selected') is True
        candidate = detail.get('corridor_candidate', {})
        reasons[detail['reason']] += 1
        candidates[candidate.get('reason', 'not_evaluated')] += 1
        state = (detail['reason'], chosen, candidate.get('reason'))
        if state != last_state:
            state_changes.append(dict(seq=event['seq'], time_s=inputs['reference_s'], reason=state[0],
                                      selected=chosen, candidate_reason=state[2]))
            last_state = state
        if speed <= 0 and not chosen:
            continue
        inputs, scan, history = restore_call(data['input'], motion)
        path = np.asarray(detail.get('path_points'), dtype=float)
        if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2 or not np.all(np.isfinite(path)):
            raise ValueError('positive or selected output requires a finite recorded path')
        current = dict(seq=event['seq'], time_s=inputs['reference_s'], path=path,
                       source_s=detail['path_source_s'], mode='corridor' if chosen else 'baseline',
                       key=(scan.header.frame_id, inputs['epoch'], inputs['side']))
        if previous is not None and previous['mode'] != current['mode']:
            transitions.append(handoff_check(previous, current, history, config))
        previous = current
        if chosen:
            checks = selection_contract(output, inputs, config)
            scene = motion_scene(scan, history, inputs['reference_s'], timing_verified=inputs['timing_verified'],
                                 epoch=inputs['epoch'], rear_half_angle_deg=inputs['rear_half_angle_deg'])
            needed = required_stopping_distance(inputs['speed_mps'], speed,
                                                inputs['reference_s']-scene.source_first_s, config)
            row = dict(seq=event['seq'], time_s=inputs['reference_s'], checks=checks,
                       speed_mps=speed, steering_rad=steering, required_distance_m=needed,
                       selected_contract_passed=all(checks.values()), depth_evaluated=needed <= 2.5)
            if needed <= 2.5:
                row['sweeps'] = [sweep_visibility(scene, angle, needed, config)
                                 for angle in (inputs['previous_steering_rad'], steering)]
                row['both_depth_supported'] = all(s['stopping_sweep_depth_supported'] for s in row['sweeps'])
            else:
                row['both_depth_supported'] = None
                row['depth_reason'] = 'sweep_horizon_exceeded'
            selected.append(row)
    if (footer is None or calls == 0 or any(footer.get(k) != 0 for k in
            ('capture_errors', 'failed', 'dropped', 'pending')) or footer.get('worker_alive') is not False
            or footer.get('accepted') != expected-1 or footer.get('written') != expected-1):
        raise ValueError('complete nonempty capture and intact footer required')
    summary = dict(calls=calls, selected_calls=len(selected),
        selected_contract_failures=sum(not r['selected_contract_passed'] for r in selected),
        depth_evaluated=sum(r['depth_evaluated'] for r in selected),
        both_depth_supported=sum(r['both_depth_supported'] is True for r in selected),
        handoffs=len(transitions), handoffs_evaluated=sum(r['evaluated'] for r in transitions),
        handoff_disagreements=sum(r['path_agrees'] is False for r in transitions))
    return dict(scope='recorded_selection_handoff_and_sampled_ray_depth_only', summary=summary,
        reason_counts=dict(reasons), candidate_reason_counts=dict(candidates), selected=selected,
        handoffs=transitions, state_changes=state_changes, capture_valid=True,
        complete_lap_verified=False, continuous_sweep_verified=False,
        independent_braking_verified=False, runtime_changed=False)


def digest(path):
    result = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = digest(args.input)
    sources = {}
    for module in tuple(sys.modules.values()):
        location = getattr(module, '__file__', None)
        if location:
            path = Path(location).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                sources[str(path.relative_to(ROOT))] = digest(path)
    test = ROOT/'tests/test_corridor_runtime_audit.py'
    sources[str(test.relative_to(ROOT))] = digest(test)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in sources:
            archive.write(ROOT/path, path)
    with args.input.open() as stream:
        report = inspect_events(json.loads(line) for line in stream)
    report.update(input_sha256=before, input_unchanged=before == digest(args.input),
                  source_sha256=sources, source_unchanged=all(digest(ROOT/p) == h for p, h in sources.items()))
    (args.output/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False))
    print(json.dumps({**report['summary'], 'input_unchanged': report['input_unchanged'],
                      'source_unchanged': report['source_unchanged']}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
