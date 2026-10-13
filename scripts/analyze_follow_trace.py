#!/usr/bin/env python3
"""동일 G1 기록을 계산하면서 경계 후보/일치 검사 경위를 별도로 회수한다.

운영 코드를 수정하지 않는 오프라인 계측. 후보 검사만 래핑하고 원래 반환값을
유지한다. 기본 재생이 전부 일치하지 않으면 이 분석을 유효하다고 표시하지 않는다.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np

import arena_autonomy.local_follow as lf
from arena_autonomy.follow_trace import equal_output, freeze, replay_call, replay_events
from arena_autonomy.lidar_motion import MotionConfig


def agreement_distance(candidate, reference):
    paths = [np.asarray(p)[(np.asarray(p)[:, 0] >= .05) & (np.asarray(p)[:, 0] <= .65)]
             for p in (candidate, reference)]
    if any(len(p) < 5 or np.ptp(p[:, 0]) < .25 for p in paths):
        return None
    lo = max(float(np.min(p[:, 0])) for p in paths)
    hi = min(float(np.max(p[:, 0])) for p in paths)
    if hi-lo < .25:
        return None
    paths = [p[(p[:, 0] >= lo-1e-12) & (p[:, 0] <= hi+1e-12)] for p in paths]
    if any(len(p) < 5 or np.ptp(p[:, 0]) < .25 for p in paths):
        return None
    distances = np.linalg.norm(paths[0][:, None, :]-paths[1][None, :, :], axis=2)
    return float(max(np.percentile(np.min(distances, axis=0), 80),
                     np.percentile(np.min(distances, axis=1), 80)))


def agreement_support(candidate, reference):
    """비교 전 관측 범위와 실제 공통 표본의 개수/길이를 설명한다."""
    paths = [np.asarray(p)[(np.asarray(p)[:, 0] >= .05) & (np.asarray(p)[:, 0] <= .65)]
             for p in (candidate, reference)]
    def extent(p):
        return {'count': len(p), 'min_x': float(np.min(p[:, 0])) if len(p) else None,
                'max_x': float(np.max(p[:, 0])) if len(p) else None,
                'span_x': float(np.ptp(p[:, 0])) if len(p) else None}
    result = {'near': [extent(p) for p in paths]}
    if any(len(p) == 0 for p in paths):
        return {**result, 'common': [], 'support_sufficient': False}
    lo, hi = max(np.min(p[:, 0]) for p in paths), min(np.max(p[:, 0]) for p in paths)
    common = [p[(p[:, 0] >= lo-1e-12) & (p[:, 0] <= hi+1e-12)] for p in paths]
    return {**result, 'common': [extent(p) for p in common],
            'support_sufficient': bool(hi-lo >= .25 and all(len(p) >= 5 and np.ptp(p[:, 0]) >= .25 for p in common))}


def analyze(events):
    baseline = replay_events(events)
    if not baseline['passed']:
        return {'passed': False, 'error': 'baseline_replay_mismatch', 'mismatches': baseline['mismatches']}
    header = events[0]['data']
    cfg = lf.FollowConfig(**header['follow_config'])
    follow, motion = lf.LocalFollow(cfg), MotionConfig(**header['motion_config'])
    original_path, original_agree, original_transport = lf.local_path, lf.path_agrees, lf.transport_points
    probes, rows, mismatches = [], [], []

    def local_path(*args, **kwargs):
        path, details = original_path(*args, **kwargs)
        ahead, distance = details.get('boundary_observed_ahead_m'), details.get('wall_distance_m')
        probes.append({'kind': 'candidate', 'side': args[1], 'input_points': len(args[0]),
            'reason': details['reason'], 'ahead_m': ahead, 'wall_distance_m': distance,
            'anchor_ok': path is not None and abs(distance-cfg.wall_offset_m) <= cfg.boundary_anchor_error_m,
            'length_ok': path is not None and ahead >= cfg.boundary_min_ahead_m})
        return path, details

    def agrees(a, b, tolerance):
        result = original_agree(a, b, tolerance)
        probes.append({'kind': 'agreement', 'passed': bool(result),
                       'distance_m': agreement_distance(a, b), 'tolerance_m': tolerance,
                       **agreement_support(a, b)})
        return result

    def transport(points, history, start, end):
        output = original_transport(points, history, start, end)
        probes.append({'kind': 'memory_transport', 'from_s': start, 'to_s': end,
                       'estimated_max_point_motion_m': float(np.max(np.linalg.norm(output-points, axis=1)))})
        return output

    with patch.object(lf, 'local_path', local_path), patch.object(lf, 'path_agrees', agrees), patch.object(lf, 'transport_points', transport):
        for event in events[1:]:
            if event['kind'] == 'reset':
                follow.reset()
            elif event['kind'] == 'call':
                probes = []
                data = event['data']
                before = {'path_source_s': follow.path_source, 'path_ref_s': follow.path_ref,
                          'held': follow.held, 'pending_count': follow.pending_count,
                          'release_count': follow.release_count, 'last_source_s': follow.last_source}
                result = freeze(replay_call(follow, data['input'], motion))
                if not equal_output(result, data['output']):
                    mismatches.append(event['seq'])
                rows.append({'seq': event['seq'], 'reference_s': data['input']['reference_s'],
                    'measured_speed_mps': data['input']['speed_mps'], 'before': before,
                    'reason': result[2]['reason'], 'behavior': result[2]['behavior'],
                    'speed_target_mps': result[0], 'steering_target_rad': result[1],
                    'selection': result[2].get('boundary_selection'),
                    'accepted': result[2].get('fresh_boundary_accepted'),
                    'boundary_age_s': result[2].get('boundary_age_s'),
                    'clearance_m': result[2].get('swept_clearance_m'), 'probes': probes})
    changes = [r for i, r in enumerate(rows) if i == 0 or r['reason'] != rows[i-1]['reason']]
    return {'passed': not mismatches, 'scope': 'offline_boundary_decision_instrumentation_not_closed_loop',
            'calls': len(rows), 'baseline_matched': baseline['matched'], 'instrumented_mismatches': mismatches,
            'reason_counts': dict(Counter(r['reason'] for r in rows)), 'changes': changes, 'rows': rows}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    before = hashlib.sha256(args.input.read_bytes()).hexdigest()
    with args.input.open(encoding='utf-8') as stream:
        events = [json.loads(line) for line in stream]
    result = analyze(events)
    result.update(input_sha256=before, input_unchanged=before == hashlib.sha256(args.input.read_bytes()).hexdigest(),
                  analyzer_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'changes')}, ensure_ascii=False))
    return 0 if result['passed'] and result['input_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
