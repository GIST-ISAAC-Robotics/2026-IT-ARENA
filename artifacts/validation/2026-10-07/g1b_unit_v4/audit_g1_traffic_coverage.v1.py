#!/usr/bin/env python3
"""G1-b 고정 런타임을 바꾸지 않는 사후 수신/상태/자세 감사.

행동 판정선을 완화하거나 report.json을 고치지 않는다. 5 Hz/상태변화 발행
autonomy 원문과 10 Hz trace를 대조하고, 밀집 Gazebo 자세로 정지 기록을 교차 확인한다.
원본 dynamics와 safe_drive의 개별 취득 시각은 현재 기록에 없으므로 미확인으로 남긴다.
"""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path


AUDIT_VERSION = 'g1b_post_coverage_v1'


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def kinematic_speed_segments(ego):
    result = []
    for (t0, x0, y0, _), (t1, x1, y1, _) in zip(ego, ego[1:]):
        if not all(finite(v) for v in (t0, x0, y0, t1, x1, y1)) or t1 <= t0:
            raise ValueError('invalid/nonincreasing ego pose')
        result.append({'t0': t0, 't1': t1, 'average_speed_mps': math.hypot(x1-x0, y1-y0)/(t1-t0)})
    return result


def audit_rows(report, trace, statuses, ego, stop_trace):
    first = report.get('green_observed_sim_s')
    stop = report.get('stop_request_sim_s')
    checks = {'green_and_stop_recorded': finite(first) and finite(stop) and stop > first,
              'raw_status_received': bool(statuses), 'dense_ego_received': len(ego) > 1}
    invalid_status = [i for i, r in enumerate(statuses) if not finite(r.get('sim_time_s'))]
    ordered = not invalid_status and all(b['sim_time_s'] > a['sim_time_s'] for a, b in zip(statuses, statuses[1:]))
    checks['raw_status_finite_ordered'] = ordered
    if not all(checks.values()):
        return {'audit_version': AUDIT_VERSION, 'checks': checks, 'passed': False,
                'invalid_status_indices': invalid_status}
    rows = [r for r in trace if first <= r['t'] <= stop]
    status_times = [r['sim_time_s'] for r in statuses]
    ages, mismatches, missing, boundary_bad = [], [], [], []
    boundary_refs, boundary_sources, epochs = [], [], set()
    compared = ('state', 'reason', 'mode', 'behavior', 'speed_command_mps', 'source_first_s', 'reference_stamp_s')
    for row in rows:
        status = row['autonomy']
        emitted = status.get('sim_time_s')
        index = bisect_right(status_times, emitted)-1 if finite(emitted) else -1
        if index < 0 or abs(status_times[index]-emitted) > 1e-9:
            missing.append(row['t'])
            continue
        raw = statuses[index]
        if any(status.get(k) != raw.get(k) for k in compared):
            mismatches.append(row['t'])
        ages.append(row['t']-emitted)
    active = [r for r in statuses if first <= r['sim_time_s'] <= stop and r.get('mode') == 'g1_follow']
    for row in active:
        ref, source, path_source = row.get('reference_stamp_s'), row.get('source_first_s'), row.get('path_source_s')
        if finite(ref) and finite(source):
            boundary_refs.append(ref)
            boundary_sources.append(source)
            epochs.add(row.get('source_epoch'))
            if not 0 <= ref-source < .30:
                boundary_bad.append({'sim_time_s': row['sim_time_s'], 'reason': 'source_age', 'ref': ref, 'source': source})
        if row.get('speed_command_mps', 0) > 0 and (not finite(ref) or not finite(source)
                or not finite(path_source) or not 0 <= ref-path_source < .30):
            boundary_bad.append({'sim_time_s': row['sim_time_s'], 'reason': 'positive_command_without_fresh_boundary'})
    # 상태는 정상일 때 0.2초마다 발행한다. 0.30초는 기존 관측 만료 상한과
    # 비교한 보조 상태 수신 조건이며 제어기의 센서 TTL을 늘리는 값이 아니다.
    checks.update(trace_raw_status_matches=bool(rows) and not missing and not mismatches,
        status_age_within_existing_observation_ttl=bool(ages) and min(ages) >= -.002 and max(ages) < .30,
        boundary_times_valid=not boundary_bad,
        boundary_reference_progresses=bool(boundary_refs) and all(b >= a for a, b in zip(boundary_refs, boundary_refs[1:])),
        boundary_source_progresses=bool(boundary_sources) and all(b >= a for a, b in zip(boundary_sources, boundary_sources[1:])))
    segments = kinematic_speed_segments(ego)
    times = [s['t1'] for s in segments]
    contradictions, held_segments, maximum_held_speed = [], 0, 0.
    for previous, row in zip(rows, rows[1:]):
        a, b = previous['autonomy'], row['autonomy']
        if not all(r['ground_speed_mps'] < .01 and r['safe_speed_mps'] == 0 and
                   r['autonomy'].get('behavior') == 'HOLD' for r in (previous, row)):
            continue
        # 상태 전환 직후 구간은 아직 모델이 제동 중일 수 있어, 상태가 같은
        # 두 표본 사이의 자세 증거만 비교한다.
        if a.get('reason') != b.get('reason'):
            continue
        start = bisect_right(times, previous['t'])
        end = bisect_right(times, row['t'])
        window = [s for s in segments[start:end] if s['t0'] >= previous['t']]
        held_segments += len(window)
        maximum = max((s['average_speed_mps'] for s in window), default=0.)
        maximum_held_speed = max(maximum_held_speed, maximum)
        if maximum > .01+1e-8:
            contradictions.append({'t0': previous['t'], 't1': row['t'], 'max_pose_speed_mps': maximum})
    # STOP 중 최초 감속은 허용하고 마지막 실제 0.5초 안정 구간을 검사한다.
    stop_end = max((r['time'] for r in stop_trace), default=None)
    stop_segments = [s for s in segments if finite(stop_end) and s['t0'] >= stop_end-.5 and s['t1'] <= stop_end]
    stop_max = max((s['average_speed_mps'] for s in stop_segments), default=None)
    checks.update(hold_matches_dense_pose=held_segments > 0 and not contradictions,
                  stop_matches_dense_pose=stop_max is not None and stop_max < .01)
    return {'audit_version': AUDIT_VERSION, 'checks': checks, 'passed': all(checks.values()),
        'trace_rows_in_case': len(rows), 'raw_status_samples': len(statuses),
        'raw_status_max_gap_s': max((b-a for a, b in zip(status_times, status_times[1:])), default=None),
        'trace_status_max_age_s': max(ages, default=None), 'missing_raw_status_times': missing[:20],
        'trace_status_mismatch_times': mismatches[:20], 'boundary_time_errors': boundary_bad[:20],
        'source_epochs': sorted(epochs), 'held_pose_segments': held_segments,
        'max_held_pose_speed_mps': maximum_held_speed, 'hold_pose_contradictions': contradictions[:20],
        'stop_last_half_second_pose_segments': len(stop_segments), 'stop_max_pose_speed_mps': stop_max,
        'limits': ['autonomy publishes every 0.2 s or state transition, not at 50 Hz',
                   'individual drivetrain diagnostic and safe_drive source timestamps were not recorded; their freshness is unverified',
                   'dense pose finite differences are planar sampled motion, not instantaneous velocity or real vehicle safety',
                   'post-audit does not alter the preserved run verdict or relax registered behavior thresholds']}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError('post-audit output already exists')
    names = ['report.json', 'trace.json', 'autonomy_status.jsonl', 'ego_poses.jsonl', 'stop_trace.json']
    raw = {name: (args.run/name).read_bytes() for name in names}
    result = audit_rows(json.loads(raw['report.json']), json.loads(raw['trace.json']),
        [json.loads(line) for line in raw['autonomy_status.jsonl'].splitlines()],
        [json.loads(line) for line in raw['ego_poses.jsonl'].splitlines()], json.loads(raw['stop_trace.json']))
    result['source_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    result['input_sha256'] = {name: hashlib.sha256(data).hexdigest() for name, data in raw.items()}
    result['run'] = str(args.run)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
