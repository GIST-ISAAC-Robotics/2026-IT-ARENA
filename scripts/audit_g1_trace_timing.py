#!/usr/bin/env python3
"""G1 trace에 복사한 진단의 원천 시각을 검사한다. 기존 주행/정지 판정은 변경하지 않는다."""
import argparse
from bisect import bisect_left, bisect_right
import hashlib
import json
import math
from pathlib import Path


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def stats(values):
    return dict(samples=len(values), minimum=min(values), maximum=max(values)) if values else None


def pose_neighborhood(poses, times, when):
    """주변의 실제 두 표본만 사용. 속도 보간/외삽·공백 메우기는 하지 않는다."""
    if when < times[0] or when > times[-1]:
        return dict(available=False, reason='outside_recorded_pose_range')
    i = bisect_left(times, when)
    candidates = (i-1, i) if i < len(times) and times[i] == when else (i-1,)
    segments = []
    for j in candidates:
        if 0 <= j < len(poses)-1:
            a, b = poses[j], poses[j+1]
            dt = b[0]-a[0]
            segments.append(dict(t0=a[0], t1=b[0], duration_s=dt,
                                 average_speed_mps=math.hypot(b[1]-a[1], b[2]-a[2])/dt))
    return dict(available=bool(segments), segments=segments)


def audit(trace, poses, dynamics, windows):
    for label, rows, getter in (('trace', trace, lambda r: r.get('t')),
                                ('poses', poses, lambda r: r[0]),
                                ('dynamics', dynamics, lambda r: r.get('sim_time_s'))):
        times = [getter(r) for r in rows]
        if not times or not all(finite(t) for t in times) or any(b <= a for a, b in zip(times, times[1:])):
            raise ValueError(label + ': finite, strictly ordered timestamps required')
    if len(poses) < 2 or any(len(r) != 4 or not all(finite(v) for v in r) for r in poses):
        raise ValueError('pose shape/values invalid')
    pose_times = [r[0] for r in poses]
    raw = {r['sim_time_s']: r for r in dynamics}
    ages = {name: [] for name in ('dynamics', 'safe_command', 'mcu_status')}
    missing = {name: [] for name in ages}
    errors, samples, ahead_of_pose = [], [], []
    for index, row in enumerate(trace):
        t = row['t']
        sample = dict(trace_index=index, trace_s=t)
        diagnostic = row.get('braking_diagnostics')
        if not isinstance(diagnostic, dict):
            for name in missing: missing[name].append(index)
            sample['available'] = False
            samples.append(sample)
            continue
        source = diagnostic.get('sim_time_s')
        stamps = dict(dynamics=source, safe_command=diagnostic.get('safe_command_stamp_s'),
                      mcu_status=(diagnostic['mcu_now_us']*1e-6
                                  if finite(diagnostic.get('mcu_now_us')) else None))
        for name, stamp in stamps.items():
            if not finite(stamp):
                missing[name].append(index)
            else:
                age = t-stamp
                ages[name].append(age)
                # trace t는 수신 시계가 아니라 비동기 자세 표본의 원천 시각이다.
                # 다른 채널이 그 자세보다 앞선 것은 복사 오류/미래 취득 증명이 아니다.
                if age < -1e-8:
                    ahead_of_pose.append(dict(index=index, stream=name, source_s=stamp,
                                              pose_s=t, offset_s=age))
        sample.update(available=finite(source), source_s=source,
                      dynamics_age_s=t-source if finite(source) else None,
                      safe_command_stamp_s=stamps['safe_command'], mcu_status_stamp_s=stamps['mcu_status'])
        if finite(source):
            original = raw.get(source)
            if original is None:
                errors.append(dict(index=index, reason='raw_diagnostic_source_missing'))
            elif any(diagnostic.get(key) != value for key, value in original.items()):
                errors.append(dict(index=index, reason='copied_diagnostic_mismatch'))
            vx, vy, stored = (diagnostic.get('truth_longitudinal_mps'),
                              diagnostic.get('truth_lateral_mps'), row.get('ground_speed_mps'))
            if not all(finite(v) for v in (vx, vy, stored)):
                errors.append(dict(index=index, reason='nonfinite_speed'))
            else:
                source_speed = math.hypot(vx, vy)
                if not math.isclose(source_speed, stored, abs_tol=1e-10, rel_tol=1e-9):
                    errors.append(dict(index=index, reason='copied_speed_mismatch'))
                sample.update(stored_source_speed_mps=stored,
                              pose_near_source=pose_neighborhood(poses, pose_times, source),
                              pose_near_trace=pose_neighborhood(poses, pose_times, t))
        samples.append(sample)
    selections = []
    for start, end in windows:
        if not (finite(start) and finite(end) and end > start):
            raise ValueError('invalid window')
        selected = [r for r in samples if start-1e-9 <= r['trace_s'] <= end+1e-9]
        lo, hi = bisect_left(pose_times, start), bisect_right(pose_times, end)
        actual = poses[lo:hi]
        segments = [dict(t0=a[0], t1=b[0], speed_mps=math.hypot(b[1]-a[1], b[2]-a[2])/(b[0]-a[0]))
                    for a, b in zip(actual, actual[1:])]
        selections.append(dict(t0=start, t1=end, trace_samples=selected,
                               pose_sample_count=len(actual), covered_first_s=actual[0][0] if actual else None,
                               covered_last_s=actual[-1][0] if actual else None,
                               maximum_pose_segment_speed_mps=max((s['speed_mps'] for s in segments), default=None),
                               time_over_existing_0p01_mps_s=sum(s['t1']-s['t0'] for s in segments if s['speed_mps']>.01)))
    return dict(schema='g1_trace_timing_v2', raw_source_consistent=not errors,
                trace_samples=len(trace), available_diagnostic_samples=sum(r['available'] for r in samples),
                source_offset_from_pose_s={k: stats(v) for k, v in ages.items()},
                source_after_pose_reference=ahead_of_pose,
                missing_source_indices=missing, source_errors=errors, windows=selections,
                freshness_certified=False, continuous_stop_certified=False, original_verdict_changed=False,
                limits=['trace time is pose source time; channel offsets are not callback latency or causal clock errors',
                        'pose secants are sampled average speed, not instantaneous truth speed',
                        'missing source timestamps remain unknown; no extrapolation or zero fill',
                        'all truth inputs are evaluation-only; no controller or protection changes'])


def digest(path):
    with Path(path).open('rb') as stream:
        result = hashlib.sha256()
        for chunk in iter(lambda: stream.read(1024*1024), b''): result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--window', nargs=2, action='append', type=float, default=[])
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists(): parser.error('output must be new')
    names = ('trace.json', 'ego_poses.jsonl', 'braking_diagnostics.jsonl', 'report.json')
    paths = {name: args.run/name for name in names}
    if args.output.resolve() in [p.resolve() for p in paths.values()]: parser.error('cannot replace input')
    hashes = {name: digest(p) for name, p in paths.items()}
    trace = json.loads(paths['trace.json'].read_text())
    poses = [json.loads(line) for line in paths['ego_poses.jsonl'].read_text().splitlines()]
    dynamics = [json.loads(line) for line in paths['braking_diagnostics.jsonl'].read_text().splitlines()]
    result = audit(trace, poses, dynamics, args.window)
    result.update(input_sha256=hashes, inputs_unchanged=all(digest(paths[n]) == v for n, v in hashes.items()),
                  source_sha256=digest(__file__), run=str(args.run),
                  original_run_passed=json.loads(paths['report.json'].read_text()).get('passed'))
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('input_sha256', 'windows', 'missing_source_indices')}, ensure_ascii=False))
    return 0 if result['raw_source_consistent'] and result['inputs_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
