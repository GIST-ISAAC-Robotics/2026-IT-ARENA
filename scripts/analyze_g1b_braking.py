#!/usr/bin/env python3
"""G1-b 보존 기록의 제동 명령/엔코더/차체 이동을 비교하는 평가 전용 도구.

정답 위치는 원인 분석에만 사용합니다. 실차 속도 추정이나 제어 입력이 아닙니다.
서로 다른 시각의 자료는 근접 표본 오차를 함께 보존하고 원인을 자동 단정하지 않습니다.
"""
import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def nearest_index(times, target):
    index = bisect_left(times, target)
    return min({max(0, index - 1), min(len(times) - 1, index)}, key=lambda i: abs(times[i] - target))


def pose_speed(poses, times, target, half_window=.025):
    """약 50ms 위치 차분. 양 끝이 없거나 시각이 먼 구간은 미관측으로 반환합니다."""
    if not poses or target - half_window < times[0] or target + half_window > times[-1]:
        return None
    a = nearest_index(times, target - half_window)
    b = nearest_index(times, target + half_window)
    dt = times[b] - times[a]
    if dt <= 0 or abs(times[a] - (target - half_window)) > .005 or abs(times[b] - (target + half_window)) > .005:
        return None
    return math.hypot(poses[b][1] - poses[a][1], poses[b][2] - poses[a][2]) / dt


def diagnostic_summary(records, onset, radius):
    """첫 감속 이후 0.5초의 동일 원천 시각 바퀴/차체 차이. 원인 판정은 하지 않습니다."""
    required = ('sim_time_s', 'left_speed_rad_s', 'right_speed_rad_s',
                'wheel_surface_speed_mps', 'truth_longitudinal_mps', 'truth_roll_rad', 'truth_pitch_rad')
    if not math.isfinite(radius) or radius <= 0:
        raise ValueError('invalid wheel radius')
    for row in records:
        if any(type(row.get(key)) not in (int, float) or not math.isfinite(row[key]) for key in required):
            raise ValueError('missing or nonfinite diagnostic source')
    times = [row['sim_time_s'] for row in records]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('non-monotonic diagnostic source timestamps')
    selected = [row for row in records if onset is not None and onset <= row['sim_time_s'] <= onset + .5]
    if not selected:
        return {'samples': 0, 'available': False}
    peak = max(selected, key=lambda row: abs(row['left_speed_rad_s'] - row['right_speed_rad_s']))
    return {
        'available': True, 'samples': len(selected), 'window_start_sim_s': onset, 'window_duration_s': .5,
        'max_source_gap_s': max((b-a for a, b in zip(times, times[1:])), default=None),
        'max_wheel_body_difference_mps': max(abs(row['wheel_surface_speed_mps']-row['truth_longitudinal_mps']) for row in selected),
        'max_left_right_difference_mps': radius * abs(peak['left_speed_rad_s']-peak['right_speed_rad_s']),
        'max_abs_roll_rad': max(abs(row['truth_roll_rad']) for row in selected),
        'max_abs_pitch_rad': max(abs(row['truth_pitch_rad']) for row in selected),
        'left_right_peak': peak,
        'scope': 'simulated wheel/body discrepancy only; no contact-load measurement or real drivetrain fault verdict',
    }


def analyze(run):
    paths = [run / name for name in ('report.json', 'trace.json', 'actuation_trace.json', 'ego_poses.jsonl')]
    report, trace, actuation = [json.loads(path.read_text(encoding='utf-8')) for path in paths[:3]]
    poses = [json.loads(line) for line in paths[3].read_text(encoding='utf-8').splitlines()]
    times = [row[0] for row in poses]
    mcu_times = [row['now_us'] / 1e6 for row in actuation]
    if not times or not mcu_times or any(b <= a for a, b in zip(times, times[1:])) or any(b < a for a, b in zip(mcu_times, mcu_times[1:])):
        raise ValueError('empty or non-monotonic source timestamps')
    if hashlib.sha256(paths[3].read_bytes()).hexdigest() != report['ego_poses_sha256']:
        raise ValueError('ego source differs from the saved report')
    active = [row for row in trace if row.get('elapsed_since_green_s') is not None]
    braking = [row for row in active if row['safety_reason'] == 'lidar_braking' and row['ground_speed_mps'] > .3]
    onset = braking[0]['t'] if braking else None
    rows = []
    for row in active:
        if onset is None or not onset - .2 <= row['t'] <= onset + 1.5:
            continue
        i = nearest_index(mcu_times, row['t'])
        mcu = actuation[i]
        close = abs(mcu_times[i] - row['t']) <= .025
        speed = pose_speed(poses, times, row['t'])
        wheel = mcu.get('mean_wheel_mps') if close and mcu.get('feedback_valid') else None
        rows.append({
            'sim_s': row['t'], 'body_gap_m': row['sampled_gap_m'],
            'pose_difference_speed_mps': speed, 'cached_truth_speed_mps': row['ground_speed_mps'],
            'safe_command_mps': row['safe_speed_mps'], 'safety_reason': row['safety_reason'],
            'safety_clearance_m': row['safety_clearance_m'],
            'mcu_time_offset_s': mcu_times[i] - row['t'],
            'mcu_target_mps': mcu['target_speed_mps'] if close else None,
            'wheel_feedback_mps': wheel,
            'pose_minus_wheel_mps': speed - wheel if speed is not None and wheel is not None else None,
            'braking_diagnostics': row.get('braking_diagnostics'),
        })
    result = {
        'run': str(run), 'run_valid': report.get('run_valid'),
        'input_sha256': {path.name: hashlib.sha256(path.read_bytes()).hexdigest() for path in paths},
        'gap': report['evaluation']['gap'], 'first_braking_trace_sim_s': onset,
        'scope': 'offline evaluation only; nearest MCU within 25ms, body XY difference over about 50ms; missing diagnostics are not zero; no contact/friction/root-cause conclusion from wheel-body discrepancy alone',
        'rows': rows,
    }
    path = run / 'braking_diagnostics.jsonl'
    if path.exists():
        if hashlib.sha256(path.read_bytes()).hexdigest() != report['braking_diagnostics']['sha256']:
            raise ValueError('diagnostic source differs from the saved report')
        vehicle_path = run / 'runtime_vehicle.sdf'
        vehicle = ET.parse(vehicle_path)
        radius = float(vehicle.findtext(".//plugin[@name='arena::SingleMotorDrive']/wheel_radius"))
        records = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
        result['diagnostics'] = diagnostic_summary(records, onset, radius)
        result['input_sha256'][path.name] = report['braking_diagnostics']['sha256']
        result['input_sha256'][vehicle_path.name] = hashlib.sha256(vehicle_path.read_bytes()).hexdigest()
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = [analyze(run) for run in args.runs]
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2, allow_nan=False)
    for result in data:
        print(result['run'], 'contacts=', result['gap']['actor_contact_messages_with_ego'],
              'braking_onset=', result['first_braking_trace_sim_s'])


if __name__ == '__main__':
    main()
