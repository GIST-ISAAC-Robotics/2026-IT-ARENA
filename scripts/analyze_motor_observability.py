"""보존된 구동 진단의 관측 한계 감사. 평가용 정답은 이 오프라인 도구에서만 사용한다.

20 ms 진단의 구간별 일정 속도 적분은 거리 불일치의 근사일 뿐, 당시 센서
콜백/스캔/경계 기억의 재생이나 실제 LiDAR 점 오차의 측정이 아니다.
"""
import argparse
from bisect import bisect_left, bisect_right
import hashlib
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LOW_MPS = .03  # 기존 계약 관측 기준. 실물 정지 허용치로 새 채택하지 않는다.
MOVING_MPS = .1  # 감사 사례 추출 기준, 제어/보호 판정선이 아니다.
MAX_GAP_S = .030
RUNS = {
    'g1_left': 'artifacts/validation/2026-10-07/g1b_side_v1/side_left_s0',
    'dantzig': 'artifacts/validation/2026-10-12/solver_compare_v1/d1_dantzig',
    'pgs': 'artifacts/validation/2026-10-12/solver_compare_v1/p1_pgs',
}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read_rows(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def checked_dynamics(rows):
    if not rows:
        raise ValueError('missing dynamics')
    keys = ('sim_time_s', 'wheel_surface_speed_mps', 'truth_longitudinal_mps',
            'truth_lateral_mps', 'motor_speed_rad_s', 'left_speed_rad_s', 'right_speed_rad_s')
    previous = -math.inf
    for row in rows:
        if not all(type(row.get(k)) in (float, int) and math.isfinite(row[k]) for k in keys):
            raise ValueError('missing/nonfinite diagnostic field')
        if row['sim_time_s'] <= previous:
            raise ValueError('unordered/duplicate dynamics')
        previous = row['sim_time_s']
    return rows


def nearest(rows, times, when, tolerance=.0105):
    """표본 간격의 반 이내만 맞춘다. 정확한 동시각/콜백 인과 관계를 주장하지 않는다."""
    index = bisect_left(times, when)
    candidates = [i for i in (index-1, index) if 0 <= i < len(rows)]
    if not candidates:
        return None
    best = min(candidates, key=lambda i: abs(times[i]-when))
    if abs(times[best]-when) > tolerance + 1e-12:
        return None
    return rows[best]


def discrepancy(rows, times, start, end, max_gap=MAX_GAP_S):
    """signed 종방향 속도 차이의 ZOH 적분. 구간 누락/외삽은 None이며 0이 아니다."""
    if not all(math.isfinite(v) for v in (start, end)) or end <= start:
        raise ValueError('invalid interval')
    if not times or start < times[0] or end > times[-1]:
        return None
    index = bisect_right(times, start)-1
    value, t = 0., start
    while t < end-1e-12:
        if index+1 >= len(times) or times[index+1]-times[index] > max_gap+1e-12:
            return None
        stop = min(end, times[index+1])
        row = rows[index]
        value += (row['truth_longitudinal_mps']-row['wheel_surface_speed_mps'])*(stop-t)
        t, index = stop, index+1
    return value


def sample(row):
    if row is None:
        return None
    keys = ('sim_time_s', 'wheel_surface_speed_mps', 'truth_longitudinal_mps',
            'truth_lateral_mps', 'motor_speed_rad_s', 'left_speed_rad_s', 'right_speed_rad_s')
    return {k: row[k] for k in keys}


def windows(rows, times, duration, begin, end):
    valid, missing, peak = 0, 0, None
    for finish in times:
        start = finish-duration
        if start < begin-1e-12 or finish > end:
            continue
        value = discrepancy(rows, times, start, finish)
        if value is None:
            missing += 1
            continue
        valid += 1
        if peak is None or abs(value) > abs(peak['signed_distance_discrepancy_m']):
            peak = {'start_s': start, 'end_s': finish, 'signed_distance_discrepancy_m': value}
    return {'duration_s': duration, 'valid_windows': valid, 'missing_windows': missing, 'peak': peak}


def stationary_audit(rows, times, statuses):
    ready, matched, missing, moving = 0, 0, 0, []
    for status in statuses:
        if not status.get('stationary_ready'):
            continue
        ready += 1
        when = status['now_us']/1e6
        row = nearest(rows, times, when)
        if row is None:
            missing += 1
            continue
        matched += 1
        if math.hypot(row['truth_longitudinal_mps'], row['truth_lateral_mps']) > MOVING_MPS:
            moving.append({'status_time_s': when, 'match_delta_s': row['sim_time_s']-when,
                           'state': status['state'], 'stationary_scope': status.get('stationary_scope'),
                           'feedback_valid': status.get('feedback_valid'),
                           'mean_wheel_mps': status.get('mean_wheel_mps'), 'diagnostic': sample(row)})
    return {'ready_records': ready, 'matched_records': matched, 'unmatched_records': missing,
            'body_moving_records': len(moving), 'first_body_moving': moving[0] if moving else None,
            'last_body_moving': moving[-1] if moving else None,
            'match_tolerance_s': .0105,
            'count_scope': '수신 기록 수이며 독립된 실패 횟수가 아니다.'}


def g1_events(statuses, rows, times):
    reached = next((s for s in statuses if s.get('speed_command_mps', 0) >= .99), None)
    holds = [s for s in statuses if reached and s['sim_time_s'] > reached['sim_time_s']
             and s.get('reason') == 'follow_boundary_unconfirmed']
    first = holds[0] if holds else None
    release = next((s for s in statuses if first and s['sim_time_s'] > first['sim_time_s']
                    and s.get('reason') == 'local_follow' and s.get('speed_command_mps', 0) > 0), None)
    result = {'selection': '1 m/s 명령 도달 뒤 최초 boundary HOLD와 그 뒤 최초 양수 명령',
              'reached_command_s': reached['sim_time_s'] if reached else None}
    for name, status in (('first_boundary_hold', first), ('first_release_after_boundary_hold', release)):
        if status is None:
            result[name] = None
            continue
        when = status['sim_time_s']
        matched = nearest(rows, times, when)
        result[name] = {
            'status': {k: status.get(k) for k in ('sim_time_s', 'reason', 'behavior',
                'speed_command_mps', 'measured_speed_mps', 'source_first_s', 'boundary_age_s')},
            'diagnostic': sample(matched),
            'match_delta_s': matched['sim_time_s']-when if matched else None,
            'preceding_windows': {str(d): discrepancy(rows, times, when-d, when) for d in (.1, .3)},
        }
    return result


def analyze_run(directory):
    report_path = directory/'report.json'
    diagnostic_path = directory/'braking_diagnostics.jsonl'
    report = json.loads(report_path.read_text(encoding='utf-8'))
    rows = checked_dynamics(read_rows(diagnostic_path))
    times = [r['sim_time_s'] for r in rows]
    begin = report.get('command_start_sim_s', report.get('green_observed_sim_s'))
    end = report['stop_request_sim_s']
    if begin is None or not times[0] <= begin < end <= times[-1]:
        raise ValueError('missing/outside operation interval')
    inputs = [report_path, diagnostic_path]
    if (directory/'actuation_trace.json').is_file():
        inputs.append(directory/'actuation_trace.json')
        statuses = json.loads(inputs[-1].read_text(encoding='utf-8'))
    else:
        # probe는 진단 수신 당시의 마지막 MCU 상태를 저장한다. 같은 상태 시각은 중복 제거.
        unique = {r['mcu']['now_us']: r['mcu'] for r in rows if r.get('mcu')}
        statuses = [unique[t] for t in sorted(unique)]
    active = [r for r in rows if begin <= r['sim_time_s'] <= end]
    low_but_moving = [r for r in active if abs(r['wheel_surface_speed_mps']) <= LOW_MPS
                      and math.hypot(r['truth_longitudinal_mps'], r['truth_lateral_mps']) > MOVING_MPS]
    result = {
        'operation_interval_s': [begin, end],
        'preserved_outcome': {k: report.get(k) for k in
            ('run_valid', 'input_valid', 'passed', 'stopped', 'stop_acknowledged', 'shutdown_clean')},
        'dynamic_samples': len(rows),
        'single_motor_relation': all(abs(r['motor_speed_rad_s']-15*(r['left_speed_rad_s']+
            r['right_speed_rad_s'])/2) < 1e-7 for r in rows),
        'low_mean_body_moving_samples': len(low_but_moving),
        'first_low_mean_body_moving': sample(low_but_moving[0]) if low_but_moving else None,
        'last_low_mean_body_moving': sample(low_but_moving[-1]) if low_but_moving else None,
        'operation_discrepancy_windows': [windows(rows, times, d, begin, end) for d in (.1, .3)],
        'stationary_ready': stationary_audit(rows, times, statuses),
        'final_sample': sample(rows[-1]),
    }
    if (directory/'autonomy_status.jsonl').is_file():
        inputs.append(directory/'autonomy_status.jsonl')
        result['g1_events'] = g1_events([r for r in read_rows(inputs[-1])
                                       if begin <= r['sim_time_s'] <= end], rows, times)
    result['input_sha256'] = {p.name: sha(p) for p in inputs}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = {
        'scope': '오프라인 관측 한계 감사. 제어 입력/판정 변경 및 새 주행 없음.',
        'thresholds': {'low_mean_mps': LOW_MPS, 'body_moving_mps': MOVING_MPS,
                       'diagnostic_gap_limit_s': MAX_GAP_S},
        'runs': {name: analyze_run(ROOT/path) for name, path in RUNS.items()},
        'analysis_sha256': sha(Path(__file__)),
        'current_source_sha256': {p: sha(ROOT/p) for p in (
            'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
            'src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py',
            'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py',
            'src/arena_autonomy/arena_autonomy/lidar_motion.py',
            'src/arena_autonomy/arena_autonomy/lidar_motion_ros.py',
            'src/arena_autonomy/arena_autonomy/local_follow.py',
            'src/arena_autonomy/arena_autonomy/local_pursuit.py',
            'src/arena_autonomy/arena_autonomy/lidar_safety.py',
            'src/arena_autonomy/arena_autonomy/scene_motion.py',
            'tests/test_motor_observability.py')},
        'limits': [
            'truth_* 및 좌우 속도는 평가 전용이며 제어기에 전달하지 않는다.',
            '20 ms 진단 ZOH의 종방향 거리 차이. 센서/콜백 재생이나 실제 보정점 위치 오차가 아니다.',
            '0.1/0.3초는 스캔/경계 기억 시간척도 대조. 해당 경계가 실제로 전체 구간 유지됐다는 뜻이 아니다.',
            'MCU/상태와 물리 표본은 최대 10.5 ms 근접 대응. 누락은 null로 두고 정지로 메우지 않는다.',
            'PGS는 평가 전용 실패이며 실물 발생 확률/주행 합격 근거가 아니다.',
        ],
    }
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({name: {'low_mean_moving': r['low_mean_body_moving_samples'],
                            'ready_moving': r['stationary_ready']['body_moving_records'],
                            'windows': r['operation_discrepancy_windows']}
                      for name, r in result['runs'].items()}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
