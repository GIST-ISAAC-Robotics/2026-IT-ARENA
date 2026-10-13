"""유한한 합성 행렬과 보존 G1 스캔에 기록 전용 진단을 적용한다. 새 시뮬레이션 없음."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from scan_motion_diagnostic import Settings, compare_scans, prepare

ROOT = Path(__file__).resolve().parents[1]
G1 = ROOT/'artifacts/validation/2026-10-07/g1b_side_v1/side_left_s0'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cross(a, b):
    return a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]


def synthetic_scan(stamp, *, v=0., w=0., room=True, small_front=False, moving_front=False,
                   noise=0., seed=0, dropout=False):
    """500개 광선을 실제 각 취득 시점의 위치에서 쏜다. 반환 없음은 Inf 그대로다."""
    n, step = 500, .0002
    angle = -math.pi+np.arange(n)*math.tau/n
    t = stamp+np.arange(n)*step-.2
    yaw = w*t
    x = v*t if abs(w) < 1e-10 else v/w*np.sin(yaw)
    y = np.zeros(n) if abs(w) < 1e-10 else v/w*(1-np.cos(yaw))
    origin = np.column_stack((x+.1275*np.cos(yaw), y+.1275*np.sin(yaw)))
    directions = np.column_stack((np.cos(angle+yaw), np.sin(angle+yaw)))
    walls = [((-6., -.425), (6., -.425)), ((6., .425), (-6., .425))]
    if room:
        walls.append(((1.5, -.425), (1.5, .425)))
    if small_front:
        # 움직이는 앞차는 독립 동적 선분. 정답 ID는 진단기에 주지 않는다.
        front_x = 1.2-(stamp-.2) if moving_front else 1.2
        walls.append(((front_x, -.075), (front_x, .075)))
    distances = []
    for start, end in walls:
        a, vector = np.asarray(start), np.asarray(end)-start
        denom = cross(directions, vector)
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = cross(a-origin, vector)/denom
            along = cross(a-origin, directions)/denom
        valid = (distance >= .05) & (distance <= 12) & (along >= 0) & (along <= 1) & (np.abs(denom) > 1e-9)
        distances.append(np.where(valid, distance, math.inf))
    ranges = np.min(distances, axis=0)
    rng = np.random.default_rng(seed)
    finite = np.isfinite(ranges)
    ranges[finite] += rng.uniform(-noise, noise, np.count_nonzero(finite))
    if dropout:
        ranges[200:320] = math.inf
    return {'stamp_s': stamp, 'time_increment': step, 'angle_min': -math.pi,
            'angle_increment': math.tau/n, 'ranges': ranges.tolist()}


CASES = {
    'stationary_room': ({}, 0., 'compatible_under_assumptions'),
    'honest_straight': ({'v': 1.4}, .42, 'compatible_under_assumptions'),
    'zero_encoder_sliding': ({'v': 1.4}, 0., 'inconsistent_candidate'),
    'parallel_corridor': ({'v': 1.4, 'room': False}, 0., 'unknown'),
    'stationary_parallel_corridor': ({'room': False}, 0., 'unknown'),
    'small_moving_front': ({'room': False, 'small_front': True, 'moving_front': True}, 0., 'unknown'),
    'small_static_front': ({'room': False, 'small_front': True}, 0., 'unknown'),
    'turn_without_gyro': ({'v': .3, 'w': .5}, .09, 'unknown'),
    'front_dropout': ({'dropout': True}, 0., 'unknown'),
}


def count_series(statuses):
    """취득 시각과 원시 모터 카운트만 회수. 같은 표본의 중복/상충을 구별한다."""
    unique = {}
    for s in statuses:
        t, c = s.get('feedback_capture_us'), s.get('motor_counts')
        if t is None or c is None or not s.get('feedback_valid'):
            continue
        if s.get('feedback_mode') != 'drive_motor_shaft' or type(t) is not int or type(c) is not int:
            raise ValueError('invalid motor count record')
        if t in unique and unique[t] != c:
            raise ValueError('conflicting counts at same capture time')
        unique[t] = c
    return np.array(sorted(unique))/1e6, np.array([unique[t] for t in sorted(unique)], dtype=float)


def encoder_travel(times, counts, start, finish):
    if len(times) < 2 or start < times[0] or finish > times[-1] or finish <= start:
        return None
    lo = max(0, np.searchsorted(times, start, side='right')-1)
    hi = min(len(times), np.searchsorted(times, finish, side='left')+1)
    if np.max(np.diff(times[lo:hi]), initial=0.) > .025:
        return None
    # 카운트 표본 사이 선형 보간. 로봇이 실제로 그 속도로 움직였다는 가정은 하지 않는다.
    delta = np.interp(finish, times, counts)-np.interp(start, times, counts)
    return float(delta*math.tau*.0325/(28*15))


def run_synthetic(cfg):
    results = []
    for name, (kwargs, encoder, expected) in CASES.items():
        a, b = synthetic_scan(.2, **kwargs), synthetic_scan(.5, **kwargs)
        result = compare_scans(a, b, encoder, cfg)
        results.append({'name': name, 'sensor_model': 'sequential_raycast', 'expected': expected,
                        'expectation_met': result['classification'] == expected, 'result': result})
    # 잡음은 제조사 표준편차가 아닌 ±30 mm 균등 주입 스트레스다.
    for room in (False, True):
        for seed in range(3):
            a = synthetic_scan(.2, room=room, noise=.03, seed=seed)
            b = synthetic_scan(.5, room=room, noise=.03, seed=seed+10)
            result = compare_scans(a, b, 0., cfg)
            passed = result['classification'] != 'inconsistent_candidate'
            if not room:
                passed = result['classification'] == 'unknown'
            results.append({'name': f'noise_stationary_{"room" if room else "corridor"}_{seed}',
                            'expected': 'not_inconsistent' if room else 'unknown',
                            'expectation_met': passed, 'result': result})
    return results


def run_g1(cfg):
    scans = [json.loads(line) for line in (G1/'scans_received.jsonl').read_text().splitlines()]
    statuses = json.loads((G1/'actuation_trace.json').read_text())
    times, counts = count_series(statuses)
    result = []
    for lag in (1, 3):
        for i in range(lag, len(scans)):
            a, b = scans[i-lag], scans[i]
            start = a['stamp_s']+(len(a['ranges'])-1)*a['time_increment']/2
            finish = b['stamp_s']+(len(b['ranges'])-1)*b['time_increment']/2
            if not 3.6 <= start < finish <= 10.22:
                continue
            travel = encoder_travel(times, counts, start, finish)
            diagnosis = compare_scans(a, b, travel, cfg)
            result.append({'lag': lag, 'start_s': start, 'end_s': finish, 'encoder_travel_m': travel,
                           'result': diagnosis})
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    cfg = Settings()
    frozen = {str(p.relative_to(ROOT)): sha(p) for p in [Path(__file__), ROOT/'scripts/scan_motion_diagnostic.py',
                ROOT/'tests/test_scan_motion_diagnostic.py', G1/'scans_received.jsonl', G1/'actuation_trace.json']}
    synthetic = run_synthetic(cfg)
    # 합성 관문 실패는 숨기지 않으며 실제 기록 적용 전에 중단한다.
    gate = all(r['expectation_met'] for r in synthetic)
    g1 = run_g1(cfg) if gate else []
    result = {'scope': '기록 전용. 새 Gazebo 주행/제어 연결 없음. 정적 구조 가정과 sweep 상한은 미보증.',
              'settings': asdict(cfg), 'synthetic': synthetic, 'synthetic_expectations_met': gate,
              'g1': g1, 'g1_executed': gate,
              'g1_classifications': dict(Counter(r['result']['classification'] for r in g1)),
              'g1_reasons': dict(Counter(r['result']['reason'] for r in g1)),
              'input_source_sha256': frozen,
              'inputs_unchanged': all(sha(ROOT/p) == value for p, value in frozen.items()),
              'runtime_not_connected': True}
    with (args.output/'report.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({'synthetic': [(r['name'], r['expectation_met'], r['result']['classification'],
                                  r['result']['reason']) for r in synthetic],
                      'g1_classifications': result['g1_classifications'], 'g1_reasons': result['g1_reasons']},
                     ensure_ascii=False, indent=2))
    raise SystemExit(0 if gate and result['inputs_unchanged'] else 1)


if __name__ == '__main__':
    main()
