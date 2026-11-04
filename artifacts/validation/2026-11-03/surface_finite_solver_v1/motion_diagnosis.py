"""저장 입력의 운동 보정 오차 분석. 정답은 비교에만 쓰며 제어/기록에 재주입하지 않는다."""
import json
import math
from pathlib import Path

import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
CASES = {
    'failed_braking': ('2026-11-02/solo_ray_combination_v1/run1', (37.45, 38.3)),
    'selected_corner': ('2026-11-03/solo_post_free_v1/run1', (38.5, 40.3)),
}


def truth_project(raw, source, target):
    c, s = np.cos(source[:, 2]), np.sin(source[:, 2])
    wx = c*raw[:, 0]-s*raw[:, 1]+source[:, 0]-target[0]
    wy = s*raw[:, 0]+c*raw[:, 1]+source[:, 1]-target[1]
    c, s = math.cos(target[2]), math.sin(target[2])
    return np.column_stack((c*wx+s*wy, -s*wx+c*wy))


def filtered(times, values, mode):
    # 시험 가설일 뿐 실물 가속 한계/노이즈 공칭값으로 채택하지 않는다.
    out = [values[0]]
    for a, b, value in zip(times, times[1:], values[1:]):
        dt = b-a
        if mode == 'lowpass40ms':
            out.append(out[-1]+(-math.expm1(-dt/.04))*(value-out[-1]))
        elif mode == 'rate8':
            out.append(out[-1]+float(np.clip(value-out[-1], -8*dt, 8*dt)))
        else:
            out.append(value)
    return out


def evaluate(label, relative, window):
    run = ROOT/'artifacts/validation'/relative
    poses = np.array([json.loads(line) for line in (run/'ego_poses.jsonl').read_bytes().splitlines()])
    poses[:, 3] = np.unwrap(poses[:, 3])
    if poses.shape[1] != 4 or np.any(np.diff(poses[:, 0]) <= 0):
        raise ValueError('truth pose format/order')
    def pose(times):
        if np.min(times) < poses[0, 0] or np.max(times) > poses[-1, 0]:
            raise ValueError('truth extrapolation is not allowed')
        return np.column_stack([np.interp(times, poses[:, 0], poses[:, k]) for k in (1, 2, 3)])
    rows = []
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        config = MotionConfig(**json.loads(next(stream))['data']['motion_config'])
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call' or not window[0] <= event['data']['input']['reference_s'] <= window[1]:
                continue
            data, scan, history = restore_call(event['data']['input'], config)
            ref = data['reference_s']
            stamp = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            distances = np.array(scan.ranges)
            indices = np.flatnonzero(np.isfinite(distances) & (distances >= scan.range_min) & (distances <= scan.range_max))
            angles = scan.angle_min+indices*scan.angle_increment+config.lidar_yaw_rad
            raw = np.column_stack((distances[indices]*np.cos(angles)+config.lidar_x_m,
                                   distances[indices]*np.sin(angles)+config.lidar_y_m))
            acquired = stamp+indices*scan.time_increment
            truth = pose(np.r_[acquired, ref])
            reference = truth_project(raw, truth[:-1], truth[-1])
            near = distances[indices] <= 3.5
            original = list(history.wheels.values)
            comparison = {}
            for mode in ('raw', 'lowpass40ms', 'rate8'):
                history.wheels.values = filtered(history.wheels.times, original, mode)
                projected, found, _ = history.project(scan.ranges, scan.angle_min, scan.angle_increment,
                    scan.range_min, scan.range_max, stamp, scan.time_increment, ref, 'both', True)
                assert np.array_equal(found, indices)
                errors = np.linalg.norm(projected-reference, axis=1)
                comparison[mode] = dict(near_rms_m=float(np.sqrt(np.mean(errors[near]**2))),
                                       near_max_m=float(np.max(errors[near])),
                                       estimated_latest_speed_mps=history.wheels.values[-1])
            rows.append(dict(t=ref, scan_stamp_s=stamp, points=int(near.sum()), projection=comparison,
                             reason=event['data']['output'][2].get('reason')))
    trajectory = [json.loads(line) for line in (run/'trajectory.jsonl').read_bytes().splitlines()]
    dynamic = [r['dynamics'] for r in trajectory if window[0] <= r['time'] <= window[1] and r.get('dynamics')]
    unique = {r['sim_time_s']: r for r in dynamic}
    dynamic = list(unique.values())
    evidence = [dict(t=r['sim_time_s'], motor_implied_mps=r['wheel_surface_speed_mps'],
                     body_forward_mps=r['truth_longitudinal_mps'], yaw_rate=r['truth_yaw_rate_rad_s'],
                     target=r['applied_target_mps']) for r in dynamic]
    summary = {mode: dict(mean_near_rms_m=float(np.mean([r['projection'][mode]['near_rms_m'] for r in rows])),
                         maximum_near_error_m=max(r['projection'][mode]['near_max_m'] for r in rows))
               for mode in ('raw', 'lowpass40ms', 'rate8')}
    return dict(case=label, window_s=window, calls=len(rows), comparison=summary,
                maximum_abs_roll_pitch_rad=max(max(abs(r['truth_roll_rad']), abs(r['truth_pitch_rad'])) for r in dynamic),
                motor_body_examples=evidence, rows=rows)


if __name__ == '__main__':
    # 좌표 변환의 정지/단순 평행 이동을 별도 계산으로 확인한다.
    test = np.array([[1., 2.]])
    np.testing.assert_allclose(truth_project(test, np.array([[0., 0., 0.]]), np.array([0., 0., 0.])), test)
    np.testing.assert_allclose(truth_project(test, np.array([[0., 0., 0.]]), np.array([.1, .2, 0.])), [[.9, 1.8]])
    result = dict(scope='사후 운동 보정 오차 비교. 평가 정답 재투영을 제어기에 넣지 않음. 필터2종은 채택 전 가설·실물 상수 아님.',
                  limitations='동일 원시 거리/각도의 두 평면 재투영 차이로 운동 추정 영향만 비교한다. 실제 거리·노출·취득 시각 오차 전체가 아니다. 창마다 이미 수신한 운동 이력만 사용하며 필터 상태 초기화의 영향이 남을 수 있다.',
                  cases=[evaluate(name, *spec) for name, spec in CASES.items()])
    (HERE/'motion_diagnosis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps([{k: v for k, v in r.items() if k not in ('rows', 'motor_body_examples')} for r in result['cases']], ensure_ascii=False))
