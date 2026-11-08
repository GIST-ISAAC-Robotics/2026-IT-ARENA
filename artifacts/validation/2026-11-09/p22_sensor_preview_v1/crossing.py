"""평가 전용 사후 처리: 첫 입구 간극 N 검출 뒤 자차 중심이 실제로 E1 횡단선을 지난 시각.

관측기·판정 불변, 센서 재실행 없음. evaluation.json(첫 검출 시각)과 ego_poses.jsonl만 읽는다.
기준점: ego_poses의 모델 원점 = base_link = chassis 중심(앞뒤 바퀴축 ±0.0675 m의 가운데).
E1 횡단선: 월드 y=4.80(P22 입구 상류 끝 E1 (5.635, 4.80)을 지나는 본선 남행 횡단선),
입구 본선 근처 x∈[5.80, 6.50]에서 y가 4.80 초과→이하로 바뀌는 첫 인접 표본 쌍을 선형 보간한다.
"""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
RUNS = {
    'r11': 'artifacts/validation/2026-11-07/junction_handoff_v1/run1',
    'r27': 'artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1/two_actor_course_lap/run1',
}
LINE_Y = 4.80
X_RANGE = (5.80, 6.50)


def main():
    ev = json.loads((HERE / 'evaluation.json').read_bytes())
    out = dict(scope='evaluator-only post-processing; observer and verdict unchanged; no sensor rerun',
               reference_point='ego_poses model origin = base_link = chassis centre (midway between axles at +/-0.0675 m)',
               crossing_line='world y=4.80 through P22 entry E1 (5.635,4.80), southbound main road, x in [5.80,6.50]',
               detection_time='scan reference time t of the first registered entry notch (decision reference_s; '
                              'rays were acquired up to ~0.1-0.2 s earlier)',
               approximation_name='E1_time_left_const_speed_approx_s = body-frame forward distance to E1 / '
                                  'instantaneous ego speed (+/-0.05 s central difference) at t; '
                                  'stored in evaluation.json as E1_time_left_s',
               inputs={}, runs={})
    for name, rel in RUNS.items():
        path = ROOT / rel / 'ego_poses.jsonl'
        out['inputs'][path.relative_to(ROOT).as_posix()] = hashlib.sha256(path.read_bytes()).hexdigest()
        first = ev['runs'][name]['entry_notch_first']
        t0 = first['t']
        with path.open(encoding='utf-8') as f:
            rows = sorted((json.loads(l) for l in f if l.strip()), key=lambda r: r[0])
        cross = None
        for a, b in zip(rows, rows[1:]):
            if b[0] < t0 - 1.0:
                continue
            if (a[2] > LINE_Y >= b[2] and X_RANGE[0] <= a[1] <= X_RANGE[1]
                    and X_RANGE[0] <= b[1] <= X_RANGE[1]):
                k = (a[2] - LINE_Y) / (a[2] - b[2])
                cross = dict(t_cross_s=a[0] + k*(b[0] - a[0]), x_at_cross_m=a[1] + k*(b[1] - a[1]),
                             bracket_samples=[a, b], sample_interval_s=b[0] - a[0],
                             yaw_at_cross_rad=a[3] + k*(b[3] - a[3]))
                break
        assert cross is not None and cross['t_cross_s'] > t0, name
        gaps = [b[0] - a[0] for a, b in zip(rows, rows[1:]) if t0 <= a[0] <= cross['t_cross_s']]
        actual = cross['t_cross_s'] - t0
        out['runs'][name] = dict(
            first_detection_t_s=t0, first_detection_E1_forward_m=first['E1_forward_m'],
            E1_time_left_const_speed_approx_s=first['E1_time_left_s'],
            speed_at_detection_mps=first['speed_mps'],
            **cross, actual_time_detection_to_E1_line_s=actual,
            forward_distance_over_actual_time_mps=first['E1_forward_m'] / actual,
            approx_minus_actual_s=first['E1_time_left_s'] - actual,
            ego_sample_interval_between_detection_and_cross_s=dict(
                count=len(gaps), min=min(gaps), max=max(gaps), mean=sum(gaps)/len(gaps)))
    (HERE / 'crossing.json').write_bytes((json.dumps(out, ensure_ascii=False, indent=1) + '\n').encode('utf-8'))
    print(json.dumps({n: {k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items() if k != 'bracket_samples'}
                      for n, r in out['runs'].items()}, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
