#!/usr/bin/env python3
"""옆차 실패의 원시 시각 대조. 제어 입력·원래 판정·원시 파일을 수정하지 않는다."""
import hashlib
import json
from pathlib import Path

BASE = Path(__file__).resolve().parent
RUN = BASE/'side_left_s0'


def load_rows(name):
    return [json.loads(line) for line in (RUN/name).read_text().splitlines()]


def first(rows, predicate):
    return next((r for r in rows if predicate(r)), None)


def select(row, names):
    return {name: row.get(name) for name in names} if row else None


def main():
    report = json.loads((RUN/'report.json').read_text())
    statuses = load_rows('autonomy_status.jsonl')
    dynamics = load_rows('braking_diagnostics.jsonl')
    green, end = report['green_observed_sim_s'], report['stop_request_sim_s']
    statuses = [r for r in statuses if green <= r['sim_time_s'] < end]
    dynamics = [r for r in dynamics if green <= r['sim_time_s'] < end]
    reached = first(statuses, lambda r: r.get('speed_command_mps', 0.) >= .99)
    reduced = first(statuses, lambda r: r['sim_time_s'] > reached['sim_time_s'] and
                    0. < r.get('speed_command_mps', 0.) < .99)
    held = first(statuses, lambda r: r['sim_time_s'] > reached['sim_time_s'] and
                 r.get('reason') == 'follow_boundary_unconfirmed')
    mismatch = first(dynamics, lambda r: abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']) >= .35)
    reverse = first(dynamics, lambda r: r['truth_longitudinal_mps'] > .1 and
                    min(r['left_speed_rad_s'], r['right_speed_rad_s']) < -1.)
    rest = first(dynamics, lambda r: r['sim_time_s'] > held['sim_time_s'] and
                 abs(r['truth_longitudinal_mps']) < .01 and abs(r['truth_lateral_mps']) < .01)
    released = first(statuses, lambda r: r['sim_time_s'] > held['sim_time_s'] and
                     r.get('reason') == 'local_follow' and r.get('speed_command_mps', 0.) > 0.)
    motion = first(dynamics, lambda r: r['sim_time_s'] > released['sim_time_s'] and r['truth_longitudinal_mps'] > .1)
    status_keys = ('sim_time_s', 'reason', 'behavior', 'speed_command_mps', 'source_first_s',
                   'observed_path_ahead_m', 'boundary_age_s', 'boundary_candidate_reason',
                   'boundary_candidate_ahead_m', 'measured_speed_mps')
    dyn_keys = ('sim_time_s', 'reference_speed_mps', 'truth_longitudinal_mps', 'wheel_surface_speed_mps',
                'left_speed_rad_s', 'right_speed_rad_s', 'motor_torque_nm', 'left_torque_nm', 'right_torque_nm')
    result = {
        'scope': '원시 상태·동시각 구동 진단의 사후 순서 대조. 원인 확정 또는 새 폐루프 시험 아님.',
        'run_valid': report['run_valid'], 'preserved_passed': report['passed'],
        'failed_gates': [k for k, value in report['evaluation']['gates'].items() if not value],
        'events': {'reached_command_1mps': select(reached, status_keys),
                   'first_published_reduction_after_1mps': select(reduced, status_keys),
                   'first_mismatch_at_least_0_35mps': select(mismatch, dyn_keys),
                   'first_published_boundary_hold': select(held, status_keys),
                   'first_reverse_wheel_while_body_forward': select(reverse, dyn_keys),
                   'first_near_stationary_sample_after_hold': select(rest, dyn_keys),
                   'first_published_release': select(released, status_keys),
                   'first_forward_over_0_1_after_release': select(motion, dyn_keys)},
        'same_sample_drivetrain_checks': {
            'samples': len(dynamics),
            'no_reverse_reference_requested': all(r['reference_speed_mps'] >= 0. for r in dynamics),
            'equal_left_right_torque': all(abs(r['left_torque_nm']-r['right_torque_nm']) < 1e-9 for r in dynamics),
            'single_motor_speed_is_ratio15_times_wheel_mean': all(
                abs(r['motor_speed_rad_s']-15.*(r['left_speed_rad_s']+r['right_speed_rad_s'])/2.) < 1e-7 for r in dynamics),
        },
        'input_sha256': {name: hashlib.sha256((RUN/name).read_bytes()).hexdigest() for name in
                         ('report.json', 'autonomy_status.jsonl', 'braking_diagnostics.jsonl')},
        'analysis_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'limits': [
            '일반 상태 발행 약 0.2초/전이 시점, 구동 진단 약 0.02초. first는 수신된 첫 표본이며 실제 내부 최초 시각과 다를 수 있다.',
            '0.35 m/s는 원인 조사 전환 기준이고 실물 센서나 구동 허용 오차가 아니다.',
            '휠 차이와 단기 운동 보정의 결합 가능성은 남는다. 같은 명령/초기 상태의 노면 대조 없이 단일 원인을 확정하지 않는다.',
            '접촉 메시지 부재는 하중 0으로 해석하지 않는다. 별도 left_braking_contacts.json의 null과 구분한다.',
        ],
    }
    with (BASE/'failure_timeline.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({'events': {k: v['sim_time_s'] if v else None for k, v in result['events'].items()},
                      'same_sample_drivetrain_checks': result['same_sample_drivetrain_checks']}, indent=2))


if __name__ == '__main__':
    main()
