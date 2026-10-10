#!/usr/bin/env python3
"""보존된 A/B와 역순 반복의 마감 요약. 원시 자료/기존 판정은 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def read(path):
    return json.loads(path.read_text())


def main():
    pairs = [read(BASE/name) for name in ('ab_analysis.json', 'repeat_analysis.json')]
    runs = {Path(r['run']).name: r for p in pairs for r in p['runs']}
    ordered = []
    for name in ('official', 'flat', 'flat_repeat', 'official_repeat'):
        run = BASE/name
        report = read(run/'report.json')
        analysis = runs[name]
        data = [json.loads(line) for line in (run/'braking_diagnostics.jsonl').read_text().splitlines()]
        active = [r for r in data if report['command_start_sim_s'] <= r['sim_time_s'] <= report['stop_request_sim_s']]
        # 20 ms 전후 표본 개수이며 연속 지속 시간/적분 에너지로 환산하지 않는다.
        zero_power = [r['motor_torque_nm']*r['motor_speed_rad_s'] for r in active
                      if r['reference_speed_mps'] == 0. and abs(r['motor_speed_rad_s']) > 1e-9]
        runtime = ET.parse(run/'runtime_vehicle.sdf').find("model/plugin[@name='arena::SingleMotorDrive']")
        ordered.append({
            'name': name, 'report_sha256': sha(run/'report.json'),
            'input_valid': report['input_valid'], 'stopped': report['stopped'],
            'shutdown_clean': report['shutdown_clean'],
            'no_remaining_pids': not report['remaining_pids'],
            'source_current': all(sha(ROOT/k) == v for k, v in report['source_sha256'].items()),
            'preserved_inputs': all(sha(ROOT/analysis['run']/k) == v for k, v in analysis['input_sha256'].items()),
            'request_audit': report['request_audit'], 'dynamics_coverage': report['dynamics_coverage'],
            'peak_mean_wheel_body_difference_mps': analysis['peak_mean_wheel_body_difference_mps'],
            'max_left_right_surface_difference_mps': analysis['max_left_right_surface_difference_mps'],
            'reverse_wheel_diagnostic_samples': analysis['body_forward_reverse_wheel_samples'],
            'zero_target_positive_motor_power_samples': sum(p > 0 for p in zero_power),
            'zero_target_peak_positive_motor_power_w': max([0., *zero_power]),
            'drive_parameters': {child.tag: child.text for child in runtime},
        })
    whitebox = read(BASE/'zero_crossing_probe.json')
    checks = {
        'both_pair_audits': all(p['comparison_audit_passed'] for p in pairs),
        'aligned_20ms_command_samples_equal': all(d['samples'] >= 250 and d['max_abs'] == 0.
            for p in pairs for d in p['aligned_command_differences'].values()),
        'all_four_inputs_stop_shutdown_sources': all(all(r[k] for k in (
            'input_valid', 'stopped', 'shutdown_clean', 'no_remaining_pids', 'source_current', 'preserved_inputs'))
            for r in ordered),
        'four_same_drive_parameters': all(r['drive_parameters'] == ordered[0]['drive_parameters'] for r in ordered),
        'whitebox_positive_power_count_matches': whitebox['positive_power_samples'] ==
            sum(row['motor_mechanical_power_w'] > 0 for row in whitebox['samples']),
    }
    result = {'checks': checks, 'audit_passed': all(checks.values()), 'runs_in_execution_order': ordered,
        'zero_crossing_probe': {'positive_power_samples': whitebox['positive_power_samples'],
            'sample_period_s': whitebox['sample_period_s'], 'scope': whitebox['scope'],
            'first_sample': whitebox['samples'][0], 'last_sample': whitebox['samples'][-1]},
        'evidence_sha256': {str(p.relative_to(ROOT)): sha(p) for p in (
            BASE/'ab_analysis.json', BASE/'repeat_analysis.json', BASE/'zero_crossing_probe.json',
            BASE/'zero_crossing_probe.cpp', BASE/'pytest_final.log', BASE/'finalize_analysis.py',
            ROOT/'src/arena_gazebo/include/arena_gazebo/drivetrain.hpp', ROOT/'scripts/analyze_braking_ab.py')},
        'limits': ['20 ms 정렬 표본의 동일성은 모든 콜백/물리 step 시각이 동일하다는 보증이 아니다.',
            'positive motor power는 모터 축의 구동 방향 에너지 전달이다. 차체 가속과 같지 않다.',
            '강제 관절속도 부호 전환 시험은 현행 식의 특성 검사이며 차량 동역학/실물 제동 시험이 아니다.',
            '감사 통과는 입력/기록/정지/종료의 유효성이다. 큰 슬립 해결이나 G1 주행 성공이 아니다.']}
    with (BASE/'final_audit.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(checks, indent=2))
    if not result['audit_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
