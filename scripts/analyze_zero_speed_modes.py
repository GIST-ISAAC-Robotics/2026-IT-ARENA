#!/usr/bin/env python3
"""평가용 0목표 대조. 기존 입력/원시 감사와 동시각 동력 계측을 분리한다."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from analyze_braking_ab import (ROOT, rows, sha, summarize_run, sampled_difference, normalized_world)


def model_without_mode(path):
    root = ET.parse(path).getroot()
    plugin = root.find("model/plugin[@name='arena::SingleMotorDrive']")
    node = plugin.find('evaluation_zero_speed_mode')
    mode = 'legacy' if node is None else node.text
    if node is not None:
        plugin.remove(node)
    return mode, ET.canonicalize(ET.tostring(root, encoding='unicode'), strip_text=True)


def phase_stats(data):
    if not data:
        return None
    peak = max(data, key=lambda r: abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']))
    return {'samples': len(data), 'first_sim_s': data[0]['sim_time_s'], 'last_sim_s': data[-1]['sim_time_s'],
            'peak_difference_mps': abs(peak['wheel_surface_speed_mps']-peak['truth_longitudinal_mps']),
            'peak_body_mps': peak['truth_longitudinal_mps'], 'peak_wheel_mps': peak['wheel_surface_speed_mps'],
            'peak_sim_s': peak['sim_time_s'],
            'min_wheel_mean_mps': min(r['wheel_surface_speed_mps'] for r in data),
            'positive_pre_power_samples': sum(r['pre_motor_power_w'] > 1e-9 for r in data),
            'peak_positive_pre_power_w': max([0., *[r['pre_motor_power_w'] for r in data]]),
            'max_abs_integral_nm': max(abs(r['pi_integral_after_nm']) for r in data)}


def zero_phase(data, start):
    active = [r for r in data if r['sim_time_s'] >= start]
    positives = [r for r in active if r['applied_target_mps'] > 0]
    if not positives:
        raise ValueError('no positive applied target')
    last_positive = positives[-1]['sim_time_s']
    zeros = [r for r in active if r['sim_time_s'] > last_positive and r['applied_target_mps'] == 0]
    if len(zeros) < 25:
        raise ValueError('zero target coverage insufficient')
    initial = zeros[0]
    stable = None
    for i, r in enumerate(zeros):
        tail = zeros[i:]
        if (tail[-1]['sim_time_s']-r['sim_time_s'] >= .45 and all(
            math.hypot(v['truth_longitudinal_mps'], v['truth_lateral_mps']) < .05 and
            abs(v['wheel_surface_speed_mps']) < .05 for v in tail)):
            stable = r
            break
    distance = (math.hypot(stable['truth_x_m']-initial['truth_x_m'], stable['truth_y_m']-initial['truth_y_m'])
                if stable else None)
    return {'last_positive_sim_s': last_positive, 'first_zero_sim_s': initial['sim_time_s'],
            'zero_transition_bracket_s': initial['sim_time_s']-last_positive,
            'body_at_first_zero_mps': initial['truth_longitudinal_mps'],
            'stable_sim_s': stable['sim_time_s'] if stable else None,
            'zero_to_stable_chord_m': distance,
            'zero_positive_work_j': zeros[-1]['zero_positive_work_j']-positives[0]['zero_positive_work_j'],
            'zero_positive_power_steps': zeros[-1]['zero_positive_power_steps']-positives[0]['zero_positive_power_steps']}


def analyze(paths):
    summaries, raw, reports, models = [], [], [], []
    for path in paths:
        summary, report, data = summarize_run(path)
        mode, model = model_without_mode(path/'runtime_vehicle.sdf')
        mode_ok = mode == report['zero_speed_mode'] and all(r['evaluation_zero_speed_mode'] == mode for r in data)
        phase = zero_phase(data, report['command_start_sim_s'])
        begin, end = summary['brake_window_sim_s']
        before = [r for r in data if begin <= r['sim_time_s'] < phase['first_zero_sim_s']]
        after = [r for r in data if phase['first_zero_sim_s'] <= r['sim_time_s'] <= end]
        arithmetic = all(math.isclose(r['pre_motor_power_w'], r['motor_torque_nm']*r['pre_motor_speed_rad_s'],
                                     rel_tol=1e-8, abs_tol=1e-10) for r in data)
        # 구동 특성 판정과 입력/정지 유효성은 별도다. 수동 모드 양의 일은 완화하지 않는다.
        passive_ok = (phase['zero_positive_work_j'] <= 1e-12 and phase['zero_positive_power_steps'] == 0
                      if mode == 'passive_damping' else None)
        summaries.append({'path': str(path.relative_to(ROOT)), 'mode': mode, 'surface': report['surface'],
            'input_stop_source_audit': summary['audit_passed'], 'mode_valid': mode_ok,
            'pre_power_arithmetic': arithmetic, 'passive_invariant_passed': passive_ok,
            'initial_pose': summary['initial_pose'], 'initial_speed_mps': summary['initial_body_speed_mps'],
            'phase_transition': phase, 'braking_before_zero': phase_stats(before),
            'braking_after_zero': phase_stats(after),
            'full_brake_peak_difference_mps': summary['peak_mean_wheel_body_difference_mps'],
            'left_right_peak_difference_mps': summary['max_left_right_surface_difference_mps'],
            'safety_reason_samples': summary['safety_reason_samples'],
            'wheel_contact_stats': summary['wheel_contact_stats'], 'input_sha256': summary['input_sha256']})
        raw.append(data)
        reports.append(report)
        models.append(model)
    pairs = []
    for i in range(1, len(paths)):
        streams = []
        deltas = {}
        for filename in ('requested_commands', 'safe_commands'):
            streams = [[{**r, 't': r['sim_s']-reports[j]['command_start_sim_s']}
                        for r in rows(paths[j]/(filename+'.jsonl'))] for j in (0, i)]
            for key in ('speed_mps', 'steering_rad'):
                deltas[filename+'_'+key] = sampled_difference(*streams, key)
        streams = []
        for j in (0, i):
            unique = {r['mcu']['now_us']: r['mcu'] for r in raw[j] if r.get('mcu')}
            streams.append([{**v, 't': t*1e-6-reports[j]['command_start_sim_s']} for t,v in sorted(unique.items())])
        for key in ('target_speed_mps', 'steering_rad'):
            deltas['mcu_'+key] = sampled_difference(*streams, key)
        world0, _ = normalized_world(paths[0]/'runtime_world.sdf')
        world1, _ = normalized_world(paths[i]/'runtime_world.sdf')
        pairs.append({'a': paths[0].name, 'b': paths[i].name,
            'same_sources': reports[0]['source_sha256'] == reports[i]['source_sha256'],
            'same_profile': reports[0]['command_profile']['sha256'] == reports[i]['command_profile']['sha256'],
            'same_spawn': reports[0]['spawn'] == reports[i]['spawn'],
            'same_vehicle_except_mode': models[0] == models[i],
            'same_world_except_surface': world0 == world1, 'aligned_commands': deltas})
    audit = all(r['input_stop_source_audit'] and r['mode_valid'] and r['pre_power_arithmetic'] for r in summaries)
    audit &= all(all(p[k] for k in ('same_sources','same_profile','same_spawn','same_vehicle_except_mode',
                                     'same_world_except_surface')) for p in pairs)
    return {'audit_passed': audit, 'runs': summaries, 'comparisons': pairs, 'analysis_sha256': sha(Path(__file__)),
            'limits': ['목표 0 전환은 약 20 ms 진단 표본으로 둘러싼 구간이다. 실제 콜백 시각이 아니다.',
                       '0명령~정지 거리는 첫 0 진단부터 안정 표본까지 차체 XY 현 길이이며 실차 제동거리가 아니다.',
                       '양의 일은 물리 step 직전 축속도 기반 합이며 수치 적분 전체 에너지 보존의 증명이 아니다.',
                       '명시 STOP은 보통 차량이 이미 0목표로 감속한 뒤다. 이동 중 비상정지 시험이 아니다.',
                       '현재 모드들은 실물 회로를 재현하지 않으며 평균축 PI는 이상적 관절 피드백이다.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = analyze([p.resolve() for p in args.runs])
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({'audit_passed': result['audit_passed'], 'runs': [{k:r[k] for k in (
        'path','mode','full_brake_peak_difference_mps','phase_transition','passive_invariant_passed')}
        for r in result['runs']]}, indent=2))
    raise SystemExit(0 if result['audit_passed'] else 1)
