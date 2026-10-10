#!/usr/bin/env python3
"""평가용 물리 step 민감도. 같은 고정 요청의 1/0.5/0.25 ms 결과를 비교한다.

원시 실행 판정은 바꾸지 않는다. step 감소에 따른 지표 차이와 함께 실제 적용 step,
진단 표본 주기, 명령 정렬, 실시간 비율을 따로 보고하여 물리엔진 단독 효과로 과장하지 않는다.
"""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path
import re
import statistics

import numpy as np

from analyze_braking_ab import ROOT, rows, sha, summarize_run, sampled_difference, normalized_world
from analyze_zero_speed_modes import model_without_mode, phase_stats, zero_phase
from braking_command_profile import load_profile

LAUNCH = 'src/arena_bringup/launch/simulation.launch.py'
PROBE = 'scripts/probe_braking_contact.py'
CONTRACT = ROOT/'config/tests/physics_step_r0_sha256.json'
BASELINE_STEP = .001
# 실행 전 README에 등록한 해석 허용치: (절대값, 가장 작은 step 값 대비 비율) 중 큰 값.
TOLERANCES = {'velocity': (.05, .10), 'distance': (.01, .20), 'time': (.05, .20)}
PRIMARY_METRICS = {
    'before_zero_peak_difference_mps': 'velocity',
    'after_zero_peak_difference_mps': 'velocity',
    'brake_window_peak_difference_mps': 'velocity',
    'left_right_peak_difference_mps': 'velocity',
    'request_zero_to_stable_chord_m': 'distance',
    'first_zero_to_stable_chord_m': 'distance',
    'request_zero_to_stable_s': 'time',
}
STEP_PATTERN = re.compile(r'<max_step_size>([^<]*)</max_step_size>')


def world_step(path):
    canonical, _ = normalized_world(path)
    values = STEP_PATTERN.findall(canonical)
    if len(values) != 1:
        raise ValueError('runtime world must contain exactly one max_step_size')
    return float(values[0]), STEP_PATTERN.sub('<max_step_size>STEP</max_step_size>', canonical)


def observed_step(data, after):
    """0목표 누적 step 수의 증가율. 진단 표본 사이 모든 step이 0목표인 구간만 쓴다."""
    zeros = [r for r in data if r['sim_time_s'] >= after and r['applied_target_mps'] == 0]
    ratios = [(b['sim_time_s']-a['sim_time_s'])/(b['zero_steps']-a['zero_steps'])
              for a, b in zip(zeros, zeros[1:]) if b['zero_steps'] > a['zero_steps']]
    if not ratios:
        return None
    return {'pairs': len(ratios), 'median_s': statistics.median(ratios), 'min_s': min(ratios), 'max_s': max(ratios)}


def gaps(times):
    values = [b-a for a, b in zip(times, times[1:])]
    if not values:
        return None
    return {'samples': len(times), 'median_s': statistics.median(values), 'max_s': max(values), 'min_s': min(values)}


def request_zero(profile, requests, start):
    """프로필의 마지막 0 전환 시각과 실제 /drive 발행 시각을 구분한다."""
    samples = profile.samples
    last = max(i for i, row in enumerate(samples) if row[1] > 0)
    profile_t = samples[last+1][0]
    published = [r for r in requests if r['sim_s'] >= start]
    positive = [r for r in published if r['speed_mps'] > 0]
    first_zero = next(r for r in published if r['sim_s'] > positive[-1]['sim_s'] and r['speed_mps'] == 0)
    return profile_t, first_zero['sim_s']


def mcu_zero(data, after):
    unique = sorted({r['mcu']['now_us']: r['mcu'] for r in data if r.get('mcu')}.items())
    positive = [t for t, m in unique if m['target_speed_mps'] > 0 and t*1e-6 >= after]
    if not positive:
        return None
    return next((t*1e-6 for t, m in unique if t > positive[-1] and m['target_speed_mps'] == 0), None)


def run_metrics(path):
    summary, report, data = summarize_run(path)
    profile = load_profile(ROOT/report['command_profile']['path'])
    # 지시문 수치가 아니라 보존 파일에서 계산한 해시를 실행 기록과 대조한다.
    profile_sha = sha(ROOT/report['command_profile']['path'])
    start = report['command_start_sim_s']
    mode, model = model_without_mode(path/'runtime_vehicle.sdf')
    declared, world = world_step(path/'runtime_world.sdf')
    phase = zero_phase(data, start)
    begin, end = summary['brake_window_sim_s']
    window = [r for r in data if begin <= r['sim_time_s'] <= end]
    before = [r for r in data if begin <= r['sim_time_s'] < phase['first_zero_sim_s']]
    after = [r for r in data if phase['first_zero_sim_s'] <= r['sim_time_s'] <= end]
    requests = rows(path/'requested_commands.jsonl')
    profile_zero, published_zero = request_zero(profile, requests, start)
    stable = next(r for r in data if r['sim_time_s'] == phase['stable_sim_s']) if phase['stable_sim_s'] else None
    anchor = next((r for r in data if r['sim_time_s'] >= published_zero), None)
    chord = (math.hypot(stable['truth_x_m']-anchor['truth_x_m'], stable['truth_y_m']-anchor['truth_y_m'])
             if stable and anchor else None)
    radius = .0325
    flagged = [(a, b) for a, b in zip(window, window[1:]) if a['truth_longitudinal_mps'] > .1 and
               min(a['left_speed_rad_s'], a['right_speed_rad_s'])*radius < -.03]
    brake_limit = .045
    post = [r for r in data if r['sim_time_s'] >= phase['first_zero_sim_s'] and r['applied_target_mps'] == 0
            and r['sim_time_s'] <= (phase['stable_sim_s'] or end)]
    active = [r['sim_time_s'] for r in data if start <= r['sim_time_s'] <= report['stop_request_sim_s']]
    timing = None
    if 'arm_done_wall_s' in report and 'finish_wall_s' in report:
        sim_span = report['finish_sim_s']-(start-1.)
        wall_span = report['finish_wall_s']-report['arm_done_wall_s']
        timing = {'sim_span_s': sim_span, 'wall_span_s': wall_span, 'real_time_factor': sim_span/wall_span}
    metrics = {
        'before_zero_peak_difference_mps': phase_stats(before)['peak_difference_mps'] if before else None,
        'after_zero_peak_difference_mps': phase_stats(after)['peak_difference_mps'] if after else None,
        'brake_window_peak_difference_mps': summary['peak_mean_wheel_body_difference_mps'],
        'left_right_peak_difference_mps': summary['max_left_right_surface_difference_mps'],
        'request_zero_to_stable_chord_m': chord,
        'first_zero_to_stable_chord_m': phase['zero_to_stable_chord_m'],
        'request_zero_to_stable_s': phase['stable_sim_s']-published_zero if phase['stable_sim_s'] else None,
        'min_mean_wheel_after_zero_mps': min((r['wheel_surface_speed_mps'] for r in after), default=None),
        'min_single_wheel_surface_body_forward_mps': min((min(r['left_speed_rad_s'], r['right_speed_rad_s'])*radius
            for r in window if r['truth_longitudinal_mps'] > .1), default=None),
        'reverse_wheel_samples': summary['body_forward_reverse_wheel_samples'],
        'reverse_wheel_sample_span_s': sum(b['sim_time_s']-a['sim_time_s'] for a, b in flagged),
        'positive_pre_power_samples_window': sum(r['pre_motor_power_w'] > 1e-9 for r in window),
        'peak_positive_pre_power_w_window': max([0., *[r['pre_motor_power_w'] for r in window]]),
        'zero_positive_work_j': phase['zero_positive_work_j'],
        'zero_positive_power_steps': phase['zero_positive_power_steps'],
        'brake_limit_saturated_fraction_after_zero': (sum(abs(abs(r['motor_torque_nm'])-brake_limit) < 1e-9 for r in post)
                                                       / len(post) if post else None),
    }
    return {
        'path': str(path.relative_to(ROOT)), 'surface': report['surface'], 'mode': mode,
        'declared_step_s': declared, 'requested_override_s': report.get('physics_step_override_s'),
        'observed_step': observed_step(data, phase['first_zero_sim_s']),
        'diagnostic_gaps_active': gaps(active), 'timing': timing,
        'wall_elapsed_s': report.get('wall_elapsed_s'),
        'validity': {'report_passed': report['passed'], 'input_valid': report['input_valid'],
                     'stopped': report['stopped'], 'shutdown_clean': report['shutdown_clean'],
                     'stop_acknowledged': report.get('stop_acknowledged'),
                     'final_reason': report.get('final_mcu', {}).get('reason'),
                     'source_unchanged': report['source_unchanged'],
                     'profile_file_matches_report': profile_sha == report['command_profile']['sha256']
                     and report['source_sha256'].get(report['command_profile']['path']) == profile_sha,
                     # 재사용 1 ms 실행은 이후 승인된 launch/probe 변경만큼 현재 소스와 다를 수 있다.
                     'run_audit': all(v for k, v in summary['checks'].items() if k != 'source_still_current'),
                     'source_current_or_approved': current_source_difference(report['source_sha256'])['approved'],
                     'checks': summary['checks'], 'mode_observed': report.get('mode_observed')},
        'current_source_difference': current_source_difference(report['source_sha256']),
        'command_profile_sha256_computed': profile_sha,
        'phase_transition': phase, 'profile_zero_s': profile_zero, 'published_zero_sim_s': published_zero,
        'published_zero_from_start_s': published_zero-start, 'mcu_zero_sim_s': mcu_zero(data, start),
        'first_zero_diag_minus_published_s': phase['first_zero_sim_s']-published_zero,
        'metrics': metrics, 'initial_pose': summary['initial_pose'], 'input_sha256': summary['input_sha256'],
    }, report, data, model, world


def step_matches(run):
    declared = run['declared_step_s']
    requested = run['requested_override_s']
    observed = run['observed_step']
    return bool(observed and (requested is None and declared == BASELINE_STEP or
                              requested is not None and declared == float(requested))
                and all(abs(observed[k]-declared) <= .01*declared for k in ('median_s', 'min_s', 'max_s')))


def source_difference(reference, candidate, contract):
    changed = sorted(k for k in set(reference) | set(candidate) if reference.get(k) != candidate.get(k))
    approved = {**contract['approved_changes'], **contract['approved_evaluation_scripts']}
    ok = set(changed) <= set(approved) and all(
        reference.get(k) == approved[k]['before'] and candidate.get(k) == approved[k]['after'] for k in changed)
    return {'changed': changed, 'approved': ok}


def current_source_difference(recorded, contract_path=CONTRACT):
    current = {k: sha(ROOT/k) for k in recorded}
    return source_difference(recorded, current, json.loads(contract_path.read_text(encoding='utf-8')))


def mismatched_samples(a, b, key, atol=1e-9):
    """sampled_difference와 같은 20 ms 격자 유지값에서 불일치 표본 수와 위치."""
    if not a or not b:
        return {'samples': 0, 'mismatched': None, 'times_s': []}
    ta, tb = [r['t'] for r in a], [r['t'] for r in b]
    total, times = 0, []
    for t in np.arange(max(ta[0], tb[0], 0.), min(ta[-1], tb[-1]), .02):
        ia, ib = bisect_right(ta, t)-1, bisect_right(tb, t)-1
        if t-ta[ia] > .05 or t-tb[ib] > .05:
            continue
        total += 1
        if abs(a[ia][key]-b[ib][key]) > atol:
            times.append(round(float(t), 4))
    return {'samples': total, 'mismatched': len(times), 'times_s': times}


def tolerance(kind, reference):
    absolute, relative = TOLERANCES[kind]
    return max(absolute, relative*abs(reference))


def classify(group):
    """step별 run 묶음(1 ms 포함)의 사전 등록 분류.

    여러 실행의 지표가 허용치 안인지 나눈 것이며 수학적 수렴 증명이 아니다.
    """
    steps = sorted(group, reverse=True)
    if (BASELINE_STEP not in steps or len(steps) < 2
            or any(s not in (BASELINE_STEP, .0005, .00025) for s in steps)):
        raise ValueError(f'classification needs 1 ms plus registered smaller steps, got {steps}')
    finest = steps[-1]
    result, coarse_ok, fine_ok = {}, True, True
    for name, kind in PRIMARY_METRICS.items():
        values = {s: group[s]['metrics'][name] for s in steps}
        if any(v is None for v in values.values()):
            result[name] = {'values': values, 'comparable': False}
            coarse_ok = fine_ok = False
            continue
        tol = tolerance(kind, values[finest])
        row = {'values': values, 'tolerance': tol,
               'baseline_minus_finest': values[BASELINE_STEP]-values[finest] if BASELINE_STEP in values else None}
        if len(steps) == 3:
            row['middle_minus_finest'] = values[steps[1]]-values[finest]
            fine_ok &= abs(row['middle_minus_finest']) <= tol
        coarse_ok &= abs(row['baseline_minus_finest']) <= tol
        result[name] = row
    if len(steps) == 3:
        label = ('consistent_including_1ms' if fine_ok and coarse_ok else
                 'fine_consistent_1ms_distorted' if fine_ok else 'not_converged')
    else:
        label = 'consistent_including_1ms' if coarse_ok else 'differs_at_1ms'
    return {'steps_s': steps, 'classification': label, 'metrics': result}


def analyze(paths, contract_path=CONTRACT):
    contract = json.loads(contract_path.read_text(encoding='utf-8'))
    runs = [run_metrics(p) for p in paths]
    groups = {}
    for item in runs:
        run = item[0]
        group = groups.setdefault((run['surface'], run['mode']), {})
        # 같은 노면/모드/step 두 실행은 덮어쓰기로 비교 대상이 사라지므로 별도 분석으로 나눈다.
        if run['declared_step_s'] in group:
            raise ValueError(f"duplicate run for {run['surface']}/{run['mode']}/{run['declared_step_s']}: "
                             f"{group[run['declared_step_s']][0]['path']} and {run['path']}")
        group[run['declared_step_s']] = item
    comparisons, classifications = [], {}
    for (surface, mode), members in sorted(groups.items()):
        if BASELINE_STEP not in members:
            raise ValueError(f'1 ms reference missing for {surface}/{mode}')
        ref, ref_report, ref_data, ref_model, ref_world = members[BASELINE_STEP]
        for step, (run, report, data, model, world) in sorted(members.items(), reverse=True):
            if step == BASELINE_STEP:
                continue
            streams = {}
            for filename in ('requested_commands', 'safe_commands'):
                pair = [[{**r, 't': r['sim_s']-rep['command_start_sim_s']} for r in rows(ROOT/p['path']/(filename+'.jsonl'))]
                        for p, rep in ((ref, ref_report), (run, report))]
                for key in ('speed_mps', 'steering_rad'):
                    streams[filename+'_'+key] = sampled_difference(*pair, key)
            mcu = []
            for rows_, rep in ((ref_data, ref_report), (data, report)):
                unique = {r['mcu']['now_us']: r['mcu'] for r in rows_ if r.get('mcu')}
                mcu.append([{**v, 't': t*1e-6-rep['command_start_sim_s']} for t, v in sorted(unique.items())])
            for key in ('target_speed_mps', 'steering_rad'):
                streams['mcu_'+key] = sampled_difference(*mcu, key)
            mismatches = {}
            for filename in ('requested_commands', 'safe_commands'):
                pair = [[{**r, 't': r['sim_s']-rep['command_start_sim_s']} for r in rows(ROOT/p['path']/(filename+'.jsonl'))]
                        for p, rep in ((ref, ref_report), (run, report))]
                for key in ('speed_mps', 'steering_rad'):
                    mismatches[filename+'_'+key] = mismatched_samples(*pair, key)
            for key in ('target_speed_mps', 'steering_rad'):
                mismatches['mcu_'+key] = mismatched_samples(*mcu, key)
            checks = {
                'same_profile': ref_report['command_profile']['sha256'] == report['command_profile']['sha256'],
                'same_spawn': ref_report['spawn'] == report['spawn'],
                'same_vehicle_bytes': ref['input_sha256']['runtime_vehicle.sdf'] == run['input_sha256']['runtime_vehicle.sdf'],
                'same_vehicle_except_mode': ref_model == model,
                'same_world_except_step': ref_world == world,
                'step_override_applied': step_matches(run) and step_matches(ref),
                'sources_approved': source_difference(ref_report['source_sha256'], report['source_sha256'], contract)['approved'],
                # 각 발행 요청의 프로필 일치는 실행별 request_audit가 판정한다. 발행 시각 차이는 아래 정렬 차이로 보고한다.
                'both_request_audits': ref_report['request_audit']['passed'] and report['request_audit']['passed'],
            }
            comparisons.append({'surface': surface, 'mode': mode, 'reference': ref['path'], 'candidate': run['path'],
                                'candidate_step_s': step, 'checks': checks, 'passed': all(checks.values()),
                                'source_difference': source_difference(ref_report['source_sha256'], report['source_sha256'], contract),
                                'aligned_commands': streams, 'aligned_mismatches': mismatches,
                                'applied_input_identical': all(v['mismatched'] == 0 for v in mismatches.values()),
                                'zero_latency_difference_s': {
                                    'mcu_zero_minus_published': [r['mcu_zero_sim_s']-r['published_zero_sim_s']
                                                                 if r['mcu_zero_sim_s'] is not None else None for r in (ref, run)],
                                    'first_zero_diag_minus_published': [r['first_zero_diag_minus_published_s'] for r in (ref, run)]},
                                'initial_pose_difference_m': math.hypot(
                                    ref['initial_pose']['truth_x_m']-run['initial_pose']['truth_x_m'],
                                    ref['initial_pose']['truth_y_m']-run['initial_pose']['truth_y_m'])})
        classifications[f'{surface}/{mode}'] = classify({s: m[0] for s, m in members.items()})
    run_valid = all(all(v for k, v in r[0]['validity'].items() if k != 'checks') for r in runs)
    audit = run_valid and all(c['passed'] for c in comparisons)
    if not audit:
        for value in classifications.values():
            value['classification'] = 'invalid_input_or_audit'
    identical = all(c['applied_input_identical'] for c in comparisons)
    return {'audit_passed': audit,
            # audit_passed는 실행/보존 유효성이다. step 단독 효과 식별은 적용 입력 동일성까지 요구한다.
            'execution_preservation_valid': audit,
            'step_only_effect_identifiable': audit and identical,
            'identifiability_basis': ('20 ms 격자 유지값으로 요청/safe/MCU 목표의 속도·조향 불일치 표본 수가 모두 0이어야 한다. '
                                      '0이 아니면 step과 명령 적용 시각 차이가 섞여 있어 step 단독 효과로 해석하지 않는다.'),
            'classification_scope': '사전 등록 허용치에 따른 여러 실행 지표의 분류이며 수학적 수렴 증명이 아니다.',
            'runs': [r[0] for r in runs], 'comparisons': comparisons,
            'classifications': classifications, 'tolerances': TOLERANCES, 'primary_metrics': PRIMARY_METRICS,
            'contract_sha256': hashlib.sha256(contract_path.read_bytes()).hexdigest(),
            'analysis_sha256': sha(Path(__file__)),
            'limits': ['진단은 약 20 ms 간격 표본이다. 매 물리 step의 진동/부호 전환은 직접 관측하지 않는다.',
                       '양의 일은 PreUpdate 동시각 축속도 기반이며 적분 후 전체 에너지 보존이나 실물 수동성의 증명이 아니다.',
                       'step 감소는 실시간 비율을 낮춰 ROS/MCU 콜백의 시뮬레이션 시각 지연을 바꿀 수 있다. 명령 정렬을 별도로 보고한다.',
                       '수렴은 이 고정 요청·두 노면·한 초기 자세에서의 수치 일관성이며 실물 MD20A 제동의 물리 타당성이 아니다.',
                       '정지 이동은 요청 0 발행 뒤 첫 진단 표본 또는 첫 0 진단부터 안정 표본까지 XY 현 길이다.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--contract', type=Path, default=CONTRACT)
    args = parser.parse_args()
    result = analyze([p.resolve() for p in args.runs], args.contract.resolve())
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({'audit_passed': result['audit_passed'],
                      'execution_preservation_valid': result['execution_preservation_valid'],
                      'step_only_effect_identifiable': result['step_only_effect_identifiable'],
                      'classifications': {k: v['classification'] for k, v in result['classifications'].items()},
                      'comparisons': [{k: c[k] for k in ('surface', 'mode', 'candidate_step_s', 'checks')}
                                      for c in result['comparisons']]}, indent=2))
    raise SystemExit(0 if result['audit_passed'] else 1)
