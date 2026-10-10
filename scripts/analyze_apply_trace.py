#!/usr/bin/env python3
"""플러그인 명령 수신/적용 이벤트(평가 전용)와 같은 조건 반복의 물리 분기 대조.

용어(이 분석의 정의):
- spawn: 플러그인 첫 PreUpdate(`first_update`)의 iteration·sim 시각. 차량이 월드에 들어온 step의 관측값이다.
- 수신(rx): 해당 값을 처음 실어 온 gz 메시지가 플러그인 콜백에 도착한 순간. sim 시각은 그 직전에
  잠금 구역을 지난 PreUpdate의 UpdateInfo::simTime(`rx_after_sim_ns`)으로만 안다.
- 적용(apply): 감시 만료 반영 후 구동/조향 계산에 실제로 넘긴 값이 바뀐 첫 PreUpdate의 iteration·simTime.
- 진단 분기: 두 실행의 /sim/drivetrain 표본(같은 sim 시각, 12자리 출력 정밀도)에서 값이 처음 달라진 시각.
분류는 실행 전 README에 등록한 규칙을 따른다. 2회 반복은 분포가 아니다.
"""
import argparse
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from analyze_braking_ab import ROOT, rows, sha
from analyze_physics_step import run_metrics
from analyze_scheduled_phase import VALUE_TOL, compare_metrics, delivery, posthoc_validity, schedule_fidelity
from braking_command_profile import load_profile
from scheduled_command import ProfileSampler

CONTRACT = ROOT/'config/tests/scheduled_command_r2_sha256.json'
DIAG_KEYS = ('truth_x_m', 'truth_y_m', 'truth_z_m', 'truth_yaw_rad', 'truth_longitudinal_mps', 'truth_lateral_mps',
             'left_speed_rad_s', 'right_speed_rad_s', 'left_steering_rad', 'right_steering_rad', 'motor_torque_nm',
             'applied_target_mps')
WHEEL_KEYS = ('left_speed_rad_s', 'right_speed_rad_s', 'truth_longitudinal_mps')
DIVERGENCE_LEVELS = (1e-9, 1e-6, 1e-3, 1e-1)


def spawn_from_diagnostics(data):
    """추적이 없는 과거 실행용: 첫 진단 표본 시각(플러그인 첫 PostUpdate)과 그때 차체 높이."""
    first = data[0]
    return {'first_diag_sim_ns': round(first['sim_time_s']*1e9), 'first_diag_truth_z_m': first['truth_z_m']}


def apply_events(events, channel, cause='command'):
    return [e for e in events if e.get('type') == 'apply' and e.get('channel') == channel
            and (cause is None or e.get('cause') == cause)]


def trace_summary(events, start_ns, end_ns):
    first = next((e for e in events if e.get('type') == 'first_update'), None)
    steps = [e for e in events if e.get('type') in ('first_update', 'summary', 'apply')]
    offsets = sorted({e['sim_ns']-e['iteration']*e['dt_ns'] for e in steps})
    out = {'spawn': {k: first[k] for k in ('iteration', 'sim_ns', 'dt_ns')} if first else None,
           'sim_ns_minus_iteration_dt_ns': offsets,
           'watchdog_applies': len(apply_events(events, 'speed', 'watchdog'))+len(apply_events(events, 'steering', 'watchdog')),
           'last_summary': next((e for e in reversed(events) if e.get('type') == 'summary'), None)}
    for channel in ('speed', 'steering'):
        window = [e for e in apply_events(events, channel) if start_ns <= e['sim_ns'] <= end_ns]
        out[channel] = [{'to': e['to'], 'apply_rel_ns': e['sim_ns']-start_ns, 'apply_iteration': e['iteration'],
                         'rx_after_rel_ns': e['rx_after_sim_ns']-start_ns, 'steps_after_rx': e['steps_after_rx'],
                         'rx_to_apply_wall_ns': e['rx_to_apply_wall_ns'], 'rx_seq_first': e['rx_seq_first'],
                         'rx_seq_latest': e['rx_seq_latest'], 'rx_superseded_total': e['rx_superseded_total']}
                        for e in window]
    return out


def stage_alignment(trace, stages):
    """프로필 전환 순서대로 MCU/출력 단계와 플러그인 적용을 짝짓는다. 개수·새 값이 다르면 None."""
    out = {}
    for channel in ('speed', 'steering'):
        applied = trace[channel]
        profile = stages[channel]['stages_rel_start_ns']
        if len(applied) != stages[channel]['events']:
            out[channel] = {'matched': False, 'applies': len(applied), 'expected': stages[channel]['events']}
            continue
        mcu, output = profile.get('mcu'), profile.get('output')
        rows_ = []
        for i, event in enumerate(applied):
            row = {'apply_rel_ns': event['apply_rel_ns'], 'steps_after_rx': event['steps_after_rx'],
                   'rx_after_rel_ns': event['rx_after_rel_ns']}
            if mcu:
                row['apply_minus_mcu_ns'] = event['apply_rel_ns']-mcu[i]
            if output:
                row['apply_minus_output_ns'] = event['apply_rel_ns']-output[i]
                row['rx_after_minus_output_ns'] = event['rx_after_rel_ns']-output[i]
            rows_.append(row)
        out[channel] = {'matched': True, 'rows': rows_}
    return out


def registered_sources(recorded, contract):
    for path, change in contract['approved_evaluation_scripts'].items():
        if path in recorded and recorded[path] != change['after']:
            return False
    for path, change in contract['approved_changes'].items():
        if path in recorded and recorded[path] != change['after']:
            return False
    for path, value in contract['implementation_sha256'].items():
        if path in recorded and recorded[path] != value:
            return False
    return True


def analyze_run(path, contract, expected_plugin_sha):
    run, report, data, model, world = run_metrics(path)
    spec = report.get('command_schedule') or {}
    if spec.get('mode') != 'sim_grid':
        raise ValueError(f'{path} is not a sim_grid run')
    sampler = ProfileSampler(load_profile(ROOT/report['command_profile']['path']))
    dt_ns = round(run['declared_step_s']*1e9)
    start = spec['start_ns']
    events = rows(path/'plugin_command_events.jsonl') if (path/'plugin_command_events.jsonl').is_file() else []
    stages = delivery(path, report, data, sampler, dt_ns)
    trace = trace_summary(events, start, start+sampler.duration_ns+200_000_000) if events else None
    validity = {'report_passed': report['passed'] is True, 'input_valid': report['input_valid'] is True,
                'stopped': report['stopped'] is True, 'shutdown_clean': report['shutdown_clean'] is True,
                'stop_acknowledged': report.get('stop_acknowledged') is True,
                'final_reason_software_stop': (report.get('final_mcu') or {}).get('reason') == 'software_stop',
                'source_unchanged': report['source_unchanged'] is True,
                'sources_registered_r2': registered_sources(report['source_sha256'], contract),
                'plugin_binary_registered': report.get('plugin_binary_sha256') == expected_plugin_sha,
                'request_audit': report['request_audit']['passed'] is True,
                'all_ticks_audit': (report.get('request_audit_all_ticks') or {}).get('passed') is True,
                'plugin_trace_valid': (report.get('plugin_command_trace') or {}).get('valid') is True,
                'no_fault_injection': report.get('evaluation_fault_injection_requested') is None,
                **posthoc_validity(report, data, sampler)}
    validity.pop('runner_validity_block_missing', None)
    validity.pop('posthoc_dynamics_max_gap_s', None)
    validity['passed'] = all(v is True for v in validity.values())
    return {'path': run['path'], 'step_s': run['declared_step_s'], 'phase_ns': spec['phase_ns'], 'validity': validity,
            'schedule': schedule_fidelity(path, report, sampler), 'delivery': stages, 'trace': trace,
            'alignment': stage_alignment(trace, stages) if trace else None,
            'spawn_from_diagnostics': spawn_from_diagnostics(data), 'metrics': run['metrics'],
            'timing': run['timing'], 'initial_pose': run['initial_pose'],
            'plugin_binary_sha256': report.get('plugin_binary_sha256')}, data, model, world


def divergence(data_a, data_b, start_ns):
    """같은 sim 시각 진단 표본의 최초 차이와 수준별 최초 초과 시각(프로필 시작 기준 ns)."""
    a = {round(r['sim_time_s']*1e9): r for r in data_a}
    b = {round(r['sim_time_s']*1e9): r for r in data_b}
    common = sorted(set(a) & set(b))
    first_any, first_keys = None, []
    exceed = {str(level): None for level in DIVERGENCE_LEVELS}
    for t in common:
        diffs = {k: abs(a[t][k]-b[t][k]) for k in DIAG_KEYS}
        if first_any is None and any(diffs.values()):
            first_any, first_keys = t, sorted(k for k, v in diffs.items() if v)
        wheel = max(diffs[k] for k in WHEEL_KEYS)
        for level in DIVERGENCE_LEVELS:
            if exceed[str(level)] is None and wheel > level:
                exceed[str(level)] = t-start_ns
    return {'common_samples': len(common), 'only_a': len(set(a)-set(b)), 'only_b': len(set(b)-set(a)),
            'first_any_difference_rel_ns': None if first_any is None else first_any-start_ns,
            'first_any_difference_keys': first_keys,
            'first_wheel_or_body_speed_exceeds_rel_ns': exceed,
            'precision': '진단 JSON 12자리 유효숫자. 같다는 것은 이 정밀도에서 같다는 뜻'}


def first_application_difference(trace_a, trace_b):
    out, first = {}, None
    for channel in ('speed', 'steering'):
        x, y = trace_a[channel], trace_b[channel]
        if len(x) != len(y) or any(abs(p['to']-q['to']) > VALUE_TOL for p, q in zip(x, y)):
            out[channel] = {'comparable': False, 'applies': [len(x), len(y)]}
            continue
        diffs = [q['apply_rel_ns']-p['apply_rel_ns'] for p, q in zip(x, y)]
        out[channel] = {'comparable': True, 'apply_diff_ns': diffs, 'differing': sum(d != 0 for d in diffs),
                        'steps_after_rx': [[p['steps_after_rx'], q['steps_after_rx']] for p, q in zip(x, y)]}
        for p, d in zip(x, diffs):
            if d:
                first = p['apply_rel_ns'] if first is None else min(first, p['apply_rel_ns'])
                break
    out['first_difference_rel_ns'] = first
    return out


def classify(spawn_same, applications, diverge):
    """README 등록 분류. 원인 확정이 아니라 관측된 선후 관계다."""
    first_app = applications['first_difference_rel_ns']
    first_div = diverge['first_any_difference_rel_ns']
    comparable = all(applications[c]['comparable'] for c in ('speed', 'steering'))
    if not comparable:
        return 'applications_not_comparable'
    if not spawn_same:
        return 'spawn_differs_initial_state_not_identical'
    if first_app is None:
        return ('reproduced_at_print_precision_given_same_spawn_and_applications' if first_div is None
                else 'diverged_without_traced_application_difference')
    if first_div is None:
        return 'application_difference_without_observed_divergence'
    return ('divergence_onset_not_before_first_application_difference' if first_div >= first_app
            else 'divergence_precedes_first_application_difference')


def runtime_model_without_trace(path):
    root = ET.parse(path).getroot()
    for plugin in root.iter('plugin'):
        for node in plugin.findall('evaluation_command_trace'):
            plugin.remove(node)
    return ET.tostring(root)


def compare(a, b, data_a, data_b, start_a, start_b):
    if start_a != start_b:
        raise ValueError('profile start differs; comparison is not registered')
    diverge = divergence(data_a, data_b, start_a)
    if a['trace'] and b['trace']:
        spawn_same = a['trace']['spawn'] == b['trace']['spawn']
        applications = first_application_difference(a['trace'], b['trace'])
        label = classify(spawn_same, applications, diverge)
    else:
        spawn_same = a['spawn_from_diagnostics'] == b['spawn_from_diagnostics']
        applications, label = None, 'no_plugin_trace_retrospective_only'
    metrics, within, secondary = compare_metrics(a, b)
    return {'a': a['path'], 'b': b['path'], 'both_valid': a['validity']['passed'] and b['validity']['passed'],
            'spawn_same': spawn_same, 'spawn': [a['trace'] and a['trace']['spawn'], b['trace'] and b['trace']['spawn']],
            'spawn_from_diagnostics': [a['spawn_from_diagnostics'], b['spawn_from_diagnostics']],
            'applications': applications, 'divergence': diverge, 'classification': label,
            'metrics': metrics, 'metrics_within_tolerance': within, 'secondary_metrics': secondary}


def instrumentation_invariance(off_dir, on_dir):
    """같은 빌드에서 계측 끔/켬 실행 사본과 진단 스키마 차이. 물리 결과 비교가 아니다."""
    keys = lambda d: sorted(json.loads(Path(d, 'braking_diagnostics.jsonl').open(encoding='utf-8').readline()))
    return {'runtime_world_identical': sha(Path(off_dir, 'runtime_world.sdf')) == sha(Path(on_dir, 'runtime_world.sdf')),
            'runtime_vehicle_identical_except_trace_element':
                runtime_model_without_trace(Path(off_dir, 'runtime_vehicle.sdf'))
                == runtime_model_without_trace(Path(on_dir, 'runtime_vehicle.sdf')),
            'trace_element_only_in_on_run': b'evaluation_command_trace' not in Path(off_dir, 'runtime_vehicle.sdf').read_bytes()
                                            and b'evaluation_command_trace' in Path(on_dir, 'runtime_vehicle.sdf').read_bytes(),
            'diagnostic_keys_identical': keys(off_dir) == keys(on_dir)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True, help='name=path')
    parser.add_argument('--pair', action='append', default=[], help='a:b')
    parser.add_argument('--invariance', action='append', default=[], help='off_name:on_name')
    parser.add_argument('--plugin-sha256', required=True, help='등록한 빌드 플러그인 바이너리 해시')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    contract = json.loads(CONTRACT.read_text(encoding='utf-8'))
    runs, data, starts, paths = {}, {}, {}, {}
    for item in args.run:
        name, path = item.split('=', 1)
        if name in runs:
            raise ValueError(f'duplicate run {name}')
        path = Path(path).resolve()
        runs[name], data[name], _, _ = analyze_run(path, contract, args.plugin_sha256)
        starts[name] = json.loads((path/'report.json').read_text(encoding='utf-8'))['command_schedule']['start_ns']
        paths[name] = path
    comparisons = []
    for item in args.pair:
        a, b = item.split(':')
        comparisons.append({'names': [a, b], **compare(runs[a], runs[b], data[a], data[b], starts[a], starts[b])})
    invariance = {item: instrumentation_invariance(paths[item.split(':')[0]], paths[item.split(':')[1]])
                  for item in args.invariance}
    result = {'runs': runs, 'comparisons': comparisons, 'instrumentation_invariance': invariance,
              'contract_sha256': hashlib.sha256(CONTRACT.read_bytes()).hexdigest(), 'analysis_sha256': sha(Path(__file__)),
              'limits': ['2회 반복은 분포가 아니다. 분류는 관측된 선후 관계이며 인과 확정이 아니다.',
                         '수신 sim 시각은 직전 PreUpdate 시각까지만 안다(전송 지연의 벽시계는 별도 기록).',
                         '진단은 약 20 ms 표본이므로 분기 시각은 그 해상도 안에서만 정해진다.',
                         'passive_damping은 평가 전용 근사이며 실물 MD20A 회로가 아니다.']}
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({'validity': {k: v['validity']['passed'] for k, v in runs.items()},
                      'spawn': {k: (v['trace'] or {}).get('spawn') for k, v in runs.items()},
                      'comparisons': [{k: c[k] for k in ('names', 'classification', 'spawn_same', 'metrics_within_tolerance')}
                                      | {'first_div': c['divergence']['first_any_difference_rel_ns'],
                                         'first_app': (c['applications'] or {}).get('first_difference_rel_ns')}
                                      for c in comparisons],
                      'invariance': invariance}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
