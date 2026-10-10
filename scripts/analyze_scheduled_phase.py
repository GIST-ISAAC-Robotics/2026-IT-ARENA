#!/usr/bin/env python3
"""격자 발행 고정 요청(sim_grid)의 실행 유효성·스케줄 충실도·전달 재현성·물리 지표·식별 가능성.

다섯 축을 따로 판정한다. 20 ms 격자 표본값 일치만으로 동일 입력을 주장하지 않으며,
step 단독 효과(`step_only_effect_identifiable`)는 플러그인 적용 시각 양자화와 이산화가 함께 바뀌므로 항상 false다.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from analyze_braking_ab import ROOT, rows, sha
from analyze_physics_step import TOLERANCES, PRIMARY_METRICS, mismatched_samples, run_metrics, step_matches
from braking_command_profile import load_profile
from scheduled_command import ProfileSampler, audit_scheduled_requests

CONTRACT = ROOT/'config/tests/scheduled_command_r0_sha256.json'
SAFETY_PERIOD_NS = 20_000_000
VALUE_TOL = 1e-5  # AckermannDrive float32 왕복 허용
# 실행 전 README에 등록한 전달 비교 허용치(ns).
DELIVERY_TOL_NS = {'request': 0, 'request_observed': 0, 'safe': 0, 'mcu': 1_000_000, 'mcu_hold': 2_000_000, 'output': 2_000_000,
                   'plugin_final_zero': 2_000_000}
SECONDARY_METRICS = ('min_mean_wheel_after_zero_mps', 'min_single_wheel_surface_body_forward_mps',
                     'reverse_wheel_samples', 'positive_pre_power_samples_window', 'peak_positive_pre_power_w_window',
                     'zero_positive_work_j', 'brake_limit_saturated_fraction_after_zero')


def change_events(series, tol=VALUE_TOL):
    """(t_ns, value) 시계열의 값 변화 사건. 첫 값은 기준선으로만 쓴다."""
    events, previous = [], None
    for t, value in series:
        if previous is not None and abs(value-previous) > tol:
            events.append((t, previous, value))
        previous = value
    return events


def profile_events(sampler, index):
    out, previous = [], sampler.values[0][index]
    for t, values in zip(sampler.times_ns, sampler.values):
        if abs(values[index]-previous) > VALUE_TOL:
            out.append((t, previous, values[index]))
        previous = values[index]
    return out


def match(expected, events):
    """프로필 전환과 같은 개수·같은 새 값 순서일 때만 대응시킨다. 아니면 None(식별 불가)."""
    if len(events) != len(expected) or any(abs(e[2]-x[2]) > VALUE_TOL for e, x in zip(events, expected)):
        return None
    return [e[0] for e in events]


def plugin_final_zero_ns(diagnostics, start_ns, dt_ns):
    """마지막 양수→0 목표 전환의 플러그인 적용 시각(step 단위). zero_steps 누적 증가로 역산한다.

    전환 직전 표본 a(목표>0)와 첫 0 표본 b 사이에 0목표 step이 n개면 첫 0 step은 b-(n-1)·dt다.
    표본 사이 목표가 다시 양수가 되었을 가능성은 b 이후가 계속 0인지로만 막는다.
    """
    data = [(round(r['sim_time_s']*1e9), r) for r in diagnostics]
    positive = [i for i, (t, r) in enumerate(data) if t >= start_ns and r['applied_target_mps'] > 0]
    if not positive or positive[-1]+1 >= len(data):
        return None
    (ta, a), (tb, b) = data[positive[-1]], data[positive[-1]+1]
    count = b['zero_steps']-a['zero_steps']
    span = round((tb-ta)/dt_ns)
    if b['applied_target_mps'] != 0 or not 1 <= count <= span:
        return None
    return {'first_zero_step_ns': tb-(count-1)*dt_ns-start_ns, 'bracket_ns': [ta-start_ns, tb-start_ns],
            'zero_steps_in_bracket': count, 'steps_in_bracket': span}


def delivery(run_dir, report, data, sampler, dt_ns):
    start = report['command_schedule']['start_ns']
    end = start+sampler.duration_ns+200_000_000
    window = lambda t: start-50_000_000 <= t <= end
    requests = rows(run_dir/'requested_commands.jsonl')
    safe = [(round(r['sim_s']*1e9), r) for r in rows(run_dir/'safe_commands.jsonl')]
    mcu = sorted(((r['now_us']*1000, r) for r in rows(run_dir/'actuation_status.jsonl')
                  if type(r.get('now_us')) is int and r.get('target_speed_mps') is not None), key=lambda x: x[0])
    output = sorted(((r['now_us']*1000, r) for r in rows(run_dir/'output_status.jsonl')
                     if type(r.get('now_us')) is int), key=lambda x: x[0])
    streams = {
        'speed': {'request': [(r['scheduled_ns'], r['speed_mps']) for r in requests if window(r['scheduled_ns'])],
                  'request_observed': [(r['observed_ns'], r['speed_mps']) for r in requests if window(r['scheduled_ns'])],
                  'safe': [(t, r['speed_mps']) for t, r in safe if window(t)],
                  'mcu': [(t, r['target_speed_mps']) for t, r in mcu if window(t)],
                  'output': [(t, r['speed_mps']) for t, r in output if window(t)]},
        'steering': {'request': [(r['scheduled_ns'], r['steering_rad']) for r in requests if window(r['scheduled_ns'])],
                     'request_observed': [(r['observed_ns'], r['steering_rad']) for r in requests if window(r['scheduled_ns'])],
                     'safe': [(t, r['steering_rad']) for t, r in safe if window(t)],
                     'mcu': [(t, r['steering_rad']) for t, r in mcu if window(t)]},
    }
    result = {}
    for quantity, index in (('speed', 0), ('steering', 1)):
        expected = profile_events(sampler, index)
        stages = {'profile': [e[0] for e in expected]}
        for name, series in streams[quantity].items():
            matched = match(expected, change_events(series))
            stages[name] = None if matched is None else [t-start for t in matched]
        lags = {}
        for a, b in (('profile', 'request'), ('request', 'request_observed'), ('request_observed', 'safe'),
                     ('safe', 'mcu'), ('mcu', 'output'), ('profile', 'mcu')):
            if stages.get(a) is not None and stages.get(b) is not None:
                values = [y-x for x, y in zip(stages[a], stages[b])]
                lags[f'{b}_minus_{a}_ns'] = {'min': min(values), 'max': max(values), 'values': values}
            elif a in stages and b in stages:
                lags[f'{b}_minus_{a}_ns'] = 'unknown'
        holds = None
        if stages.get('mcu') is not None:
            holds = [b-a for a, b in zip(stages['mcu'], stages['mcu'][1:])]
        result[quantity] = {'events': len(expected), 'stages_rel_start_ns': stages, 'lags': lags, 'mcu_hold_ns': holds,
                            'unmatched_stages': sorted(k for k, v in stages.items() if v is None)}
    active_safe = [t for t, _ in safe if start <= t <= start+sampler.duration_ns]
    phases = {}
    for t in active_safe:
        phase = round((t % SAFETY_PERIOD_NS)/1000)*1000 % SAFETY_PERIOD_NS
        phases[phase] = phases.get(phase, 0)+1
    plugin = plugin_final_zero_ns(data, start, dt_ns)
    final_output = result['speed']['stages_rel_start_ns'].get('output')
    # 수신 순서(평가기 구독 순서)에서 같은 now_us의 이전 목표 상태가 먼저 왔는지. AppliedGuard는
    # stamp<=last_stamp 상태를 버리므로, 그런 사건은 다음 MCU tick 상태까지 출력 반영이 늦어진다.
    arrival = [r for r in rows(run_dir/'actuation_status.jsonl') if type(r.get('now_us')) is int]
    shadowed = []
    for t in result['speed']['stages_rel_start_ns'].get('mcu') or []:
        stamp = (t+start)//1000
        same = [r['target_speed_mps'] for r in arrival if r['now_us'] == stamp]
        shadowed.append(len(same) > 1 and abs(same[0]-same[-1]) > VALUE_TOL)
    result['speed']['same_stamp_old_status_first'] = shadowed if result['speed']['stages_rel_start_ns'].get('mcu') else None
    return {**result, 'safe_phase_ns_histogram': sorted(phases.items()),
            'safe_reasons_active': sorted({(r.get('safety') or {}).get('reason', 'missing') for r in data
                                           if start <= round(r['sim_time_s']*1e9) <= start+sampler.duration_ns}),
            'plugin_final_zero': plugin,
            'plugin_final_zero_minus_output_ns': (plugin['first_zero_step_ns']-final_output[-1]
                                                  if plugin and final_output else 'unknown'),
            'intermediate_plugin_application': 'unknown: 진단이 약 20 ms 표본이라 중간 전환의 플러그인 적용 시각은 구간 경계만 안다',
            'counts': {'requests': len(requests), 'safe': len(safe), 'mcu_status': len(mcu), 'output_status': len(output)}}


def schedule_fidelity(run_dir, report, sampler):
    spec = report['command_schedule']
    requests = rows(run_dir/'requested_commands.jsonl')
    audit = audit_scheduled_requests(requests, sampler, spec['start_ns'], spec['start_ns']+sampler.duration_ns,
                                     spec['period_ns'], spec['phase_ns'])
    clock_gap = [r['observed_ns']-r['clock_msg_ns'] for r in requests if r.get('clock_msg_ns') is not None]
    return {'recomputed_audit': audit, 'report_audit_passed': report['request_audit']['passed'],
            'recomputed_matches_report': audit['checks'] == report['request_audit']['checks'],
            'all_ticks_lateness_ns': {'max': max(r['lateness_ns'] for r in requests),
                                      'nonzero': sum(r['lateness_ns'] > 0 for r in requests), 'ticks': len(requests)},
            'missed_ticks_total': spec['diagnostics']['missed_ticks'],
            'duplicate_clock_polls': spec['diagnostics']['duplicate_clock_polls'],
            'start_on_registered_phase': spec['start_ns'] % spec['period_ns'] == spec['phase_ns'],
            'clock_observation': report.get('clock_observation'),
            'node_clock_minus_clock_subscription_ns': ({'min': min(clock_gap), 'max': max(clock_gap),
                                                        'nonzero': sum(v != 0 for v in clock_gap)} if clock_gap else None),
            'passed': audit['passed'] and spec['diagnostics']['missed_ticks'] == 0}


def posthoc_validity(report, data, sampler):
    start = report['command_start_sim_s']
    stop_elapsed = sampler.duration_ns/1e9+1.
    active = [r for r in data if start <= r['sim_time_s'] < start+stop_elapsed]
    final = [r for r in data if start+stop_elapsed+.5 <= r['sim_time_s'] <= start+stop_elapsed+1.]
    gaps = [b['sim_time_s']-a['sim_time_s'] for a, b in zip(active, active[1:])]
    coverage = bool(gaps and min(gaps) > 0 and max(gaps) <= .05 and active[0]['sim_time_s']-start <= .05
                    and start+stop_elapsed-active[-1]['sim_time_s'] <= .05)
    return {'runner_validity_block_missing': 'dynamics_coverage' not in report,
            'posthoc_dynamics_coverage': coverage,
            'posthoc_dynamics_max_gap_s': max(gaps, default=None),
            'posthoc_final_stationary_coverage': bool(len(final) >= 20 and final[-1]['sim_time_s']-final[0]['sim_time_s'] >= .45),
            'drive_graph_valid': report.get('drive_graph_valid') is True}


def registered_run_sources(recorded):
    """실행이 기록한 소스가 r0 계약의 실행판 해시이고, 나머지는 현재 파일과 같은지."""
    contract = json.loads(CONTRACT.read_text(encoding='utf-8'))
    for path, change in contract['approved_evaluation_scripts'].items():
        if recorded.get(path) != change['after']:
            return False
    return all(sha(ROOT/path) == value for path, value in recorded.items()
               if path not in contract['approved_evaluation_scripts'])


def execution_passed(validity):
    """검사 결과만 결합한다. 누락 여부·간격 같은 설명값을 참/거짓 관문으로 쓰지 않는다."""
    checks = ('report_passed', 'input_valid', 'stopped', 'shutdown_clean', 'stop_acknowledged',
              'source_unchanged', 'profile_file_matches_report', 'run_audit', 'mode_observed',
              'sources_registered_run_version', 'posthoc_dynamics_coverage',
              'posthoc_final_stationary_coverage', 'drive_graph_valid', 'step_override_applied',
              'passive_mode', 'surface_official', 'extra_raw_hashes')
    return all(validity.get(k) is True for k in checks) and validity.get('final_reason') == 'software_stop'


def analyze_run(path):
    run, report, data, model, world = run_metrics(path)
    spec = report.get('command_schedule') or {}
    if spec.get('mode') != 'sim_grid':
        raise ValueError(f'{path} is not a sim_grid run')
    sampler = ProfileSampler(load_profile(ROOT/report['command_profile']['path']))
    dt_ns = round(run['declared_step_s']*1e9)
    # 격자 모드 실행기(첫 판)는 진단 coverage·최종 정지 coverage·요청 감사를 input_valid에 반영하지 못했다.
    # 원시 기록에서 같은 정의로 다시 계산해 유효성에 넣는다(report 원본은 그대로 둔다).
    recomputed = posthoc_validity(report, data, sampler)
    validity = {**{k: v for k, v in run['validity'].items() if k not in ('checks', 'source_current_or_approved')},
                'sources_registered_run_version': registered_run_sources(report['source_sha256']),
                **recomputed,
                'raw_and_source_checks': run['validity']['checks'], 'step_override_applied': step_matches(run),
                'passive_mode': run['mode'] == 'passive_damping' and report['zero_speed_mode'] == 'passive_damping',
                'surface_official': report['surface'] == 'official',
                'extra_raw_hashes': all(sha(path/(n+'.jsonl')) == report[n]['sha256'] for n in ('actuation_status', 'output_status'))}
    validity['passed'] = execution_passed(validity)
    return {'path': run['path'], 'step_s': run['declared_step_s'], 'phase_ns': spec['phase_ns'],
            'validity': validity, 'schedule': schedule_fidelity(path, report, sampler),
            'delivery': delivery(path, report, data, sampler, dt_ns),
            'metrics': run['metrics'], 'timing': run['timing'], 'observed_step': run['observed_step'],
            'diagnostic_gaps_active': run['diagnostic_gaps_active'], 'phase_transition': run['phase_transition'],
            'initial_pose': run['initial_pose'], 'input_sha256': run['input_sha256'],
            'source_sha256': report['source_sha256']}, model, world


def tolerance(kind, a, b):
    absolute, relative = TOLERANCES[kind]
    return max(absolute, relative*max(abs(a), abs(b)))


def compare_delivery(a, b):
    out, comparable = {}, True
    for quantity in ('speed', 'steering'):
        stages_a, stages_b = a['delivery'][quantity]['stages_rel_start_ns'], b['delivery'][quantity]['stages_rel_start_ns']
        for stage in ('request', 'request_observed', 'safe', 'mcu', 'output'):
            if stage == 'output' and quantity == 'steering':  # 출력 상태에는 조향각이 없다.
                continue
            key = f'{quantity}_{stage}'
            if (not stages_a.get(stage) or not stages_b.get(stage)
                    or len(stages_a[stage]) != len(stages_b[stage])):
                out[key] = {'max_abs_diff_ns': 'unknown', 'within': False}
                comparable = False
                continue
            diffs = [y-x for x, y in zip(stages_a[stage], stages_b[stage])]
            worst = max(abs(d) for d in diffs)
            within = worst <= DELIVERY_TOL_NS[stage]
            out[key] = {'max_abs_diff_ns': worst, 'diffs_ns': diffs, 'within': within}
            comparable &= within
        holds_a, holds_b = a['delivery'][quantity]['mcu_hold_ns'], b['delivery'][quantity]['mcu_hold_ns']
        if holds_a and holds_b and len(holds_a) == len(holds_b):
            worst = max(abs(y-x) for x, y in zip(holds_a, holds_b))
            out[f'{quantity}_mcu_hold'] = {'max_abs_diff_ns': worst, 'within': worst <= DELIVERY_TOL_NS['mcu_hold']}
            comparable &= out[f'{quantity}_mcu_hold']['within']
        else:
            out[f'{quantity}_mcu_hold'] = {'max_abs_diff_ns': 'unknown', 'within': False}
            comparable = False
    pa, pb = a['delivery']['plugin_final_zero'], b['delivery']['plugin_final_zero']
    if pa and pb:
        worst = abs(pb['first_zero_step_ns']-pa['first_zero_step_ns'])
        out['plugin_final_zero'] = {'diff_ns': pb['first_zero_step_ns']-pa['first_zero_step_ns'],
                                    'within': worst <= DELIVERY_TOL_NS['plugin_final_zero']}
        comparable &= out['plugin_final_zero']['within']
    else:
        out['plugin_final_zero'] = {'diff_ns': 'unknown', 'within': None}
    return out, comparable


def compare_metrics(a, b):
    out, within = {}, True
    for name, kind in PRIMARY_METRICS.items():
        x, y = a['metrics'][name], b['metrics'][name]
        if x is None or y is None:
            out[name] = {'values': [x, y], 'comparable': False}
            within = False
            continue
        tol = tolerance(kind, x, y)
        out[name] = {'values': [x, y], 'difference': y-x, 'tolerance': tol, 'within': abs(y-x) <= tol}
        within &= out[name]['within']
    secondary = {name: [a['metrics'][name], b['metrics'][name]] for name in SECONDARY_METRICS}
    return out, within, secondary


def compare(kind, a, b, models, worlds):
    delivery_diff, comparable = compare_delivery(a, b)
    metrics, within, secondary = compare_metrics(a, b)
    grid = {}
    for name in ('requested_commands', 'safe_commands'):
        pair = [[{**r, 't': r['sim_s']-s['start']} for r in rows(ROOT/x['path']/(name+'.jsonl'))]
                for x, s in ((a, {'start': a['delivery_start_s']}), (b, {'start': b['delivery_start_s']}))]
        for key in ('speed_mps', 'steering_rad'):
            grid[f'{name}_{key}'] = mismatched_samples(*pair, key)
    checks = {'both_valid': a['validity']['passed'] and b['validity']['passed'],
              'both_schedule': a['schedule']['passed'] and b['schedule']['passed'],
              'same_profile_and_spawn_inputs': a['input_sha256']['runtime_vehicle.sdf'] == b['input_sha256']['runtime_vehicle.sdf']
              and models[a['path']] == models[b['path']],
              'same_world_except_step': worlds[a['path']] == worlds[b['path']],
              'same_sources': a['source_sha256'] == b['source_sha256']}
    return {'kind': kind, 'a': a['path'], 'b': b['path'], 'checks': checks, 'valid': all(checks.values()),
            'delivery': delivery_diff, 'delivery_comparable': comparable,
            'grid_20ms_mismatches_informational': grid,
            'metrics': metrics, 'metrics_within_tolerance': within, 'secondary_metrics': secondary,
            'rtf': [a['timing'] and a['timing']['real_time_factor'], b['timing'] and b['timing']['real_time_factor']],
            'initial_pose_difference_m': math.hypot(a['initial_pose']['truth_x_m']-b['initial_pose']['truth_x_m'],
                                                    a['initial_pose']['truth_y_m']-b['initial_pose']['truth_y_m'])}


def identification(comparisons):
    """README 등록 규칙. 2회 반복은 분포가 아니다."""
    def find(kind, step=None, phase=None):
        return [c for c in comparisons if c['kind'] == kind and (step is None or c['step_s'] == step)
                and (phase is None or c['phase_ns'] == phase)]
    repeat = {}
    for c in find('repeat'):
        repeat[c['step_s']] = c['valid'] and c['delivery_comparable'] and c['metrics_within_tolerance']
    phase = {}
    for c in find('phase'):
        if not c['valid']:
            phase[c['step_s']] = 'invalid'
        elif not repeat.get(c['step_s']):
            phase[c['step_s']] = 'not_identifiable_reference_repeat_not_reproduced'
        else:
            phase[c['step_s']] = ('phase_sensitive_via_delivery_timing' if not c['metrics_within_tolerance']
                                  else 'no_phase_difference_beyond_tolerance')
    dt = {}
    for c in find('dt'):
        label = f"phase_{c['phase_ns']//1_000_000}ms_{c['label']}"
        if not c['valid']:
            dt[label] = 'invalid'
        elif not (repeat.get(.001) and repeat.get(.00025)):
            dt[label] = 'not_identifiable_repeat_not_reproduced_at_both_dt'
        elif not c['delivery_comparable']:
            dt[label] = 'not_identifiable_delivery_differs'
        else:
            dt[label] = ('dt_effect_including_plugin_discretization' if not c['metrics_within_tolerance']
                         else 'no_dt_difference_beyond_tolerance')
    return {'repeat_reproduced_two_runs': repeat, 'phase_effect': phase, 'dt_effect': dt,
            'step_only_effect_identifiable': False,
            'step_only_basis': '플러그인 적용 시각은 step 해상도로 다르게 양자화되고 PI·토크 응답 이산화도 함께 바뀐다. '
                               '20 ms 격자 표본 일치만으로 동일 입력을 주장하지 않는다.'}


def analyze(named, pairs):
    runs, models, worlds = {}, {}, {}
    for name, path in named.items():
        run, model, world = analyze_run(path)
        run['delivery_start_s'] = json.loads((path/'report.json').read_text())['command_start_sim_s']
        runs[name], models[run['path']], worlds[run['path']] = run, model, world
    comparisons = []
    for kind, a, b, label in pairs:
        item = compare(kind, runs[a], runs[b], models, worlds)
        item.update(step_s=runs[a]['step_s'] if runs[a]['step_s'] == runs[b]['step_s'] else None,
                    phase_ns=runs[a]['phase_ns'] if runs[a]['phase_ns'] == runs[b]['phase_ns'] else None,
                    label=label, names=[a, b])
        comparisons.append(item)
    return {'runs': runs, 'comparisons': comparisons, 'identification': identification(comparisons),
            'delivery_tolerance_ns': DELIVERY_TOL_NS, 'metric_tolerances': TOLERANCES,
            'contract_sha256': hashlib.sha256(CONTRACT.read_bytes()).hexdigest(), 'analysis_sha256': sha(Path(__file__)),
            'limits': ['반복은 조건당 2회(기준 위상)·1회(대조 위상)다. 분포나 통계적 안정성을 확정하지 않는다.',
                       '플러그인 중간 전환 적용 시각은 20 ms 진단 구간 경계만 알 수 있다. 마지막 0목표만 zero_steps로 step 단위 역산한다.',
                       '/clock 간격은 best-effort 구독 기준이며 발행 묶음과 수신 유실을 구분하지 않는다.',
                       'MCU·출력 시각은 각 노드가 읽은 sim 시계값이다. 노드 간 /clock 수신 순서 차이는 분리하지 않는다.',
                       'passive_damping은 평가 전용 저항 근사이며 실물 MD20A 회로가 아니다.']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True, help='name=path')
    parser.add_argument('--pair', action='append', default=[], help='kind:a:b[:label], kind in repeat/phase/dt')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    named = {}
    for item in args.run:
        name, path = item.split('=', 1)
        if name in named:
            raise ValueError(f'duplicate run name {name}')
        named[name] = Path(path).resolve()
    pairs = []
    for item in args.pair:
        kind, a, b, *label = item.split(':')
        if kind not in ('repeat', 'phase', 'dt') or a not in named or b not in named or a == b:
            raise ValueError(f'bad pair {item}')
        pairs.append((kind, a, b, label[0] if label else f'{a}_vs_{b}'))
    result = analyze(named, pairs)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({'validity': {k: v['validity']['passed'] for k, v in result['runs'].items()},
                      'schedule': {k: v['schedule']['passed'] for k, v in result['runs'].items()},
                      'comparisons': [{k: c[k] for k in ('kind', 'names', 'valid', 'delivery_comparable',
                                                         'metrics_within_tolerance')} for c in result['comparisons']],
                      'identification': result['identification']}, indent=2, ensure_ascii=False))
