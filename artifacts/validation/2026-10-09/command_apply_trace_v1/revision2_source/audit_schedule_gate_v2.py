#!/usr/bin/env python3
"""Gate A 감사기 revision 2(2026-10-10). v1(`audit_schedule_gate.py`, r2 계약에 동결)은 보존한다.

v1 대비 fail-closed 보강:
- r2 report 필수 키·타입을 검사한다. 감사 checks 누락/빈 dict/키 결손, 입력 유효성 구성 요소 결손,
  수신기 오류 수·잔존 PID·status의 residual_after 누락은 통과시키지 않는다.
- 기대 실패는 report의 자기기술이 아니라 아래 FAULT_CONTRACT로 판정한다. 요청 사양과 결과가 같고,
  실제 보류 poll과 목표 tick 지연(또는 누락) 근거가 있어야 한다.
- status 실행 목록은 plan의 첫 기대 실패까지 정확한 순서 prefix여야 하며 이름·expect·인자가 plan과 같아야 한다.
"""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WINDOW_CHECKS = ('start_on_grid', 'every_registered_tick_once', 'values_match_scheduled_sample', 'not_backdated',
                 'header_is_observed_clock', 'lateness_within_limit', 'no_missed_ticks')
ALL_TICK_CHECKS = ('all_ticks_contiguous_from_origin', 'all_ticks_not_backdated', 'all_ticks_lateness_within_limit',
                   'all_ticks_no_missed')
COMPONENTS = ('feedback_present', 'active_samples', 'final_samples', 'motor_shaft_feedback',
              'peak_body_speed_above_0p8', 'mode_observed')
# r2 fault 계약(scheduled_command.ScheduleFaultInjection)의 지정 실패 검사. report 자기기술과 별개로 고정한다.
FAULT_CONTRACT = {'late': (['lateness_within_limit'], ['all_ticks_lateness_within_limit']),
                  'miss': (['every_registered_tick_once', 'no_missed_ticks'],
                           ['all_ticks_contiguous_from_origin', 'all_ticks_no_missed'])}
PERIOD_NS = 20_000_000


def is_int(value):
    return type(value) is int


def checks_dict(audit, names):
    """정확한 키 집합과 bool 값일 때만 dict를 돌려준다. 아니면 None(증거 결손)."""
    checks = (audit or {}).get('checks') if isinstance(audit, dict) else None
    if not isinstance(checks, dict) or set(checks) != set(names) or not all(type(v) is bool for v in checks.values()):
        return None
    return checks


def evidence_checks(report):
    components = report.get('input_validity_components')
    reader = report.get('contact_reader_audit')
    coverage = report.get('dynamics_coverage')
    checks = {
        'stopped': report.get('stopped') is True,
        'shutdown_clean': report.get('shutdown_clean') is True,
        'stop_acknowledged': report.get('stop_acknowledged') is True,
        'final_reason_software_stop': isinstance(report.get('final_mcu'), dict)
                                      and report['final_mcu'].get('reason') == 'software_stop',
        'source_unchanged': report.get('source_unchanged') is True,
        # r2 probe는 예외가 있을 때만 error 키를 기록한다(성공 report에는 키가 없다).
        'no_error_recorded': report.get('error') is None,
        'remaining_pids_recorded_empty': isinstance(report.get('remaining_pids'), dict) and not report['remaining_pids'],
        'dynamics_coverage': isinstance(coverage, dict) and coverage.get('passed') is True,
        'final_stationary_coverage': report.get('final_stationary_coverage') is True,
        'drive_graph_valid': report.get('drive_graph_valid') is True,
        'components_complete': isinstance(components, dict) and set(components) == set(COMPONENTS)
                               and all(components[k] is True for k in COMPONENTS),
        'contact_reader_clean': isinstance(reader, dict) and is_int(reader.get('handled_messages'))
                                and reader['handled_messages'] > 0 and reader.get('parse_errors') == 0
                                and is_int(reader.get('parse_errors')) and reader.get('handler_errors') == 0
                                and is_int(reader.get('handler_errors')),
        'window_checks_complete': checks_dict(report.get('request_audit'), WINDOW_CHECKS) is not None,
        'all_tick_checks_complete': checks_dict(report.get('request_audit_all_ticks'), ALL_TICK_CHECKS) is not None,
    }
    if report.get('plugin_command_trace_requested') is not False:
        trace = report.get('plugin_command_trace')
        checks['plugin_trace_valid'] = (report.get('plugin_command_trace_requested') is True
                                        and isinstance(trace, dict) and trace.get('valid') is True)
    return checks


def failing(checks):
    return sorted(k for k, v in checks.items() if v is not True)


def injection_evidence(report):
    """요청 사양·결과·실제 주입 근거를 fault 계약과 대조한다."""
    spec, result = report.get('evaluation_fault_injection_requested'), report.get('evaluation_fault_injection')
    schedule = report.get('command_schedule') or {}
    if not isinstance(spec, dict) or not isinstance(result, dict) or spec.get('kind') not in FAULT_CONTRACT:
        return {'valid': False, 'reason': 'missing or unknown fault spec/result'}
    kind = spec['kind']
    window, every = FAULT_CONTRACT[kind]
    target = result.get('target_tick')
    following = result.get('following_tick')
    start = schedule.get('start_ns')
    checks = {
        'kind_matches': result.get('kind') == kind,
        'delay_matches': is_int(spec.get('delay_ns')) and result.get('delay_ns') == spec['delay_ns'],
        'target_matches': is_int(start) and is_int(spec.get('target_rel_ns'))
                          and result.get('target_scheduled_ns') == start+spec['target_rel_ns'],
        'self_description_matches_contract': result.get('expected_failed_checks') == {
            'request_audit': window, 'request_audit_all_ticks': every},
        'withheld_polls_positive': is_int(result.get('withheld_polls')) and result['withheld_polls'] > 0,
        'applied_flag': result.get('applied') is True,
    }
    if kind == 'late':
        checks['target_tick_late_by_delay'] = (isinstance(target, dict) and is_int(target.get('observed_ns'))
            and is_int(target.get('scheduled_ns')) and target['scheduled_ns'] == result.get('target_scheduled_ns')
            and target['observed_ns']-target['scheduled_ns'] >= spec['delay_ns'] and target.get('missed_before') == 0)
    else:
        checks['target_missing_and_next_missed'] = (target is None and isinstance(following, dict)
            and following.get('scheduled_ns') == result.get('target_scheduled_ns', 0)+PERIOD_NS
            and following.get('missed_before') == 1)
    return {'valid': all(checks.values()), 'checks': checks, 'kind': kind, 'contract_window': window,
            'contract_all_ticks': every}


def classify(report, rc):
    evidence = evidence_checks(report)
    window = checks_dict(report.get('request_audit'), WINDOW_CHECKS)
    every = checks_dict(report.get('request_audit_all_ticks'), ALL_TICK_CHECKS)
    window_fail = failing(window) if window is not None else None
    all_fail = failing(every) if every is not None else None
    base = {'rc': rc, 'report_passed': report.get('passed'), 'input_valid': report.get('input_valid'),
            'evidence_failed': failing(evidence), 'request_audit_failed': window_fail, 'all_ticks_failed': all_fail}
    if 'evaluation_fault_injection_requested' not in report:
        return {**base, 'class': 'unexpected', 'reason': 'fault request key missing'}
    if report['evaluation_fault_injection_requested'] is None and 'evaluation_fault_injection' not in report:
        ok = (rc == 0 and report.get('passed') is True and report.get('input_valid') is True
              and not base['evidence_failed'] and window_fail == [] and all_fail == [])
        return {**base, 'class': 'pass' if ok else 'unexpected_failure'}
    injection = injection_evidence(report)
    ok = (is_int(rc) and rc != 0 and report.get('passed') is False and report.get('input_valid') is False
          and injection['valid'] and not base['evidence_failed']
          and window_fail == sorted(injection['contract_window']) and all_fail == sorted(injection['contract_all_ticks'])
          and bool(window_fail) and bool(all_fail))
    return {**base, 'injection_evidence': injection, 'class': 'expected_injected_failure' if ok else 'unexpected'}


def gate(status_path, plan_path, repo=ROOT):
    status = json.loads(Path(status_path).read_text(encoding='utf-8'))
    plan = json.loads(Path(plan_path).read_text(encoding='utf-8'))
    base = (repo/plan['base']).resolve()
    planned = plan['cases']
    first = next((i for i, c in enumerate(planned) if c['expect'] == 'expected_failure'), None)
    expected_prefix = planned[:first+1] if first is not None else []
    cases = status.get('cases') if isinstance(status.get('cases'), list) else []
    prefix_ok = (first is not None and status.get('plan') == plan and len(cases) == len(expected_prefix)
                 and all(c.get('name') == p['name'] and c.get('expect') == p['expect'] and c.get('args') == p['args']
                         for c, p in zip(cases, expected_prefix)))
    results, sequence_ok = [], bool(cases)
    for entry, wanted in zip(cases, expected_prefix):
        residual = entry.get('residual_after')
        report_path = base/entry.get('name', '')/'report.json'
        if entry.get('started') is not True or not isinstance(residual, list) or not report_path.is_file():
            results.append({'name': entry.get('name'), 'class': 'missing_evidence'})
            sequence_ok = False
            continue
        result = classify(json.loads(report_path.read_text(encoding='utf-8')), entry.get('rc'))
        result.update(name=entry['name'], expect=wanted['expect'], residual_after=residual)
        sequence_ok &= result['class'] == ('pass' if wanted['expect'] == 'pass' else 'expected_injected_failure')
        results.append(result)
    later = [c['name'] for c in planned[first+1:]] if first is not None else []
    checks = {
        'status_is_plan_prefix_through_first_expected_failure': prefix_ok,
        'baseline_present_before_expected_failure': first is not None and first > 0
                                                    and all(c['expect'] == 'pass' for c in planned[:first]),
        'all_started_cases_as_expected': sequence_ok and prefix_ok,
        'stopped_at_first_expected_failure': first is not None and status.get('stopped_at') == planned[first]['name'],
        'batch_exit_nonzero': is_int(status.get('exit_code')) and status['exit_code'] != 0,
        'later_cases_not_started': status.get('not_started') == later
                                   and all(not (base/n).exists() and not (base/f'{n}.runner.log').exists() for n in later),
        'no_residual_after_any_case': bool(cases) and all(isinstance(c.get('residual_after'), list)
                                                          and not c['residual_after'] for c in cases),
    }
    return {'status': str(status_path), 'plan': str(plan_path), 'revision': 2, 'cases': results, 'checks': checks,
            'passed': all(checks.values()),
            'scope': 'Gate A 실패 전파·기대 실패 구분(fail-closed revision 2). 물리 재현성 판정이 아니다.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--status', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = gate(args.status, args.plan)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps({'passed': result['passed'], 'checks': result['checks'],
                      'cases': [{k: c.get(k) for k in ('name', 'class', 'rc')} for c in result['cases']]},
                     ensure_ascii=False, indent=2))
