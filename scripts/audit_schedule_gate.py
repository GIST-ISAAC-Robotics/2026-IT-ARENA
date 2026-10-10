#!/usr/bin/env python3
"""격자 실행기 관문(Gate A) 판정: 정상 기준 통과와 평가 전용 지연주입의 기대 실패를 구분하고,
그 실패가 report·종료코드·배치 중단(다음 사례 미실행)으로 전파됐는지 확인한다.

기대 실패는 '주입한 검사만 실패하고 STOP·정지·정상 종료·소스·다른 입력 유효성은 모두 통과'일 때만 인정한다.
판정은 실행 기록(report/status/디렉터리)을 읽기만 하며 어떤 결과도 고치지 않는다.
"""
import argparse
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def failing(checks):
    return sorted(k for k, v in (checks or {}).items() if v is not True)


def common_checks(report):
    """요청 감사 이외의 실행·입력 유효성. 기대 실패에서도 모두 참이어야 한다."""
    components = report.get('input_validity_components') or {}
    checks = {
        'stopped': report.get('stopped') is True,
        'shutdown_clean': report.get('shutdown_clean') is True,
        'stop_acknowledged': report.get('stop_acknowledged') is True,
        'final_reason_software_stop': (report.get('final_mcu') or {}).get('reason') == 'software_stop',
        'source_unchanged': report.get('source_unchanged') is True,
        'no_error': report.get('error') is None,
        'no_remaining_pids': not report.get('remaining_pids'),
        'dynamics_coverage': (report.get('dynamics_coverage') or {}).get('passed') is True,
        'final_stationary_coverage': report.get('final_stationary_coverage') is True,
        'drive_graph_valid': report.get('drive_graph_valid') is True,
        'components_recorded': bool(components),
        **{f'component_{k}': v is True for k, v in components.items()},
        'contact_reader_clean': (report.get('contact_reader_audit') or {}).get('handled_messages', 0) > 0
                                and not (report.get('contact_reader_audit') or {}).get('parse_errors')
                                and not (report.get('contact_reader_audit') or {}).get('handler_errors'),
    }
    if report.get('plugin_command_trace_requested'):
        checks['plugin_trace_valid'] = (report.get('plugin_command_trace') or {}).get('valid') is True
    return checks


def classify(report, rc):
    common = common_checks(report)
    request_fail = failing((report.get('request_audit') or {}).get('checks'))
    all_fail = failing((report.get('request_audit_all_ticks') or {}).get('checks'))
    injection = report.get('evaluation_fault_injection')
    base = {'rc': rc, 'report_passed': report.get('passed'), 'input_valid': report.get('input_valid'),
            'common_checks_failed': failing(common), 'request_audit_failed': request_fail,
            'all_ticks_failed': all_fail}
    if injection is None and report.get('evaluation_fault_injection_requested') is None:
        ok = (rc == 0 and report.get('passed') is True and report.get('input_valid') is True
              and not base['common_checks_failed'] and not request_fail and not all_fail
              and report.get('request_audit_all_ticks') is not None)
        return {**base, 'class': 'pass' if ok else 'unexpected_failure'}
    expected = (injection or {}).get('expected_failed_checks') or {}
    ok = (rc not in (0, None) and report.get('passed') is False and report.get('input_valid') is False
          and (injection or {}).get('applied') is True and not base['common_checks_failed']
          and request_fail == sorted(expected.get('request_audit', []))
          and all_fail == sorted(expected.get('request_audit_all_ticks', [])))
    return {**base, 'injection': injection, 'expected_failed_checks': expected,
            'class': 'expected_injected_failure' if ok else 'unexpected'}


def gate(status_path, repo=ROOT):
    status = json.loads(Path(status_path).read_text(encoding='utf-8'))
    plan = status['plan']
    base = (repo/plan['base']).resolve()
    cases, sequence_ok = [], True
    first_expected_failure = next((c['name'] for c in plan['cases'] if c['expect'] == 'expected_failure'), None)
    for entry in status['cases']:
        if not entry.get('started'):
            cases.append({'name': entry['name'], 'class': 'not_started', 'reason': entry.get('reason')})
            sequence_ok = False
            continue
        report_path = base/entry['name']/'report.json'
        report = json.loads(report_path.read_text(encoding='utf-8')) if report_path.is_file() else {}
        result = classify(report, entry.get('rc')) if report else {'class': 'report_missing', 'rc': entry.get('rc')}
        result.update(name=entry['name'], expect=entry['expect'], residual_after=entry.get('residual_after'))
        wanted = 'pass' if entry['expect'] == 'pass' else 'expected_injected_failure'
        sequence_ok &= result['class'] == wanted
        cases.append(result)
    started = [c['name'] for c in status['cases'] if c.get('started')]
    planned = [c['name'] for c in plan['cases']]
    later = planned[planned.index(first_expected_failure)+1:] if first_expected_failure else []
    checks = {
        'all_started_cases_as_expected': sequence_ok and bool(started),
        'stopped_at_first_expected_failure': first_expected_failure is not None
                                             and status.get('stopped_at') == first_expected_failure,
        'batch_exit_nonzero': status.get('exit_code') not in (0, None),
        'later_cases_not_started': status.get('not_started') == later
                                   and all(not (base/n).exists() and not (base/f'{n}.runner.log').exists() for n in later),
        'no_residual_after_any_case': all(not c.get('residual_after') for c in status['cases'] if c.get('started')),
    }
    return {'status': str(status_path), 'cases': cases, 'checks': checks, 'passed': all(checks.values()),
            'scope': 'Gate A: 실행기·배치의 실패 전파와 기대 실패 구분. 물리 결과의 재현성 판정이 아니다.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--status', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = gate(args.status)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps({'passed': result['passed'], 'checks': result['checks'],
                      'cases': [{k: c.get(k) for k in ('name', 'class', 'rc')} for c in result['cases']]},
                     ensure_ascii=False, indent=2))
