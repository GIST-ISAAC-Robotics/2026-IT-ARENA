"""2026-10-10 마감 보강: Gate A 감사기 revision 2와 추적 분석 revision 2의 fail-closed 음성 회귀."""
import copy
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import analyze_apply_trace_v2 as trace_v2  # noqa: E402
import audit_schedule_gate_v2 as g2  # noqa: E402

MS = 1_000_000
PRESERVED = ROOT/'artifacts/validation/2026-10-09/command_apply_trace_v1'


def good():
    return {'passed': True, 'input_valid': True, 'stopped': True, 'shutdown_clean': True, 'stop_acknowledged': True,
            'source_unchanged': True, 'error': None, 'remaining_pids': {}, 'final_mcu': {'reason': 'software_stop'},
            'dynamics_coverage': {'passed': True}, 'final_stationary_coverage': True, 'drive_graph_valid': True,
            'input_validity_components': {k: True for k in g2.COMPONENTS},
            'contact_reader_audit': {'handled_messages': 3, 'parse_errors': 0, 'handler_errors': 0},
            'request_audit': {'checks': {k: True for k in g2.WINDOW_CHECKS}},
            'request_audit_all_ticks': {'checks': {k: True for k in g2.ALL_TICK_CHECKS}},
            'plugin_command_trace_requested': False, 'evaluation_fault_injection_requested': None,
            'command_schedule': {'start_ns': 2_010*MS}}


def injected(kind='late'):
    r = good()
    window, every = g2.FAULT_CONTRACT[kind]
    r.update(passed=False, input_valid=False)
    for name in window:
        r['request_audit']['checks'][name] = False
    for name in every:
        r['request_audit_all_ticks']['checks'][name] = False
    target = 2_010*MS+2_920*MS
    r['evaluation_fault_injection_requested'] = {'kind': kind, 'target_rel_ns': 2_920*MS,
                                                 'delay_ns': 15*MS if kind == 'late' else 25*MS}
    r['evaluation_fault_injection'] = {
        'kind': kind, 'target_scheduled_ns': target, 'delay_ns': r['evaluation_fault_injection_requested']['delay_ns'],
        'withheld_polls': 5, 'applied': True,
        'expected_failed_checks': {'request_audit': window, 'request_audit_all_ticks': every},
        'target_tick': {'scheduled_ns': target, 'observed_ns': target+15*MS, 'missed_before': 0} if kind == 'late' else None,
        'following_tick': None if kind == 'late' else {'scheduled_ns': target+20*MS, 'missed_before': 1}}
    return r


def drop(report, *path):
    out = copy.deepcopy(report)
    node = out
    for key in path[:-1]:
        node = node[key]
    del node[path[-1]]
    return out


def setv(report, value, *path):
    out = copy.deepcopy(report)
    node = out
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value
    return out


def test_positive_classes():
    assert g2.classify(good(), 0)['class'] == 'pass'
    assert g2.classify(injected('late'), 1)['class'] == 'expected_injected_failure'
    assert g2.classify(injected('miss'), 1)['class'] == 'expected_injected_failure'


@pytest.mark.parametrize('mutate', [
    lambda r: setv(r, {}, 'request_audit', 'checks'),
    lambda r: drop(r, 'request_audit'),
    lambda r: drop(r, 'request_audit', 'checks', 'no_missed_ticks'),
    lambda r: setv(r, {}, 'request_audit_all_ticks', 'checks'),
    lambda r: drop(r, 'request_audit_all_ticks'),
    lambda r: drop(r, 'input_validity_components', 'motor_shaft_feedback'),
    lambda r: setv(r, {}, 'input_validity_components'),
    lambda r: drop(r, 'contact_reader_audit', 'parse_errors'),
    lambda r: drop(r, 'contact_reader_audit', 'handler_errors'),
    lambda r: drop(r, 'remaining_pids'),
    lambda r: setv(r, {'1': ['gz']}, 'remaining_pids'),
    lambda r: setv(r, 'RuntimeError: boom', 'error'),
    lambda r: drop(r, 'final_mcu'),
    lambda r: drop(r, 'evaluation_fault_injection_requested'),
    lambda r: setv(r, 'yes', 'request_audit', 'checks', 'start_on_grid'),
])
def test_missing_evidence_never_passes(mutate):
    assert g2.classify(mutate(good()), 0)['class'] != 'pass'
    assert g2.classify(mutate(injected()), 1)['class'] != 'expected_injected_failure'


@pytest.mark.parametrize('mutate', [
    lambda r: setv(r, {'request_audit': [], 'request_audit_all_ticks': []},
                   'evaluation_fault_injection', 'expected_failed_checks'),
    lambda r: setv(r, 0, 'evaluation_fault_injection', 'withheld_polls'),
    lambda r: setv(r, 9*MS, 'evaluation_fault_injection', 'delay_ns'),
    lambda r: setv(r, 1, 'evaluation_fault_injection', 'target_scheduled_ns'),
    lambda r: setv(r, 'miss', 'evaluation_fault_injection', 'kind'),
    lambda r: setv(r, None, 'evaluation_fault_injection', 'target_tick'),
    lambda r: setv(r, 2_010*MS+2_920*MS+5*MS, 'evaluation_fault_injection', 'target_tick', 'observed_ns'),
    lambda r: setv(r, {k: True for k in g2.WINDOW_CHECKS}, 'request_audit', 'checks'),   # 지정 검사가 실패하지 않음
    lambda r: setv(r, False, 'request_audit', 'checks', 'no_missed_ticks'),              # 다른 검사도 실패
    lambda r: drop(r, 'evaluation_fault_injection'),
    lambda r: setv(r, 'other', 'evaluation_fault_injection_requested', 'kind'),
])
def test_injection_requires_contract_evidence(mutate):
    assert g2.classify(mutate(injected()), 1)['class'] != 'expected_injected_failure'


def test_injection_requires_nonzero_rc_and_failed_report():
    assert g2.classify(injected(), 0)['class'] != 'expected_injected_failure'
    assert g2.classify(injected(), None)['class'] != 'expected_injected_failure'
    assert g2.classify(setv(injected(), True, 'passed'), 1)['class'] != 'expected_injected_failure'


def make_status(tmp_path, entries, plan_cases, stopped_at, not_started, exit_code=1):
    base = tmp_path/'artifacts'/'b'
    base.mkdir(parents=True)
    plan = {'base': 'artifacts/b', 'cases': plan_cases}
    reports = {'pass': good(), 'inject': injected()}
    for entry in entries:
        (base/entry['name']).mkdir()
        (base/entry['name']/'report.json').write_text(json.dumps(reports[entry.pop('kind')]))
    status = {'plan': plan, 'cases': entries, 'stopped_at': stopped_at, 'not_started': not_started, 'exit_code': exit_code}
    (tmp_path/'plan.json').write_text(json.dumps(plan))
    (tmp_path/'status.json').write_text(json.dumps(status))
    return tmp_path/'status.json', tmp_path/'plan.json'


PLAN = [{'name': 'a1', 'expect': 'pass', 'args': ['x']}, {'name': 'a2', 'expect': 'expected_failure', 'args': ['y']},
        {'name': 'a3', 'expect': 'pass', 'args': ['x']}]


def entry(name, expect, args, rc, kind, residual=()):
    return {'name': name, 'expect': expect, 'args': args, 'started': True, 'rc': rc, 'residual_after': list(residual),
            'kind': kind}


def test_gate_v2_passes_correct_prefix(tmp_path):
    status, plan = make_status(tmp_path, [entry('a1', 'pass', ['x'], 0, 'pass'),
                                          entry('a2', 'expected_failure', ['y'], 1, 'inject')], PLAN, 'a2', ['a3'])
    assert g2.gate(status, plan, repo=tmp_path)['passed']


@pytest.mark.parametrize('entries,plan_cases,stopped,later', [
    ([entry('a2', 'expected_failure', ['y'], 1, 'inject')], PLAN, 'a2', ['a3']),                        # 기준 삭제
    ([entry('a1', 'pass', ['x'], 0, 'pass'), entry('a2', 'pass', ['y'], 1, 'inject')], PLAN, 'a2', ['a3']),  # expect 바꿔치기
    ([entry('a1', 'pass', ['x'], 0, 'pass'), entry('a9', 'expected_failure', ['y'], 1, 'inject')], PLAN, 'a9', ['a3']),
    ([entry('a1', 'pass', ['z'], 0, 'pass'), entry('a2', 'expected_failure', ['y'], 1, 'inject')], PLAN, 'a2', ['a3']),
    ([entry('a1', 'pass', ['x'], 0, 'pass'), entry('a2', 'expected_failure', ['y'], 1, 'inject')],
     [PLAN[1], PLAN[0], PLAN[2]], 'a2', ['a3']),                                                       # 계획 자체에 기준 없음
])
def test_gate_v2_rejects_tampered_sequence(tmp_path, entries, plan_cases, stopped, later):
    status, plan = make_status(tmp_path, entries, plan_cases, stopped, later)
    assert not g2.gate(status, plan, repo=tmp_path)['passed']


def test_gate_v2_requires_residual_evidence(tmp_path):
    first = entry('a1', 'pass', ['x'], 0, 'pass')
    del first['residual_after']
    status, plan = make_status(tmp_path, [first, entry('a2', 'expected_failure', ['y'], 1, 'inject')], PLAN, 'a2', ['a3'])
    assert not g2.gate(status, plan, repo=tmp_path)['passed']


def test_gate_v2_on_preserved_gate_a():
    result = g2.gate(PRESERVED/'status_gate_a_v2.json', PRESERVED/'plan_gate_a.json')
    assert result['passed'], result['checks']
    assert [c['class'] for c in result['cases']] == ['pass', 'expected_injected_failure']


# ---- 추적 분석 revision 2 -------------------------------------------------------------------

def test_registered_sources_requires_all_contract_sources():
    contract = json.loads((ROOT/'config/tests/scheduled_command_r2_sha256.json').read_text())
    required = trace_v2.required_sources(contract)
    assert trace_v2.registered_sources(dict(required), contract)
    for path in required:
        missing = {k: v for k, v in required.items() if k != path}
        assert not trace_v2.registered_sources(missing, contract), path
        assert not trace_v2.registered_sources({**required, path: '0'*64}, contract), path
    report = json.loads((PRESERVED/'c1_trace/report.json').read_text())
    assert trace_v2.registered_sources(report['source_sha256'], contract)


def trace(applies):
    return {c: [{'to': to, 'apply_rel_ns': t, 'steps_after_rx': 1} for to, t in applies.get(c, [])]
            for c in ('speed', 'steering')}


@pytest.mark.parametrize('a_time,b_time,expected', [(31*MS, 30*MS, 30*MS), (30*MS, 31*MS, 30*MS)])
def test_first_application_difference_uses_earlier_application(a_time, b_time, expected):
    a = trace({'speed': [(.1, 5*MS), (.2, a_time)]})
    b = trace({'speed': [(.1, 5*MS), (.2, b_time)]})
    result = trace_v2.first_application_difference(a, b)
    assert result['first_difference_rel_ns'] == expected
    # b가 먼저 적용하고 분기가 그 직후(30.5 ms)면 v2는 '이후 분기', a 시각만 쓰면 '선행'으로 잘못 분류될 수 있다.
    assert trace_v2.v1.classify(True, result, {'first_any_difference_rel_ns': 30_500_000}) == \
        'divergence_onset_not_before_first_application_difference'
