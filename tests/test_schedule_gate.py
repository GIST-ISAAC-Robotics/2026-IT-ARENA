"""격자 실행기 r2: 전체 tick 감사·평가 전용 지연주입·배치 중단·관문 분류의 순수 회귀(ROS/Gazebo 미실행)."""
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from braking_command_profile import load_profile  # noqa: E402
import audit_schedule_gate as gate_audit  # noqa: E402
import probe_braking_contact as probe  # noqa: E402
import run_probe_batch as batch  # noqa: E402
from scheduled_command import (  # noqa: E402
    GridSchedule, ProfileSampler, ScheduleFaultInjection, align_up, audit_all_ticks, audit_scheduled_requests)

MS = 1_000_000
PERIOD = 20*MS
REGISTERED = ROOT/'artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json'


@pytest.fixture(scope='module')
def sampler():
    return ProfileSampler(load_profile(REGISTERED))


def run_loop(sampler, phase_ns=10*MS, step_ns=MS, injection_spec=None, arm_ns=1_001_000_000, natural=None):
    """평가기 루프의 발행 경로를 순수 재현한다(실제 probe와 같은 withhold→poll 순서)."""
    origin = align_up(arm_ns, PERIOD, phase_ns)
    start = align_up(arm_ns+1_000*MS, PERIOD, phase_ns)
    schedule = GridSchedule(PERIOD, phase_ns, origin)
    injection = (ScheduleFaultInjection(injection_spec['kind'], start+injection_spec['target_rel_ns'],
                                        injection_spec['delay_ns']) if injection_spec else None)
    rows, now = [], arm_ns
    while now <= start+sampler.duration_ns+2_000*MS:
        tick = None if injection is not None and injection.withhold(now) else schedule.poll(now)
        if tick:
            speed, steering = sampler.at_ns(tick.scheduled_ns-start)
            rows.append(dict(speed_mps=speed, steering_rad=steering, scheduled_ns=tick.scheduled_ns, observed_ns=now,
                             header_ns=now, index=tick.index, lateness_ns=tick.lateness_ns,
                             missed_before=tick.missed_before))
        now += step_ns*(natural(now) if natural else 1)
    return rows, start, injection


def audits(rows, sampler, start, phase_ns=10*MS):
    window = audit_scheduled_requests(rows, sampler, start, start+sampler.duration_ns, PERIOD, phase_ns)
    return window, audit_all_ticks(rows, PERIOD)


def test_all_ticks_pass_and_hold_value_lateness_outside_window_fails(sampler):
    rows, start, _ = run_loop(sampler)
    window, every = audits(rows, sampler, start)
    assert window['passed'] and every['passed'] and every['ticks'] == len(rows)
    # 등록 창 뒤(4.26 s) 0 유지 tick 11 ms 지연: 창 감사는 통과하지만 전체 tick 감사는 실패해야 한다.
    late = next(i for i, r in enumerate(rows) if r['scheduled_ns'] == start+4_260*MS)
    rows[late] = {**rows[late], 'observed_ns': rows[late]['scheduled_ns']+11*MS, 'header_ns': rows[late]['scheduled_ns']+11*MS}
    window, every = audits(rows, sampler, start)
    assert window['passed'] and not every['passed']
    assert every['checks'] == {'all_ticks_contiguous_from_origin': True, 'all_ticks_not_backdated': True,
                               'all_ticks_lateness_within_limit': False, 'all_ticks_no_missed': True}
    assert every['late_ticks'][0]['lateness_ns'] == 11*MS


@pytest.mark.parametrize('mutation,check', [
    (lambda rows: rows[:5]+rows[6:], 'all_ticks_contiguous_from_origin'),
    (lambda rows: [*rows[:5], {**rows[5], 'missed_before': 1}, *rows[6:]], 'all_ticks_no_missed'),
    (lambda rows: [*rows[:5], {**rows[5], 'observed_ns': rows[5]['scheduled_ns']-1}, *rows[6:]], 'all_ticks_not_backdated'),
    (lambda rows: [*rows[:5], {**rows[5], 'observed_ns': rows[5]['scheduled_ns']+10*MS+1}, *rows[6:]],
     'all_ticks_lateness_within_limit'),
    (lambda rows: [], 'all_ticks_contiguous_from_origin'),
])
def test_all_ticks_negatives(sampler, mutation, check):
    rows, _, _ = run_loop(sampler)
    result = audit_all_ticks(mutation(rows), PERIOD)
    assert not result['passed'] and not result['checks'][check]


def test_all_ticks_limit_boundary_is_inclusive(sampler):
    rows, _, _ = run_loop(sampler)
    rows[7] = {**rows[7], 'observed_ns': rows[7]['scheduled_ns']+10*MS}
    assert audit_all_ticks(rows, PERIOD)['passed']


@pytest.mark.parametrize('kind,at_ms,delay_ms', [
    ('late', 2_920, 15), ('late', 3_000, 11), ('late', 3_000, 19), ('miss', 2_920, 25), ('miss', 3_000, 20),
])
def test_fault_validation_accepts_hold_ticks(sampler, kind, at_ms, delay_ms):
    rel, delay = ScheduleFaultInjection.validate(kind, at_ms, delay_ms, sampler, PERIOD)
    assert rel == at_ms*MS and delay == delay_ms*MS


@pytest.mark.parametrize('kind,at_ms,delay_ms', [
    ('late', 2_920, 10),    # 등록 한도와 같으면 실패를 만들지 못한다
    ('late', 2_920, 20),    # 한 주기 이상은 late가 아니다
    ('miss', 2_920, 19), ('miss', 2_920, 40),
    ('late', 2_930, 15),    # 격자 밖
    ('late', 0, 15), ('late', 3_480, 15), ('late', 4_260, 15),  # 등록 창 여유 밖
    ('late', 440, 15), ('late', 420, 15), ('late', 2_260, 15),   # 전환 tick 또는 전환 직전/직후
    ('miss', 2_240, 25),   # 보류 중 지나가는 tick에 전환이 있음
    ('other', 2_920, 15), ('late', 2_920.0, 15), ('late', 2_920, 15.),
])
def test_fault_validation_rejects(sampler, kind, at_ms, delay_ms):
    with pytest.raises(ValueError):
        ScheduleFaultInjection.validate(kind, at_ms, delay_ms, sampler, PERIOD)


@pytest.mark.parametrize('step_ns', [MS, MS//4])
def test_late_injection_fails_only_lateness_checks(sampler, step_ns):
    spec = {'kind': 'late', 'target_rel_ns': 2_920*MS, 'delay_ns': 15*MS}
    rows, start, injection = run_loop(sampler, step_ns=step_ns, injection_spec=spec)
    window, every = audits(rows, sampler, start)
    outcome = injection.outcome(rows, PERIOD)
    assert outcome['applied'] and outcome['withheld_polls'] > 0
    assert outcome['target_tick']['lateness_ns'] == 15*MS
    assert sorted(k for k, v in window['checks'].items() if not v) == outcome['expected_failed_checks']['request_audit']
    assert sorted(k for k, v in every['checks'].items() if not v) == \
        outcome['expected_failed_checks']['request_audit_all_ticks']
    # 값은 예정 시각 표본 그대로(유지값)이며 다른 tick 발행 시각은 바뀌지 않는다.
    clean, _, _ = run_loop(sampler, step_ns=step_ns)
    changed = [(a['scheduled_ns'], a['observed_ns'], b['observed_ns']) for a, b in zip(clean, rows)
               if a['observed_ns'] != b['observed_ns']]
    assert changed == [(start+2_920*MS, start+2_920*MS, start+2_935*MS)]
    assert [(r['speed_mps'], r['steering_rad']) for r in clean] == [(r['speed_mps'], r['steering_rad']) for r in rows]


def test_miss_injection_fails_registration_and_missed_checks(sampler):
    spec = {'kind': 'miss', 'target_rel_ns': 2_920*MS, 'delay_ns': 25*MS}
    rows, start, injection = run_loop(sampler, injection_spec=spec)
    window, every = audits(rows, sampler, start)
    outcome = injection.outcome(rows, PERIOD)
    assert outcome['applied'] and outcome['target_tick'] is None
    assert outcome['following_tick']['missed_before'] == 1 and outcome['following_tick']['lateness_ns'] == 5*MS
    assert sorted(k for k, v in window['checks'].items() if not v) == outcome['expected_failed_checks']['request_audit']
    assert sorted(k for k, v in every['checks'].items() if not v) == \
        outcome['expected_failed_checks']['request_audit_all_ticks']


def test_injection_not_applied_is_reported(sampler):
    # 시계가 목표+지연을 한 번에 넘으면 보류는 없었고 applied=false로 남는다(자연 지연과 구분).
    injection = ScheduleFaultInjection('late', 3_000*MS+2_920*MS, 15*MS)
    assert not injection.withhold(3_000*MS+2_919*MS)
    assert not injection.withhold(3_000*MS+2_936*MS)
    assert injection.released_ns == 3_000*MS+2_936*MS and injection.withheld_polls == 0
    assert not injection.withhold(3_000*MS+2_921*MS)  # 한 번 해제되면 다시 작동하지 않는다
    rows = [dict(scheduled_ns=3_000*MS+2_920*MS, observed_ns=3_000*MS+2_936*MS, missed_before=0)]
    assert not injection.outcome(rows, PERIOD)['applied']


def test_fault_and_trace_arguments_guard_default_path(sampler):
    grid = {'mode': 'sim_grid', 'period_ns': PERIOD, 'phase_ns': 10*MS}
    assert probe.fault_arguments(None, None, None, None, None, 'any') is None
    assert probe.fault_arguments(None, None, None, grid, sampler, 'any') is None
    assert probe.trace_arguments(False, None) == [] and probe.trace_arguments(False, grid) == []
    assert probe.trace_arguments(True, grid) == ['evaluation_command_trace:=true']
    assert probe.fault_arguments('late', 2_920, 15, grid, sampler, 'a1_inject_late') == \
        {'kind': 'late', 'target_rel_ns': 2_920*MS, 'delay_ns': 15*MS}
    for bad in ((None, 2_920, None, grid, sampler, 'inject'), (None, None, 15, grid, sampler, 'inject'),
                ('late', 2_920, 15, None, sampler, 'inject'), ('late', 2_920, None, grid, sampler, 'inject'),
                ('late', 2_920, 15, grid, sampler, 'a1_late')):
        with pytest.raises(ValueError):
            probe.fault_arguments(*bad)
    with pytest.raises(ValueError):
        probe.trace_arguments(True, None)
    # 기본 실행 인자는 이전과 같다.
    assert probe.evaluation_extra() == []


def trace_event(seq, kind='apply', **extra):
    return {'type': kind, 'event_seq': seq, **extra}


def test_command_trace_audit():
    clean = dict(json_messages=5, handled_messages=5, parse_errors=0, handler_errors=0, last_error=None)
    events = [trace_event(1, 'first_update'), trace_event(2, channel='speed'), trace_event(3, channel='steering'),
              trace_event(4, 'summary')]
    assert probe.command_trace_audit(events, clean)['valid']
    for broken in (events[1:], [events[0], events[2], events[3]], [*events[:3]], [events[0], events[1], events[3]],
                   [events[0], {**events[1], 'event_seq': 3}, {**events[2], 'event_seq': 2}, events[3]]):
        assert not probe.command_trace_audit(broken, clean)['valid']
    assert not probe.command_trace_audit(events, {**clean, 'parse_errors': 1})['valid']
    assert not probe.command_trace_audit(events, {**clean, 'handled_messages': 0})['valid']


def validity_block():
    tree = ast.parse((ROOT/'scripts/probe_braking_contact.py').read_text(encoding='utf-8'))
    return next(node for node in ast.walk(tree) if isinstance(node, ast.If) and isinstance(node.test, ast.Name)
                and node.test.id == 'profile' and any(isinstance(b, ast.If) and isinstance(b.test, ast.Name)
                and b.test.id == 'grid_spec' for b in node.body))


@pytest.mark.parametrize('failure,traced', [(None, False), ('window', False), ('all_ticks', False), (None, True),
                                            ('window', True), ('all_ticks', True), ('trace', True)])
def test_grid_validity_block_includes_all_ticks_and_trace(failure, traced):
    """실제 main의 블록을 실행해 r2 감사·추적 유효성이 input_valid로 전파되는지 확인한다."""
    scope = dict(profile=SimpleNamespace(duration_s=.5), sampler=SimpleNamespace(duration_ns=500*MS),
                 grid_spec={'period_ns': PERIOD, 'phase_ns': 0}, start=0., start_ns=0, stop_elapsed=1.5,
                 requests=[], clock_values=[], plugin_events=[], trace_audit={},
                 trace_extra=['evaluation_command_trace:=true'] if traced else [],
                 active=[{'sim_time_s': i*.02} for i in range(75)], final=[{'sim_time_s': 2.+i*.02} for i in range(26)],
                 report={'input_valid': True, 'drive_graph_valid': True, 'command_schedule': {'origin_ns': 0},
                         'finish_sim_s': 2.5},
                 audit_scheduled_requests=lambda *a: {'passed': failure != 'window'},
                 audit_all_ticks=lambda *a: {'passed': failure != 'all_ticks'},
                 command_trace_audit=lambda *a: {'valid': failure != 'trace'},
                 audit_requests=lambda *a: {'passed': True}, clock_summary=lambda *a: {})
    exec(compile(ast.Module(body=[validity_block()], type_ignores=[]), '<grid-validity>', 'exec'), scope)
    assert scope['report']['input_valid'] is (failure is None)
    assert ('plugin_command_trace' in scope['report']) is traced


def test_passed_requires_input_valid_and_exit_code_follows_passed():
    """report.passed 결합식과 반환 코드 연결을 소스에서 확인한다(실패 주입이 passed로 새지 않음)."""
    text = (ROOT/'scripts/probe_braking_contact.py').read_text(encoding='utf-8')
    tree = ast.parse(text)
    passed = [n for n in ast.walk(tree) if isinstance(n, ast.Assign) and any(
        isinstance(t, ast.Subscript) and isinstance(t.value, ast.Name) and t.value.id == 'report'
        and isinstance(t.slice, ast.Constant) and t.slice.value == 'passed' for t in n.targets)]
    assert len(passed) == 1
    expression = ast.unparse(passed[0].value)
    for needed in ("report['input_valid']", "report['stopped']", "report['shutdown_clean']", "report['source_unchanged']",
                   "'software_stop'"):
        assert needed in expression
    assert "return 0 if report['passed'] else 1" in text
    # 주입 결과는 report를 고치지 않는 읽기 전용 outcome이다.
    assert "report['evaluation_fault_injection'] = injection.outcome(requests" in text
    assert 'passed' not in ast.unparse([n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                                        and n.name == 'fault_arguments'][0])


# ---- 배치 실행기와 관문 분류 -------------------------------------------------------------

STUB = r'''
import json, sys
from pathlib import Path
args = sys.argv[1:]
out = Path(args[args.index('--output')+1]); rc = int(args[args.index('--rc')+1])
kind = args[args.index('--kind')+1] if '--kind' in args else 'pass'
out.mkdir(parents=True)
report = json.loads(Path(args[args.index('--template')+1]).read_text())[kind]
(out/'report.json').write_text(json.dumps(report))
sys.exit(rc)
'''


def good_report(**changes):
    report = {'passed': True, 'input_valid': True, 'stopped': True, 'shutdown_clean': True, 'stop_acknowledged': True,
              'source_unchanged': True, 'error': None, 'remaining_pids': {}, 'final_mcu': {'reason': 'software_stop'},
              'dynamics_coverage': {'passed': True}, 'final_stationary_coverage': True, 'drive_graph_valid': True,
              'input_validity_components': {'feedback_present': True, 'mode_observed': True},
              'contact_reader_audit': {'handled_messages': 3, 'parse_errors': 0, 'handler_errors': 0},
              'request_audit': {'checks': {'lateness_within_limit': True, 'no_missed_ticks': True}},
              'request_audit_all_ticks': {'checks': {'all_ticks_lateness_within_limit': True, 'all_ticks_no_missed': True}},
              'evaluation_fault_injection_requested': None}
    report.update(changes)
    return report


def injected_report(**changes):
    report = good_report(passed=False, input_valid=False,
                         request_audit={'checks': {'lateness_within_limit': False, 'no_missed_ticks': True}},
                         request_audit_all_ticks={'checks': {'all_ticks_lateness_within_limit': False,
                                                             'all_ticks_no_missed': True}},
                         evaluation_fault_injection_requested={'kind': 'late'},
                         evaluation_fault_injection={'applied': True, 'expected_failed_checks': {
                             'request_audit': ['lateness_within_limit'],
                             'request_audit_all_ticks': ['all_ticks_lateness_within_limit']}})
    report.update(changes)
    return report


def test_classify_cases():
    assert gate_audit.classify(good_report(), 0)['class'] == 'pass'
    assert gate_audit.classify(good_report(), 1)['class'] == 'unexpected_failure'
    assert gate_audit.classify(good_report(request_audit_all_ticks=None), 0)['class'] == 'unexpected_failure'
    assert gate_audit.classify(injected_report(), 1)['class'] == 'expected_injected_failure'
    assert gate_audit.classify(injected_report(), 0)['class'] == 'unexpected'           # 종료코드가 실패를 숨김
    assert gate_audit.classify(injected_report(passed=True), 1)['class'] == 'unexpected'
    assert gate_audit.classify(injected_report(stopped=False), 1)['class'] == 'unexpected'  # 다른 실패 동반
    assert gate_audit.classify(injected_report(shutdown_clean=False), 1)['class'] == 'unexpected'
    assert gate_audit.classify(injected_report(evaluation_fault_injection={
        'applied': False, 'expected_failed_checks': {}}), 1)['class'] == 'unexpected'
    extra = injected_report(request_audit={'checks': {'lateness_within_limit': False, 'no_missed_ticks': False}})
    assert gate_audit.classify(extra, 1)['class'] == 'unexpected'
    lost = injected_report(evaluation_fault_injection=None)  # 요청했는데 결과 기록이 없으면 통과로 보지 않는다
    assert gate_audit.classify(lost, 1)['class'] == 'unexpected'
    traced = good_report(plugin_command_trace_requested=True, plugin_command_trace={'valid': False})
    assert gate_audit.classify(traced, 0)['class'] == 'unexpected_failure'


def make_plan(tmp_path, cases):
    base = tmp_path/'artifacts'/'batch'
    base.mkdir(parents=True)
    template = tmp_path/'template.json'
    template.write_text(json.dumps({'pass': good_report(), 'inject': injected_report()}))
    stub = tmp_path/'stub.py'
    stub.write_text(STUB)
    plan = {'base': str(base.relative_to(tmp_path)), 'cases': [
        {'name': name, 'expect': expect, 'args': ['--rc', str(rc), '--kind', kind, '--template', str(template)]}
        for name, expect, rc, kind in cases]}
    return plan, base, [sys.executable, str(stub)]


def test_batch_stops_at_expected_failure_and_gate_passes(tmp_path):
    plan, base, command = make_plan(tmp_path, [('a1_base', 'pass', 0, 'pass'), ('a2_inject', 'expected_failure', 1, 'inject'),
                                               ('a3_sentinel', 'pass', 0, 'pass')])
    status = tmp_path/'status.json'
    assert batch.run_plan(plan, status, command=command, residual=lambda: [], repo=tmp_path) == 1
    data = json.loads(status.read_text())
    assert data['stopped_at'] == 'a2_inject' and data['not_started'] == ['a3_sentinel']
    assert not (base/'a3_sentinel').exists() and not (base/'a3_sentinel.runner.log').exists()
    result = gate_audit.gate(status, repo=tmp_path)
    assert result['passed'], result
    assert [c['class'] for c in result['cases']] == ['pass', 'expected_injected_failure']


def test_gate_fails_when_batch_ignores_failure(tmp_path):
    # 실패를 종료코드 0으로 숨긴 실행(r0 결함과 같은 형태) → 배치가 다음 사례로 진행 → 관문 실패.
    plan, base, command = make_plan(tmp_path, [('a1_base', 'pass', 0, 'pass'), ('a2_inject', 'expected_failure', 0, 'inject'),
                                               ('a3_sentinel', 'pass', 0, 'pass')])
    status = tmp_path/'status.json'
    assert batch.run_plan(plan, status, command=command, residual=lambda: [], repo=tmp_path) == 0
    assert (base/'a3_sentinel').exists()
    result = gate_audit.gate(status, repo=tmp_path)
    assert not result['passed']
    assert not result['checks']['stopped_at_first_expected_failure'] and not result['checks']['later_cases_not_started']
    assert result['cases'][1]['class'] == 'unexpected'


def test_batch_refuses_residual_and_stops_on_residual_after(tmp_path):
    plan, base, command = make_plan(tmp_path, [('a1_base', 'pass', 0, 'pass'), ('a2_more', 'pass', 0, 'pass')])
    status = tmp_path/'status.json'
    assert batch.run_plan(plan, status, command=command, residual=lambda: ['123 gz sim -s'], repo=tmp_path) == 95
    data = json.loads(status.read_text())
    assert data['cases'][0]['started'] is False and not (base/'a1_base').exists() and data['not_started'] == ['a2_more']
    calls = iter([[], ['77 parameter_bridge']])
    status2 = tmp_path/'status2.json'
    plan2, base2, command2 = make_plan(tmp_path/'second', [('b1', 'pass', 0, 'pass'), ('b2', 'pass', 0, 'pass')])
    assert batch.run_plan(plan2, status2, command=command2, residual=lambda: next(calls), repo=tmp_path/'second') == 95
    data = json.loads(status2.read_text())
    assert data['stopped_at'] == 'b1' and data['cases'][0]['residual_after'] == ['77 parameter_bridge']


def test_batch_never_reuses_output(tmp_path):
    plan, base, command = make_plan(tmp_path, [('a1_base', 'pass', 0, 'pass')])
    (base/'a1_base').mkdir()
    status = tmp_path/'status.json'
    assert batch.run_plan(plan, status, command=command, residual=lambda: [], repo=tmp_path) == 2
    assert json.loads(status.read_text())['cases'][0]['reason'] == 'output_exists'


@pytest.mark.parametrize('cases', [
    [{'name': 'a', 'expect': 'pass', 'args': ['--output', 'y']}],
    [{'name': 'a', 'expect': 'pass', 'args': []}, {'name': 'a', 'expect': 'pass', 'args': []}],
    [{'name': 'a/b', 'expect': 'pass', 'args': []}],
    [{'name': 'a', 'expect': 'maybe', 'args': []}],
    [],
])
def test_plan_rejects_bad_cases(tmp_path, cases):
    bad = tmp_path/'bad.json'
    bad.write_text(json.dumps({'base': 'x', 'cases': cases}))
    with pytest.raises(ValueError):
        batch.load_plan(bad)
