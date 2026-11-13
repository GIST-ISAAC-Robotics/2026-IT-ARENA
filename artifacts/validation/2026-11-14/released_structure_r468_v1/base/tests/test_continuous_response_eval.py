"""기존 실패 보존과 우회 대안의 실제 진행/노면 조건을 확인한다."""
import copy
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location('continuous_response_eval', ROOT/
    'artifacts/validation/2026-11-02/official_forward_rejoin_v2/continuous_response_eval.py')
evaluator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(evaluator)


def fixture():
    report = dict(passed=False,run_valid=True,stop_checks=dict(explicit_mcu_stop=True),evaluation=dict(t_green=10.,gates=dict(
        hold=False,inside_road=False,no_actor_contact=True,sampled_gap=True,
        final_gap=True,curve_exit_reached=True)))
    trace = [dict(t=13.+i*.1,progress_m=i*.035,safe_speed_mps=.35,
                  autonomy=dict(behavior='BYPASS')) for i in range(21)]
    surface = dict(samples=1000,sample_gaps_over_limit=0,wall_wheel_samples=0,
                   interpretations={k:dict(max_off_wheels=2) for k in ('any','center','full')})
    return report,trace,surface


def test_successful_bypass_is_separate_from_unchanged_old_verdict():
    report,trace,surface = fixture()
    before = copy.deepcopy(report)
    result = evaluator.evaluate(report,trace,surface)
    assert result['supplementary_passed']
    assert result['original_passed'] is False and report == before


def test_stationary_bypass_label_does_not_replace_hold():
    report,trace,surface = fixture()
    for row in trace:
        row['progress_m'] = 0.
    assert not evaluator.evaluate(report,trace,surface)['supplementary_passed']


def test_missing_trace_does_not_replace_hold():
    report,trace,surface = fixture()
    assert not evaluator.evaluate(report,trace[::5],surface)['supplementary_passed']


def test_contact_or_unfinished_rejoin_stays_failed():
    for key in ('no_actor_contact','sampled_gap','final_gap','curve_exit_reached'):
        report,trace,surface = fixture()
        report['evaluation']['gates'][key] = False
        assert not evaluator.evaluate(report,trace,surface)['supplementary_passed']


def test_three_wheels_or_missing_surface_evidence_stays_failed():
    report,trace,surface = fixture()
    surface['interpretations']['any']['max_off_wheels'] = 3
    assert not evaluator.evaluate(report,trace,surface)['supplementary_passed']
    surface['interpretations']['any']['max_off_wheels'] = 2
    surface['sample_gaps_over_limit'] = 1
    assert not evaluator.evaluate(report,trace,surface)['supplementary_passed']


def test_stop_failure_cannot_be_hidden_by_functional_gates():
    report,trace,surface = fixture()
    report["stop_checks"]["explicit_mcu_stop"] = False
    assert not evaluator.evaluate(report,trace,surface)["supplementary_passed"]
