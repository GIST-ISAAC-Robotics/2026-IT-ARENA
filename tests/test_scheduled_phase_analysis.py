"""격자 발행 분석기의 사건 대응·플러그인 0목표 역산·전달 비교 규칙."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import analyze_scheduled_phase as analysis  # noqa: E402

MS = 1_000_000


def test_change_events_ignore_float32_roundtrip_and_baseline():
    series = [(0, 0.), (1, 0.), (2, .589300000001), (3, .5893), (4, 0.)]
    assert analysis.change_events(series) == [(2, 0., .589300000001), (4, .5893, 0.)]


def test_match_requires_same_count_and_values():
    expected = [(100, 0., .5), (300, .5, 0.)]
    assert analysis.match(expected, [(120, 0., .5), (320, .5, 0.)]) == [120, 320]
    assert analysis.match(expected, [(120, 0., .5)]) is None
    assert analysis.match(expected, [(120, 0., .5), (200, .5, .3), (320, .3, 0.)]) is None
    assert analysis.match(expected, [(120, 0., .4), (320, .4, 0.)]) is None


def diag(t_ms, target, zero_steps):
    return {'sim_time_s': t_ms/1000, 'applied_target_mps': target, 'zero_steps': zero_steps}


def test_plugin_final_zero_from_zero_step_counter():
    start = 0
    data = [diag(2240, .6, 500), diag(2260, .6, 500), diag(2280, 0., 513), diag(2300, 0., 533)]
    result = analysis.plugin_final_zero_ns(data, start, MS)
    # 2260~2280 사이 0목표 13 step: 첫 0 step은 2268 ms.
    assert result['first_zero_step_ns'] == 2268*MS and result['steps_in_bracket'] == 20
    quarter = [diag(2260, .6, 0), diag(2280, 0., 41)]
    assert analysis.plugin_final_zero_ns(quarter, start, MS//4)['first_zero_step_ns'] == 2270*MS
    # 0목표 step 수가 구간보다 많으면 역산하지 않는다(unknown).
    assert analysis.plugin_final_zero_ns([diag(2260, .6, 0), diag(2280, 0., 25)], start, MS) is None
    assert analysis.plugin_final_zero_ns([diag(2260, .6, 0)], start, MS) is None


def run(stages, holds, plugin):
    stages = {'request_observed': stages['request'], **stages}
    return {'delivery': {'speed': {'stages_rel_start_ns': stages, 'mcu_hold_ns': holds},
                         'steering': {'stages_rel_start_ns': stages, 'mcu_hold_ns': holds},
                         'plugin_final_zero': plugin}}


def test_delivery_comparison_rule():
    base = {'request': [0, 100*MS], 'safe': [10*MS, 110*MS], 'mcu': [10*MS, 110*MS], 'output': [11*MS, 111*MS]}
    a = run(base, [100*MS], {'first_zero_step_ns': 112*MS})
    same = run({**base, 'mcu': [10*MS+MS, 110*MS], 'output': [12*MS, 112*MS]}, [99*MS], {'first_zero_step_ns': 113*MS})
    _, ok = analysis.compare_delivery(a, same)
    assert ok
    late_safe = run({**base, 'safe': [30*MS, 110*MS]}, [100*MS], {'first_zero_step_ns': 112*MS})
    out, ok = analysis.compare_delivery(a, late_safe)
    assert not ok and out['speed_safe']['max_abs_diff_ns'] == 20*MS
    unknown = run({**base, 'mcu': None}, None, None)
    out, ok = analysis.compare_delivery(a, unknown)
    assert not ok and out['speed_mcu']['max_abs_diff_ns'] == 'unknown' and out['plugin_final_zero']['within'] is None


def test_delivery_requires_actual_publish_time_and_complete_event_sequences():
    base = {'request': [0, 100*MS], 'safe': [10*MS, 110*MS], 'mcu': [10*MS, 110*MS], 'output': [11*MS, 111*MS]}
    a = run(base, [100*MS], {'first_zero_step_ns': 112*MS})
    b = run({**base, 'request_observed': [MS, 100*MS]}, [100*MS], {'first_zero_step_ns': 112*MS})
    out, ok = analysis.compare_delivery(a, b)
    assert not ok and not out['speed_request_observed']['within']
    for bad in ([], [10*MS], None):
        b = run({**base, 'safe': bad}, [100*MS], {'first_zero_step_ns': 112*MS})
        out, ok = analysis.compare_delivery(a, b)
        assert not ok and out['speed_safe']['max_abs_diff_ns'] == 'unknown'
    b = run(base, [], {'first_zero_step_ns': 112*MS})
    assert not analysis.compare_delivery(a, b)[1]


def test_execution_validity_does_not_gate_on_descriptive_fields():
    checks = dict.fromkeys(('report_passed', 'input_valid', 'stopped', 'shutdown_clean', 'stop_acknowledged',
        'source_unchanged', 'profile_file_matches_report', 'run_audit', 'mode_observed',
        'sources_registered_run_version', 'posthoc_dynamics_coverage',
        'posthoc_final_stationary_coverage', 'drive_graph_valid', 'step_override_applied',
        'passive_mode', 'surface_official', 'extra_raw_hashes'), True)
    valid = {**checks, 'final_reason': 'software_stop', 'runner_validity_block_missing': False,
             'posthoc_dynamics_max_gap_s': .02}
    assert analysis.execution_passed(valid)
    for key in checks:
        assert not analysis.execution_passed({**valid, key: False}), key
        assert not analysis.execution_passed({k: v for k, v in valid.items() if k != key}), key
    assert not analysis.execution_passed({**valid, 'final_reason': 'timeout'})


def test_identification_never_claims_step_only():
    comparisons = [
        {'kind': 'repeat', 'step_s': .001, 'phase_ns': 10*MS, 'valid': True, 'delivery_comparable': True, 'metrics_within_tolerance': True},
        {'kind': 'repeat', 'step_s': .00025, 'phase_ns': 10*MS, 'valid': True, 'delivery_comparable': True, 'metrics_within_tolerance': True},
        {'kind': 'dt', 'step_s': None, 'phase_ns': 10*MS, 'label': 'a', 'valid': True, 'delivery_comparable': True, 'metrics_within_tolerance': False},
        {'kind': 'dt', 'step_s': None, 'phase_ns': 0, 'label': 'b', 'valid': True, 'delivery_comparable': False, 'metrics_within_tolerance': False},
        {'kind': 'phase', 'step_s': .001, 'phase_ns': None, 'valid': True, 'delivery_comparable': False, 'metrics_within_tolerance': False},
    ]
    result = analysis.identification(comparisons)
    assert result['step_only_effect_identifiable'] is False
    assert result['dt_effect'] == {'phase_10ms_a': 'dt_effect_including_plugin_discretization',
                                   'phase_0ms_b': 'not_identifiable_delivery_differs'}
    assert result['phase_effect'] == {.001: 'phase_sensitive_via_delivery_timing'}
    comparisons[0]['metrics_within_tolerance'] = False
    result = analysis.identification(comparisons)
    assert result['phase_effect'] == {.001: 'not_identifiable_reference_repeat_not_reproduced'}
    assert set(result['dt_effect'].values()) == {'not_identifiable_repeat_not_reproduced_at_both_dt'}
