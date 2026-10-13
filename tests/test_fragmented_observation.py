import numpy as np
import pytest

from validate_fragmented_observation import cases, fixed_ranges, perturb_ranges, run_case


def test_nearest_blocker_replaces_front_wall_and_no_truth_is_in_input():
    empty = fixed_ranges('empty')
    blocked = fixed_ranges('blocked')
    assert empty.shape == blocked.shape == (500,)
    assert empty[250] > 5.
    assert blocked[250] == pytest.approx(.10)


def test_noise_preserves_missing_and_does_not_clip_short_returns():
    source = np.array([np.nan, np.inf, .05, 1.])
    result = perturb_ranges(source, np.random.default_rng(0), bias_m=-.03)
    assert np.isnan(result[0]) and np.isinf(result[1])
    assert result[2:] == pytest.approx([.02, .97])
    assert source[2:] == pytest.approx([.05, 1.])


def test_front_drop_is_unknown_not_long_range():
    result = perturb_ranges(fixed_ranges(), np.random.default_rng(0), missing_front=True)
    assert np.isnan(result[250])


@pytest.mark.parametrize('fault', ['front_unknown', 'new_blocker', 'stale', 'negative_feedback', 'frozen'])
def test_fixed_scene_faults_do_not_produce_forward_command(fault):
    result = run_case(dict(name=fault, fault=fault, expected='hold'))
    assert result['passed'], result['rows']


def test_exploratory_noise_does_not_claim_successful_driving():
    result = run_case(dict(name='noise', noise_m=.03, expected='observe'))
    assert not result['behavior_asserted']
    assert 'expected_behavior' not in result['checks']


def test_case_names_are_unique_and_bounds_are_explicit():
    suite = cases()
    assert len({case['name'] for case in suite}) == len(suite) == 35
    assert sum(case.get('expected') == 'observe' for case in suite) == 12
