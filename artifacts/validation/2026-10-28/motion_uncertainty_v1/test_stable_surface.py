import copy
import json
from pathlib import Path
import numpy as np
import pytest

from stable_surface_candidate import stable_cycle, surface_translation, original_translation

CASE = json.loads((Path(__file__).resolve().parent / 'surface_convergence_case_v1.json').read_text())


def test_recorded_submillimeter_oscillation_is_bounded_with_geometry_unchanged():
    old = np.array(CASE['previous_now'])
    current = np.array(CASE['current'])
    ids = CASE['source_indices']
    original = original_translation(old, current, ids)
    assert original['reason'] == 'unsettled_or_high_residual' and not original['accepted']
    old_copy, current_copy = old.copy(), current.copy()
    result = surface_translation(old, current, ids)
    assert result['accepted'] and result['reason'] == 'bounded_surface_numerical_cycle'
    assert 7e-5 < result['numerical_cycle_diameter_m'] < 1e-4
    assert result['numerical_residual_spread_m'] < 1e-6
    assert result['steps'][-1]['residual_rms_m'] < .005
    np.testing.assert_array_equal(old, old_copy)
    np.testing.assert_array_equal(current, current_copy)


@pytest.mark.parametrize('fault', ['large_cycle', 'drift', 'unstable_cost', 'large_residual',
    'poor_support', 'unobservable', 'weak_large_shift', 'nonfinite', 'other_rejection'])
def test_unsettled_or_bad_geometry_is_not_accepted(fault):
    fit = copy.deepcopy(CASE['original_fit'])
    if fault == 'large_cycle':
        fit['steps'][-1]['step_m'][0] = .001
    if fault == 'drift':
        for step in fit['steps'][-3:]:
            step['step_m'] = [.00006, 0.]
    if fault == 'unstable_cost':
        fit['steps'][-1]['residual_rms_m'] += .0001
    if fault == 'large_residual':
        for step in fit['steps']:
            step['residual_rms_m'] = .02
    if fault == 'poor_support':
        fit['steps'][-1]['fraction'] = .64
    if fault == 'unobservable':
        fit['steps'][-1]['normal_eigen_ratio'] = .019
    if fault == 'weak_large_shift':
        fit['steps'][0]['step_m'] = [.005, 0.]
    if fault == 'nonfinite':
        fit['steps'][-1]['step_m'][0] = float('nan')
    if fault == 'other_rejection':
        fit['reason'] = 'localized_support'
    assert stable_cycle(fit) is None


def test_successful_original_fit_is_returned_without_new_decision_fields():
    current = np.array(CASE['current'])
    result = surface_translation(current, current, CASE['source_indices'])
    assert result['accepted']
    assert 'numerical_cycle_diameter_m' not in result
