"""81T 감속 기록의 대응 잔차와 사라진 독립 장애물을 구별한다."""
import json
from pathlib import Path
import numpy as np
import pytest
from arena_autonomy.partial_surface_alignment import surface_translation
from arena_autonomy.partial_observation import PartialObservationPolicy
from test_partial_surface_alignment import room, observed_room

FIXTURE = Path(__file__).resolve().parents[1]/'artifacts/validation/2026-11-11/longitudinal_surface_r75_v1/sensor_fixture.json'


@pytest.mark.parametrize('mirror', [1., -1.])
def test_recorded_residual_matches_current_wall_without_truth_or_point_changes(mirror):
    data = json.loads(FIXTURE.read_text(encoding='utf-8'))
    c = data['context']; signs = np.array([1., mirror])
    old = np.array(c['previous_points_now_xy'])*signs
    current = np.array(c['current_return_points_xy'])*signs
    preserved = old.copy(), current.copy()
    fit = surface_translation(old, current, c['current_return_source_indices'])
    assert fit['accepted'] and fit['longitudinal_residual'], fit
    assert .06 < abs(fit['translation_m'][0]) < .08
    assert abs(fit['translation_m'][1]) < .01
    for witness in data['witnesses']:
        corrected = np.array(witness['previous_point_now_xy'])*signs+fit['translation_m']
        assert np.min(np.linalg.norm(current-corrected, axis=1)) < .02
    np.testing.assert_array_equal(old, preserved[0])
    np.testing.assert_array_equal(current, preserved[1])


@pytest.mark.parametrize('shift', [[.07,.005],[-.07,-.005]])
def test_longitudinal_residual_on_multi_direction_observed_room(shift):
    current, ids = room()
    fit = surface_translation(current-shift, current, ids)
    assert fit['accepted'] and fit['longitudinal_residual'], fit
    np.testing.assert_allclose(fit['translation_m'], shift, atol=1e-5)


@pytest.mark.parametrize('shift', [[.09,0.],[-.09,0.],[0.,.07],[.055,.055],[.065,.025]])
def test_large_lateral_or_diagonal_residual_remains_rejected(shift):
    current, ids = room()
    assert not surface_translation(current-shift, current, ids)['accepted']


def test_longitudinal_correspondence_does_not_erase_disappeared_independent_object():
    raw, current, h = observed_room()
    policy = PartialObservationPolicy(True, True)
    policy._remember(np.vstack([current-np.array([.07,.005]), [.06,.15]]), .5, .4, True)
    result = policy.evaluate(raw, current, h, .6, 1., .37, .37)
    assert result['surface_correspondence']['accepted'], result['surface_correspondence']
    assert result['reason']=='known_obstacle_latched'
    assert result['latch_evidence']['pairs']>0
