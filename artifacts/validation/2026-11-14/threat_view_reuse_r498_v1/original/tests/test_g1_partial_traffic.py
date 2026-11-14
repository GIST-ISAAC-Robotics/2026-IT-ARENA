"""후보 교통 실행의 잘못된 조합·연결 누락을 실제 주행 전에 거절한다."""
import copy
import json
from pathlib import Path

import pytest

from g1_partial_trial import partial_trial_arguments, partial_trial_evidence, partial_trace_fields
from g1_partial_trial import parallel_trial_arguments, parallel_trial_evidence
from g1_partial_trial import pass_release_arguments,pass_release_evidence

ROOT = Path(__file__).resolve().parents[1]


def test_pass_release_requires_parallel_and_records_which_candidate_was_used():
    assert pass_release_arguments(False,False)==[]
    with pytest.raises(ValueError):pass_release_arguments(True,False)
    assert pass_release_arguments(True,True)==['g1_pass_release_trial:=true']
    selector={'pass_release_trial':True,'selector_mode':'SOLO','selector_choice':'baseline',
              'selector_unknown_filled':False}
    row=partial_trace_fields('bounded_local_v4',selector,{},parallel=True,pass_release=True)
    assert pass_release_evidence([row])['passed']
    assert pass_release_evidence([row])['baseline_rows']==1
    assert not pass_release_evidence([])['passed']
    row['pass_release_trial']['selector_unknown_filled']=True
    assert not pass_release_evidence([row])['passed']


def test_parallel_trial_default_and_validated_combination():
    assert parallel_trial_arguments(False, 'strict', 'r0', False, 'connected', 'oldest') == []
    assert parallel_trial_arguments(True, 'bounded_local_v4', 'g1', True,
        'corner_distance_candidate', 'per_ray_candidate') == ['g1_parallel_hold_trial:=true']


@pytest.mark.parametrize('index,value', [(0, 'strict'), (1, 'm3'), (2, False),
                                      (3, 'connected'), (4, 'oldest')])
def test_parallel_trial_rejects_unsupported_entrypoints(index, value):
    args = ['bounded_local_v4', 'g1', True, 'corner_distance_candidate', 'per_ray_candidate']
    args[index] = value
    with pytest.raises(ValueError): parallel_trial_arguments(True, *args)


def test_parallel_trial_evidence_requires_actual_selection_and_no_fill():
    row = {'parallel_hold_trial': {'unknown_filled': False}, 'parallel_hold_selected': True}
    assert parallel_trial_evidence([row])['passed']
    assert not parallel_trial_evidence([])['passed']
    assert not parallel_trial_evidence([row | {'parallel_hold_selected': False}])['passed']
    assert not parallel_trial_evidence([row | {'parallel_hold_trial': {'unknown_filled': True}}])['passed']


def launch():
    return json.loads((ROOT/'config/tests/g1_traffic_v1.json').read_text())['launch']


def test_strict_keeps_existing_launch_arguments():
    assert partial_trial_arguments('strict', 'r0', False, {}) == []


@pytest.mark.parametrize('policy', ['bounded_local_v2', 'bounded_local_v3', 'bounded_local_v4'])
def test_explicit_policy_preserves_configuration_and_forwards_only_its_choice(policy):
    settings = launch()
    original = copy.deepcopy(settings)
    assert partial_trial_arguments(policy, 'g1', True, settings) == [f'partial_observation_policy:={policy}']
    assert settings == original


@pytest.mark.parametrize('changes', [
    {'lidar_acquisition': 'instant'}, {'lidar_rate_hz': 7.}, {'lidar_compensation': 'none'},
    {'speed_profile': 'local_fast'}, {'actuation_mode': 'legacy'}, {'autonomy_mode': 'wall_follow'},
])
def test_unvalidated_motion_and_speed_combinations_are_refused(changes):
    with pytest.raises(ValueError):
        partial_trial_arguments('bounded_local_v4', 'g1', True, {**launch(), **changes})


@pytest.mark.parametrize('mode,recorded', [('r0', True), ('r0plus', True), ('m3', True), ('g1', False)])
def test_other_controllers_and_unrecorded_trials_are_refused(mode, recorded):
    with pytest.raises(ValueError):
        partial_trial_arguments('bounded_local_v4', mode, recorded, launch())


def evidence():
    return [{'partial_observation_trial': {'controller_policy': 'bounded_local_v4',
        'controller': {'unknown_samples': 0, 'unknown_filled': False},
        'safety': {'unknown_samples': 0, 'unknown_filled': False}}}]


def test_fully_observed_traffic_is_connection_regression_not_partial_gap_improvement():
    result = partial_trial_evidence('bounded_local_v4', evidence())
    assert result['passed'] and result['partial_diagnostic_rows'] == 0


def test_trace_preserves_both_independent_reports_and_strict_shape():
    controller = {'partial_observation_policy': 'bounded_local_v4',
                  'partial_observation': {'allowed': True, 'unknown_filled': False}}
    safety = {'partial_observation': {'allowed': False, 'reason': 'known_obstacle_latched',
                                    'unknown_filled': False}}
    assert partial_trace_fields('strict', controller, safety) == {}
    fields = partial_trace_fields('bounded_local_v4', controller, safety)['partial_observation_trial']
    assert fields['controller']['allowed'] and not fields['safety']['allowed']
    assert fields['safety']['reason'] == 'known_obstacle_latched'


@pytest.mark.parametrize('fault', ['empty', 'wrong_policy', 'missing_safety', 'filled', 'missing_fill_diagnostic'])
def test_unproved_connection_is_not_success(fault):
    rows = evidence()
    data = rows[0]['partial_observation_trial']
    if fault == 'empty':
        rows = []
    elif fault == 'wrong_policy':
        data['controller_policy'] = 'bounded_local_v3'
    elif fault == 'missing_safety':
        data['safety'] = None
    elif fault == 'filled':
        data['controller']['unknown_filled'] = True
    else:
        del data['safety']['unknown_filled']
    assert not partial_trial_evidence('bounded_local_v4', rows)['passed']


@pytest.mark.parametrize('mode,extra', [('r0', []), ('r0plus', []), ('m3', []),
                                    ('g1', []), ('g1', ['--record-follow-decisions', '--race-operation-test'])])
def test_cli_rejects_unsupported_partial_trial_before_loading_or_spawning(monkeypatch, tmp_path, mode, extra):
    import validate_g1_traffic as harness
    monkeypatch.setattr('sys.argv', ['x', '--case', 'stop_restart', '--seed', '0', '--mode', mode,
        '--output', str(tmp_path), '--partial-observation-policy', 'bounded_local_v4', *extra])
    monkeypatch.setattr(harness, 'load_test_profile', lambda *_: pytest.fail('must reject before loading'))
    with pytest.raises(SystemExit) as error:
        harness.main()
    assert error.value.code == 2


def test_cli_candidate_reaches_configuration_but_rejects_wrong_speed_before_world(monkeypatch, tmp_path):
    import validate_g1_traffic as harness
    monkeypatch.setattr('sys.argv', ['x', '--case', 'stop_restart', '--seed', '0', '--mode', 'g1',
        '--output', str(tmp_path), '--record-follow-decisions', '--partial-observation-policy', 'bounded_local_v4'])
    original = harness.load_config()
    original['launch']['speed_profile'] = 'local_fast'
    monkeypatch.setattr(harness, 'load_config', lambda _: original)
    monkeypatch.setattr(harness, 'load_centerline', lambda: pytest.fail('must reject before world access'))
    with pytest.raises(ValueError, match='g1_compare_1mps'):
        harness.main()


def test_snapshot_covers_new_policy_and_evaluation_dependencies():
    import validate_g1_traffic as harness
    paths = set(harness.source_paths(harness.CONFIG))
    assert {'scripts/g1_partial_trial.py', 'config/tests/partial_observation_r0_sha256.json',
        'src/arena_autonomy/arena_autonomy/partial_observation.py',
        'src/arena_autonomy/arena_autonomy/partial_surface_alignment.py',
        'src/arena_autonomy/arena_autonomy/pass_release_trial.py'} <= paths
