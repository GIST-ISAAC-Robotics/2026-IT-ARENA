from copy import deepcopy
import importlib.util
from pathlib import Path

import pytest

from rebase import rebase, supported

_spec = importlib.util.spec_from_file_location('r126_fixture', Path(__file__).resolve().parent.parent/'follow_rebase_r126_v1/test_rebase.py')
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)


def example(mode='GUARD', ranges=None):
    output, *rest = _module.example(ranges)
    normal = output[2]['normal']
    normal.update(reason='parallel_hold_follow', selector_choice='parallel', selector_mode=mode,
                  parallel_hold_selected=True, partial_observation={'allowed': True, 'reason': 'fully_observed'})
    normal['parallel_hold_trial'] = dict(eligible=True, reason='parallel_current_position',
        straight_trial_only=True, path_points=deepcopy(normal['path_points']))
    return output, *rest


@pytest.mark.parametrize('mode', ['GUARD', 'SOLO'])
def test_selected_parallel_path_can_be_rebased_without_speed_increase_or_recentering(mode):
    args = example(mode)
    before = deepcopy(args[0])
    result = rebase(*args)
    assert 0 < result['speed_mps'] <= args[0][0]
    assert result['maximum_body_corner_lateral_m'] <= .01
    assert result['rebased_selector_mode'] == mode
    assert not result['lane_recentering']
    assert result['path_points'][0][0] == pytest.approx(-.12)
    assert result['path_source_s'] == .3 and result['reference_s'] == .5
    assert args[0] == before


@pytest.mark.parametrize('change', ['path', 'partial', 'departure', 'expired', 'request', 'motion_epoch'])
def test_invalid_path_or_contract_does_not_resume_parallel(change):
    output, data, current, scene, history, cfg = example()
    if change == 'path':
        output[2]['normal']['parallel_hold_trial']['path_points'][0][1] = .1
    if change == 'partial':
        output[2]['normal']['partial_observation']['reason'] = 'narrow_gap'
    if change == 'departure':
        output[2]['normal']['parallel_hold_trial']['side_departure'] = {'applied': True}
    if change == 'expired':
        output[2]['normal']['path_source_s'] = .1
    if change == 'request':
        current['previous_steering_rad'] = .1
    if change == 'motion_epoch':
        current['epoch'] = 1
    assert rebase(output, data, current, scene, history, cfg)['speed_mps'] == 0


def test_new_current_obstacle_stops_selected_parallel_path():
    values = [3.]*500
    values[245:256] = [.18]*11
    assert rebase(*example(ranges=values))['speed_mps'] == 0


def test_current_lateral_movement_is_not_ignored_for_a_fast_vehicle():
    output, data, current, scene, history, cfg = example()
    current['speed_mps'] = 1.5
    for path in (output[2]['normal']['path_points'], output[2]['normal']['parallel_hold_trial']['path_points']):
        for point in path:
            point[1] = point[0]*.15
    result = rebase(output, data, current, scene, history, cfg)
    assert result['speed_mps'] == 0
    assert result['reason'] == 'rebase_parallel_lateral_motion_exceeded'


def test_zero_or_bypass_is_not_revived():
    output, *rest = example()
    assert rebase((0., output[1], output[2]), *rest) is None
    output[2]['handoff_state'] = 'BYPASS'
    assert not supported(output) and rebase(output, *rest) is None
