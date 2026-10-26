"""위협 소멸 뒤 hold 미세 조향의 대안. 중앙 복귀/관측/해제 장부는 유지한다."""
from dataclasses import replace

import numpy as np
import pytest

import arena_autonomy.m3_execution_trial as mx
from test_local_follow import history
from test_m3_execution_trial import padding
from test_m3_observed_follow import timed_scene
from test_m3_parallel_straight import trial as threat_trial, ON as THREAT_ON
from test_m3_small_center_straight import active, step_order, road_path, GRASS

OFF = mx.ExecutionConfig(command_fallback=True, release_to_baseline=True)
ON = replace(OFF, straight_before_hold=True)


def test_default_off_and_required_contracts():
    assert not mx.ExecutionConfig().straight_before_hold
    for values in ({'straight_before_hold': 1}, {'straight_before_hold': True},
                   {'command_fallback': True, 'straight_before_hold': True},
                   {'command_fallback': True, 'release_to_baseline': True,
                    'straight_start': False, 'straight_before_hold': True}):
        with pytest.raises(ValueError): mx.ExecutionConfig(**values)


def waiting(execution, *, reject=False, dropout=None, altered_path=None):
    follow = active(execution, -.09)
    follow.clear_fresh = 0
    scene, path = timed_scene(.4, -.09, dropout=dropout)
    checked = []
    def command(name, _):
        checked.append(name)
        return {'accepted': not reject or name != 'straight',
                'reason': 'test_straight_rejection' if reject and name == 'straight' else None}
    out = follow.step(scene, [], path if altered_path is None else altered_path, .35,
                      padding(scene), command, history=history())
    return out, checked, follow


def test_wait_for_fresh_center_prefers_parallel_straight_without_releasing():
    on, seen, a = waiting(ON)
    off, _, b = waiting(OFF)
    assert 'center' not in on['checks'] and list(on['checks'])[0] == 'straight'
    assert list(off['checks'])[0] == 'hold'
    assert on['selected'] == 'straight' and seen == ['straight']
    assert a.active and a.clear_fresh == b.clear_fresh == 1
    assert on['threats'] == off['threats'] and on['release_gate'] == off['release_gate']
    assert a.records == b.records and a.release_records == b.release_records


def test_large_center_is_still_first_and_accepted_center_is_unchanged():
    on, seen, _ = step_order(ON, -.09)
    off, old_seen, _ = step_order(OFF, -.09)
    assert list(on['checks'])[0] == list(off['checks'])[0] == 'center'
    assert on['selected'] == off['selected'] == 'center' and seen == old_seen


def test_rejected_center_tries_straight_before_hold_but_keeps_center_failure():
    out, seen, follow = step_order(ON, -.09, accept=lambda name: name != 'center')
    assert list(out['checks'])[:2] == ['center', 'straight']
    assert not out['checks']['center']['command']['accepted']
    assert out['selected'] == 'straight' and follow.active


def test_straight_command_rejection_falls_back_and_keeps_record():
    out, seen, _ = waiting(ON, reject=True)
    assert list(out['checks'])[:2] == ['straight', 'hold']
    assert seen[0] == 'straight' and out['selected'] != 'straight'
    assert not out['checks']['straight']['command']['accepted']


def test_unknown_is_not_made_free():
    out, _, _ = waiting(ON, dropout=lambda deg: np.abs(deg)<12.)
    assert out['selected'] != 'straight'
    assert 'current_sweep_unobserved' in out['checks']['straight']['reasons']


@pytest.mark.parametrize('path', [road_path(-.09,.03), road_path(-.09,0.,1.2)])
def test_heading_drift_or_curve_preserves_original_order(path):
    on, _, _ = waiting(ON, altered_path=path)
    off, _, _ = waiting(OFF, altered_path=path)
    assert not on['parallel_hold_straight']['ok']
    assert list(on['checks']) == list(off['checks'])
    assert on['selected'] == off['selected']


def test_present_threat_order_and_checks_unchanged():
    on, seen, _ = threat_trial(replace(THREAT_ON, straight_before_hold=True))
    off, old_seen, _ = threat_trial(THREAT_ON)
    assert 'parallel_hold_straight' not in on
    assert on['checks'] == off['checks'] and seen == old_seen
    assert on['release_gate'] == off['release_gate'] and on['selected'] == off['selected']
