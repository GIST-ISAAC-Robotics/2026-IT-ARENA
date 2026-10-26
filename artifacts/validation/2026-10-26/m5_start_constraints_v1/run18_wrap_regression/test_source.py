"""측방 위협 중 직진 우선은 선택 순서만 변경하고 원래 검사를 면제하지 않는다."""
from dataclasses import replace
import numpy as np
import pytest

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.lateral_intent import ego_state
from test_lateral_intent import GRASS, car
from test_m3_execution_trial import padding
from test_m3_observed_follow import timed_scene
from test_m3_small_center_straight import road_path, step_order, ON as CENTER_ON
from test_local_follow import history

ON = mx.ExecutionConfig(command_fallback=True, release_to_baseline=True,
    straight_surface_commitment=True, straight_with_threat=True)
OFF = replace(ON, straight_with_threat=False)


def test_typed_off_option_requires_actual_command_check():
    assert mx.ExecutionConfig().straight_with_threat is False
    with pytest.raises(ValueError): mx.ExecutionConfig(straight_with_threat=1)
    with pytest.raises(ValueError): mx.ExecutionConfig(straight_with_threat=True)
    with pytest.raises(ValueError): mx.ExecutionConfig(command_fallback=True, straight_start=False, straight_with_threat=True)


@pytest.mark.parametrize('offset,heading,radius,accepted', [
    (.09,0.,None,True), (-.12,0.,None,True), (0.,.03,None,False),
    (0.,0.,1.2,False), (0.,0.,4.,False)])
def test_parallel_prediction_counts_heading_and_curve(offset, heading, radius, accepted):
    frame = mx.ExecutionFrame(road_path(offset, heading, radius))
    state = ego_state(frame, GRASS)
    result = mx.parallel_straight_check(mx.straight_candidate(frame, state, GRASS), state, GRASS)
    assert result['ok'] is accepted
    assert result['limit_m'] == .022


def trial(execution, *, gap=.26, dropout=None, reject=False):
    follow = mx.ExecutionLateralTrial(GRASS, execution)
    scene, path = timed_scene(.4, -.09, extra=[car(.0, gap, -.09)], dropout=dropout)
    checked = []
    def command(name, path):
        checked.append(name)
        return {'accepted': not reject, 'reason': 'test_command_rejection' if reject else None}
    result = follow.step(scene, [], path, .35, padding(scene), command, history=history())
    return result, checked, follow


def test_threat_and_release_records_unchanged_but_parallel_first():
    on, checked, a = trial(ON)
    off, _, b = trial(OFF)
    assert on['threats']['left']['present']
    assert on['threats'] == off['threats'] and on['release_gate'] == off['release_gate']
    assert list(on['checks'])[0] == 'straight'
    assert list(off['checks'])[0] != 'straight'
    assert a.records.keys() == b.records.keys() and a.active and b.active
    if on['selected'] == 'straight':
        assert checked[0] == 'straight' and on['checks']['straight']['eligible_for_trial']


def test_unknown_and_too_close_neighbor_still_reject_straight():
    blind, _, _ = trial(ON, dropout=lambda deg: np.abs(deg)<12.)
    assert blind['selected'] != 'straight'
    assert not blind['checks']['straight']['eligible_for_trial']
    close, _, _ = trial(ON, gap=.07)
    assert close['selected'] != 'straight'
    assert 'lateral_clearance_short' in close['checks']['straight']['reasons']


def test_command_rejection_is_not_bypassed_and_other_candidates_remain():
    out, calls, _ = trial(ON, reject=True)
    assert out['selected'] is None and out['path'] is None
    assert list(out['checks'])[0]=='straight' and len(out['checks']) >= 2
    assert all(not v.get('command', {}).get('accepted', False) for v in out['checks'].values())


def test_no_threat_large_return_keeps_previous_center_order():
    on, checks, _ = step_order(replace(CENTER_ON, straight_with_threat=True), -.09)
    off, prior, _ = step_order(CENTER_ON, -.09)
    assert 'parallel_straight' not in on
    assert list(on['checks']) == list(off['checks']) and list(on['checks'])[0]=='center'
    assert checks == prior and on['selected'] == off['selected']
