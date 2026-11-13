"""HOLD 요청과 실제 정지, 원래 사례 요구조건을 혼동하지 않는 평가."""
import copy
import math

import pytest

import audit_g1_hold_evidence as audit


def fixture(kind='stop_restart'):
    times = [1+i*.02 for i in range(101)]
    events = [dict(seq=0, kind='header', data={'schema': 1})]
    for t in times:
        events.append(dict(seq=len(events), kind='call', data={
            'input': {'reference_s': t}, 'output': [0., 0., {'behavior': 'HOLD'}]}))
    events.append(dict(seq=len(events), kind='footer', data={
        'capture_errors': 0, 'failed': 0, 'dropped': 0, 'pending': 0,
        'worker_alive': False, 'accepted': len(events), 'written': len(events)}))
    dynamics = [dict(sim_time_s=t, applied_target_mps=0.) for t in times]
    trace = [dict(t=t+.01, safe_speed_mps=0., braking_diagnostics={'safe_command_stamp_s': t}) for t in times]
    poses = [(1+i*.002, 0., 0., 0.) for i in range(1001)]
    config = dict(cases=[dict(name='case', behavior_kind=kind, duration_s=2.)], acceptance={
        'case': dict(hold_window_s=[0., 2.], minimum_hold_s=1., minimum_continuous_hold_s=1.),
        'stop': dict(stable_hold_sim_s=.5)})
    report = dict(case='case', mode='g1', green_observed_sim_s=1.)
    return poses, dynamics, events, trace, config, report


def test_full_dense_hold_preserves_one_second_requirement():
    result = audit.audit(*fixture())
    assert result['dense_hold_requirement_met']
    assert result['longest_dense_hold_s'] == pytest.approx(2.)
    assert result['requirement']['required_hold_s'] == 1.
    assert not result['driving_authorized'] and not result['original_verdict_changed']


@pytest.mark.parametrize('fault', ['move', 'pulse', 'gap', 'truncated', 'reset', 'safe_nonzero', 'applied_nonzero'])
def test_required_hold_cannot_hide_motion_or_missing_evidence(fault):
    p, d, e, tr, c, r = fixture()
    if fault == 'move':
        p = [(t, .011*(t-1.), y, z) for t, _, y, z in p]
    if fault == 'pulse':
        # 임계값 초과 한 번이 기존 1초 연속 구간을 둘로 자른다.
        p = [(t, .02*min(.08, max(0., t-1.96)), y, z) for t, _, y, z in p]
    if fault == 'gap': p = [q for q in p if not 1.95 < q[0] < 2.06]
    if fault == 'truncated': p = p[:400]
    if fault == 'reset': e[51]['kind'], e[51]['data'] = 'reset', {'reason': 'input'}
    if fault == 'safe_nonzero': tr[50]['safe_speed_mps'] = .1
    if fault == 'applied_nonzero': d[50]['applied_target_mps'] = .1
    assert not audit.audit(p, d, e, tr, c, r)['dense_hold_requirement_met']


def test_short_zero_request_motion_is_not_established_stop_loss():
    p, d, e, tr, c, r = fixture('side_occlusion')
    d = d[:16]
    p = [(t, .015*(t-1.), y, z) for t, _, y, z in p]
    result = audit.audit(p, d, e, tr, c, r)
    assert result['dense_hold_requirement_met'] is None
    episode = result['sampled_zero_target_episodes'][0]
    assert episode['maximum_speed_mps'] > .01
    assert episode['settled_after_s'] is None and not episode['movement_after_settled']


def test_motion_after_complete_settled_window_remains_visible():
    p, d, e, tr, c, r = fixture('side_occlusion')
    p = [(t, .02*max(0., t-1.8), y, z) for t, _, y, z in p]
    result = audit.audit(p, d, e, tr, c, r)
    episode = result['sampled_zero_target_episodes'][0]
    assert episode['settled_after_s'] == 1.5
    assert episode['movement_after_settled']


@pytest.mark.parametrize('fault', ['footer', 'dropped', 'order', 'unknown', 'call_nan'])
def test_incomplete_decision_capture_is_not_evidence(fault):
    args = list(fixture())
    e = args[2]
    if fault == 'footer': e.pop()
    if fault == 'dropped': e[-1]['data']['dropped'] = 1
    if fault == 'order': e[40]['seq'] = 39
    if fault == 'unknown': e[40]['kind'] = 'unexpected'
    if fault == 'call_nan': e[40]['data']['input']['reference_s'] = math.nan
    with pytest.raises(ValueError): audit.audit(*args)


def test_safe_source_not_pose_reference_and_duplicate_conflict():
    args = list(fixture())
    del args[3][20]['braking_diagnostics']['safe_command_stamp_s']
    with pytest.raises(ValueError, match='source time'): audit.audit(*args)
    args = list(fixture())
    args[3][20] = copy.deepcopy(args[3][19])
    args[3][20]['safe_speed_mps'] = 1.
    with pytest.raises(ValueError, match='different values'): audit.audit(*args)


def test_gap_does_not_join_zero_target_or_low_pose_intervals():
    p, d, e, tr, c, r = fixture()
    d = [q for q in d if not 1.95 < q['sim_time_s'] < 2.06]
    result = audit.audit(p, d, e, tr, c, r)
    assert len(result['sampled_zero_target_episodes']) == 2
    assert not result['dense_hold_requirement_met']
