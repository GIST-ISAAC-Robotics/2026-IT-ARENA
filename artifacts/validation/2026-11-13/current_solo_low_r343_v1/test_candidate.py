import copy
from types import SimpleNamespace

import pytest
import candidate


def make_fixture(monkeypatch):
    output = [.5, .2, dict(handoff_state='FOLLOW', normal=dict(selector_mode='SOLO'))]
    current = dict(speed_mps=0., reference_s=10.)
    before = dict(speed_mps=0., steering_rad=.2, reference_s=10.,
                  reason='route_current_large_motion_masked_toward_left',
                  current_route_proof=dict(region=dict(queries=100, unobserved=0, masked_front_samples=0)))
    after = dict(before, speed_mps=.15, reason='rebase_current_follow')
    calls = []
    def rebase(value, submitted, cur, scene, history, cfg, **kwargs):
        calls.append((copy.deepcopy(value), cur is current, kwargs))
        return before if len(calls) == 1 else after
    monkeypatch.setattr(candidate.base, 'rebase', rebase)
    return output, current, before, after, calls


def execute(output, current):
    return candidate.rebase(output, {}, current, None, None, SimpleNamespace(), require_current_region=True)


def test_only_target_changes_and_complete_check_is_repeated_once(monkeypatch):
    output, current, before, after, calls = make_fixture(monkeypatch)
    original = copy.deepcopy(output)
    result = execute(output, current)
    assert result['speed_mps'] == .15 and result['current_solo_low_retry']['applied']
    assert output == original and len(calls) == 2
    assert calls[1][0][0] == .15 and list(calls[1][0][1:]) == output[1:]
    assert calls[1][1] and calls[1][2]['require_current_region'] is True


@pytest.mark.parametrize('case', ['zero','negative','already_low','fast','reverse','nan','guard','released',
                                  'hold','unknown','front','empty','obstacle','positive','missing'])
def test_original_other_cases_unchanged(monkeypatch, case):
    output, current, before, after, calls = make_fixture(monkeypatch)
    if case in ('zero','negative','already_low'): output[0] = {'zero':0.,'negative':-.1,'already_low':.15}[case]
    if case in ('fast','reverse','nan'): current['speed_mps'] = {'fast':.2,'reverse':-.02,'nan':float('nan')}[case]
    if case in ('guard','released'): output[2]['normal']['selector_mode'] = case.upper()
    if case == 'hold': output[2]['handoff_state'] = 'HOLD'
    if case == 'unknown': before['current_route_proof']['region']['unobserved'] = 1
    if case == 'front': before['current_route_proof']['region']['masked_front_samples'] = 1
    if case == 'empty': before['current_route_proof']['region']['queries'] = 0
    if case == 'obstacle': before['reason'] = 'rebase_current_collision'
    if case == 'positive': before['speed_mps'] = .5
    if case == 'missing': before.pop('current_route_proof')
    assert execute(output, current) is before and len(calls) == 1


@pytest.mark.parametrize('change', [dict(speed_mps=0.), dict(speed_mps=.2), dict(steering_rad=.21), dict(reference_s=10.1)])
def test_rejected_or_different_steering_and_time_cannot_advance(monkeypatch, change):
    output, current, before, after, calls = make_fixture(monkeypatch)
    after.update(change)
    result = execute(output, current)
    assert result['speed_mps'] == 0 and not result['current_solo_low_retry']['applied']
    assert len(calls) == 2
