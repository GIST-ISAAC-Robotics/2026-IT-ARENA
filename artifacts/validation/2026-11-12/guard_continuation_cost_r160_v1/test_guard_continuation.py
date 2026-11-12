"""과거 허가를 현재 허가로 복사하지 않으며 상향 탐색과 reset을 보장한다."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location('r160_candidate_test', Path(__file__).with_name('candidate.py'))
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)


@pytest.fixture
def setup(monkeypatch):
    calls = []
    def original(owner, *args):
        calls.append(('original', args))
        return .15, .2, {'selector_horizon_recheck': {'applied': True},
                         'selector_guard_candidate': {'accepted': True}}
    def current(*args, **kwargs):
        calls.append(('current', args, kwargs))
        return .15, .21, {'selector_guard_candidate': {'accepted': True}}
    monkeypatch.setattr(c, '_old_guard', original)
    monkeypatch.setattr(c, '_old_reset', lambda owner: calls.append(('reset',)))
    owner = SimpleNamespace(mode='GUARD', last_choice='guard_baseline',
        follow_config=SimpleNamespace(traffic_recovery_trial=True),
        sides={'left': {'present': False}, 'right': {'present': False}},
        _guard_curve_at_cap=current)
    scene = SimpleNamespace(source_epoch=1, source_first_s=1., t_ref_s=1.1)
    baseline = (.4, .2, {'boundary_from_memory': False, 'path_source_s': 1.,
        'partial_observation': {'allowed': True, 'reason': 'fully_observed'}})
    args = [scene, object(), baseline, (0., .1, {}), .1, .19, .02, .1, {}, object()]
    return owner, args, calls


def test_current_full_check_and_current_steering_used(setup):
    owner, args, calls = setup
    c.guard_continuation(owner, *args)
    args[0].t_ref_s += .1
    out = c.guard_continuation(owner, *args)
    assert [x[0] for x in calls] == ['original', 'current']
    assert calls[-1][1] == tuple(args)
    assert calls[-1][2] == {'target_cap': .15}
    assert out[:2] == (.15, .21)
    assert out[2]['selector_guard_continuation']['current_full_guard_rechecked']


@pytest.mark.parametrize('change', ['epoch', 'time_back', 'source_back', 'expired',
                                  'side', 'partial', 'memory', 'speed', 'mode', 'choice'])
def test_changed_state_uses_original_search(setup, change):
    owner, args, calls = setup
    c.guard_continuation(owner, *args)
    scene, _, baseline = args[:3]
    if change == 'epoch': scene.source_epoch = 2
    elif change == 'time_back': scene.t_ref_s = 1.09
    elif change == 'source_back': scene.source_first_s = .9
    elif change == 'expired': scene.t_ref_s = owner._guard_cost_state['probe_at_s']
    elif change == 'side': owner.sides['right']['present'] = True
    elif change == 'partial': baseline[2]['partial_observation']['allowed'] = False
    elif change == 'memory': baseline[2]['boundary_from_memory'] = True
    elif change == 'speed': args[4] = .151
    elif change == 'mode': owner.mode = 'BASELINE'
    elif change == 'choice': owner.last_choice = 'parallel'
    c.guard_continuation(owner, *args)
    assert [x[0] for x in calls] == ['original', 'original']


@pytest.mark.parametrize('current', [None, (0., .2, {'selector_guard_candidate': {'accepted': True}}),
                                   (.15, .2, {'selector_guard_candidate': {'accepted': False}})])
def test_failed_current_check_never_returns_cached_permission(setup, current):
    owner, args, calls = setup
    c.guard_continuation(owner, *args)
    owner._guard_curve_at_cap = lambda *a, **k: current
    c.guard_continuation(owner, *args)
    assert [x[0] for x in calls] == ['original', 'original']


def test_reset_removes_continuation_and_deadline_does_not_slide(setup):
    owner, args, calls = setup
    c.guard_continuation(owner, *args)
    deadline = owner._guard_cost_state['probe_at_s']
    for dt in (.05, .05, .05):
        args[0].t_ref_s += dt
        c.guard_continuation(owner, *args)
        assert owner._guard_cost_state['probe_at_s'] == deadline
    c.reset(owner)
    assert owner._guard_cost_state is None
    c.guard_continuation(owner, *args)
    assert calls[-1][0] == 'original'
