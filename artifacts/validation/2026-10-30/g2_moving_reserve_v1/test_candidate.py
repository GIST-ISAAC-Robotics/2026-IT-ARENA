"""추가 예약만 비교하고, 이동 관측이 멈추거나 사라질 때의 경계를 확인한다."""
import sys
from pathlib import Path
from unittest.mock import patch
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from candidate import MovingReserveCandidate
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from test_bypass_handoff import context, moving_monitor


def subject():
    return MovingReserveCandidate(execution=HandoffExecutionConfig(
        preparation_policy='observed_entry_space_v1',
        persistence_window_policy='trim_before_compare_v1',
        follow_entry_space_enabled=True, front_patch_policy='sparse_returns_v1',
        reacquire_policy='known_hold_v1'), allow_simulation_trial=True)


@pytest.mark.parametrize('target', [0., .12, .35, .8])
def test_observed_motion_removes_only_extra_reserve(target):
    h = subject(); h.monitor, states = moving_monitor([.25]*12)
    assert states[-1][0] and not states[-1][2]
    _, _, scene = context()
    with patch.object(BypassHandoffFollow, '_follow_entry_space', side_effect=AssertionError('reserve called')):
        speed, detail = h._follow_entry_space(scene, (target, .1, {}), .08, target)
    assert speed == target
    assert not detail['applied'] and not detail['free_space_assumed']
    assert not detail['identity_or_stationarity_proven']
    assert h.mode == 'FOLLOW'


@pytest.mark.parametrize('kind', ['initial', 'stationary', 'stopping', 'jump', 'reset', 'ready'])
def test_no_current_motion_uses_exact_existing_reserve(kind):
    h = subject()
    if kind == 'stationary': h.monitor, _ = moving_monitor([0.]*12)
    elif kind == 'stopping': h.monitor, _ = moving_monitor([.25]*6+[0.]*10)
    elif kind == 'jump': h.monitor, _ = moving_monitor([0., 0., 0., 3.5, 0., 0.])
    elif kind == 'reset':
        h.monitor, _ = moving_monitor([.25]*12); h.monitor.reset()
    elif kind == 'ready':
        h.monitor, _ = moving_monitor([.25]*12); h.monitor.result['ready'] = True
    _, _, scene = context()
    sentinel = (.123, {'reason': 'unchanged'})
    with patch.object(BypassHandoffFollow, '_follow_entry_space', return_value=sentinel) as parent:
        actual = h._follow_entry_space(scene, (.8, .1, {}), .08, .8)
    assert actual is sentinel and parent.call_count == 1


def test_no_runtime_policy_or_base_mutation():
    assert not HandoffExecutionConfig().follow_entry_space_enabled
    assert BypassHandoffFollow._follow_entry_space is not MovingReserveCandidate._follow_entry_space
