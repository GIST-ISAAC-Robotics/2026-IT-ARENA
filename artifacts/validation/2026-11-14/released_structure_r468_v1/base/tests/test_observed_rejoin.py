"""현재 관측으로 복귀할 수 있을 때만 중앙 정렬 전 G1에 넘기는 시험 후보."""
from dataclasses import replace
import pytest

from arena_autonomy.bypass_handoff import HandoffExecutionConfig
from test_bypass_handoff import stubbed, context


def subject(policy='observed_rejoin_v2', phase='rejoin'):
    follow, _ = stubbed(phase=phase)
    follow.execution = replace(follow.execution, reacquire_policy=policy)
    detail = dict(selector_choice='baseline', path_points=[[0., 0.], [1., 0.]],
                  swept_clearance_m=1., observed_path_ahead_m=1.,
                  selector_sides={'left': dict(seen=True, present=False, passed=True),
                                  'right': dict(seen=False, present=False, passed=False)})
    follow.normal.command = lambda *a, **k: (.7, .02, detail)
    follow.mode = 'BYPASS'
    return follow, detail


def call(follow, stamp):
    scan, history, scene = context(stamp)
    return follow.command(scene, history, .3, 'left', 0., .02, raw_scan=scan)


def test_two_distinct_scans_allow_observed_rejoin_without_centered_phase():
    follow, _ = subject()
    side_history = follow.normal.pass_selector.sides
    for stamp in (.5, .5):
        out = call(follow, stamp)
        assert follow.mode == 'REACQUIRE' and out[2]['observed_rejoin_ready']
    out = call(follow, .6)
    assert out[2]['reason'] == 'handoff_return' and follow.mode == 'FOLLOW'
    assert out[0] == .35  # 전환 첫 명령은 기존 상한. 이후 G1이 가속한다.
    assert follow.normal.pass_selector.sides is side_history


@pytest.mark.parametrize('policy,phase', [('known_hold_v1','rejoin'),
                                         ('positive_command_v0','rejoin'),
                                         ('observed_rejoin_v2','entry')])
def test_old_policies_and_outbound_entry_keep_original_transition(policy, phase):
    follow, _ = subject(policy, phase)
    call(follow, .5)
    assert call(follow, .6)[2]['reason'] == 'handoff_bypass'
    assert follow.mode == 'BYPASS'


@pytest.mark.parametrize('key,value', [('path_points', []), ('observed_path_ahead_m', 0.),
                                     ('swept_clearance_m', float('nan')),
                                     ('selector_choice', 'parallel'), ('selector_sides', {})])
def test_unavailable_or_blocked_g1_path_cannot_replace_bypass(key, value):
    follow, detail = subject()
    detail[key] = value
    call(follow, .5)
    assert call(follow, .6)[2]['reason'] == 'handoff_bypass'


@pytest.mark.parametrize('threat', [dict(seen=True,present=True,passed=True),
                                  dict(seen=True,present=False,passed=False)])
def test_present_or_unresolved_side_threat_keeps_bypass(threat):
    follow, detail = subject()
    detail['selector_sides']['left'] = threat
    call(follow, .5)
    assert call(follow, .6)[2]['reason'] == 'handoff_bypass'


def test_lost_path_between_confirmations_restarts_reacquisition():
    follow, detail = subject()
    call(follow, .5)
    detail['observed_path_ahead_m'] = 0.
    assert call(follow, .6)[2]['reason'] == 'handoff_bypass'
    detail['observed_path_ahead_m'] = 1.
    call(follow, .7)
    assert follow.mode == 'REACQUIRE'
    assert call(follow, .8)[2]['reason'] == 'handoff_return'


def test_g1_stop_is_not_early_rejoin_permission():
    follow, detail = subject()
    follow.normal.command = lambda *a, **k: (0., .02, detail)
    call(follow, .5)
    assert call(follow, .6)[2]['reason'] == 'handoff_bypass'


def test_default_and_old_mapping_keep_old_policy():
    assert HandoffExecutionConfig().reacquire_policy == 'positive_command_v0'
    assert HandoffExecutionConfig.from_mapping({'bypass': {}}).reacquire_policy == 'positive_command_v0'


def test_old_debug_shape_does_not_gain_new_policy_field():
    follow,_ = subject('known_hold_v1')
    assert 'observed_rejoin_ready' not in call(follow,.5)[2]
