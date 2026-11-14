"""명시 정책의 인스턴스 격리·기존 실제 속도/현재 관측 제한을 확인한다."""
from dataclasses import replace

import pytest
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import guard_observation_policy as selected
from arena_autonomy import pass_release_trial as ps
from test_guard_curve_trial import guard
from test_guard_masked_speed import speed_region, PARTIAL


@pytest.mark.parametrize('value', ['unknown', '', False, None, 'selected_r181'])
def test_unknown_or_incomplete_selected_profile_rejected(value):
    with pytest.raises(ValueError):
        FollowConfig(guard_observation_policy=value)


def test_default_and_selected_instances_do_not_patch_each_other(guard, monkeypatch):
    old, _, _ = guard
    enabled = ps.PassReleaseSelector(replace(old.follow_config, traffic_recovery_trial=True,
                                             guard_observation_policy='selected_r181'))
    guard_method, gap_function = ps.PassReleaseSelector._guard_curve, ps.same_path_risk
    before, after = object(), object()
    monkeypatch.setattr(ps.PassReleaseSelector, '_guard_curve_original', lambda *args: before)
    monkeypatch.setattr(selected, 'guard_curve', lambda *args: after)
    args = [None]*10
    assert old._guard_curve(*args) is before
    assert enabled._guard_curve(*args) is after
    assert old._guard_curve(*args) is before
    monkeypatch.setattr(ps, '_same_path_risk_original', lambda *args: before)
    monkeypatch.setattr(selected, 'same_path_risk', lambda *args: after)
    assert ps.same_path_risk(old, *([None]*9)) is before
    assert ps.same_path_risk(enabled, *([None]*9)) is after
    assert ps.PassReleaseSelector._guard_curve is guard_method and ps.same_path_risk is gap_function
    assert old.follow_config.guard_observation_policy == 'original_v1'


@pytest.mark.parametrize('measured, expected', [(0., .15), (.35, 0.), (.8, 0.)])
def test_selected_policy_preserves_actual_speed_and_side_history(guard, monkeypatch, measured, expected):
    owner, state, call = guard
    owner.follow_config = replace(owner.follow_config, traffic_recovery_trial=True,
                                  guard_observation_policy='selected_r181')
    speed_region(monkeypatch, state)
    out = call(speed=measured, angle=0., partial=PARTIAL)
    assert out[0] == expected
    assert all(row[0] == measured for row in state['regions'])
    assert owner.mode == 'GUARD' and owner.sides['left']['present']
    assert not owner.sides['left']['passed']


@pytest.mark.parametrize('partial', [None, {}, dict(allowed=False,reason='fully_observed'),
                                    dict(allowed=True,reason='bounded_side_missing')])
def test_selected_policy_does_not_replace_current_observation(guard, monkeypatch, partial):
    owner, state, call = guard
    owner.follow_config = replace(owner.follow_config, traffic_recovery_trial=True,
                                  guard_observation_policy='selected_r181')
    speed_region(monkeypatch, state)
    assert call(speed=0., angle=0., partial=partial)[0] == 0.
    assert len(state['regions']) == 1
