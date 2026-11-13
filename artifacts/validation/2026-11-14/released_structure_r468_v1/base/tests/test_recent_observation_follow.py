from dataclasses import replace

import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from test_local_follow import history
from test_corridor_visibility import scene
from test_linear_observation import unknown_at


def config():
    return FollowConfig(boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate',
                        corridor_fallback_enabled=True, recent_observation_enabled=True)


def mocked_path(monkeypatch):
    follow = LocalFollow(config())
    monkeypatch.setattr(follow, '_path_command', lambda *args:(.8, 0., {'reason':'test_geometry', 'behavior':'CRUISE'}))
    return follow


def test_option_off_and_dependent_candidates_required():
    assert LocalFollow().observation_cache is None
    with pytest.raises(ValueError):
        FollowConfig(recent_observation_enabled=True)
    with pytest.raises(ValueError):
        replace(config(), max_speed_mps=1.1)


@pytest.mark.parametrize('value', [1, 'true', None])
def test_option_requires_real_bool(value):
    with pytest.raises(ValueError):
        replace(config(), recent_observation_enabled=value)


def test_side_missing_is_verified_not_filled_and_capped(monkeypatch):
    follow = mocked_path(monkeypatch)
    output = follow.command(unknown_at(90), history(), 0., 'left')
    assert output[0] == .2 and output[1] == 0.
    assert output[2]['recent_observation_selected']
    assert output[2]['recent_observation']['unknown_filled'] is False
    assert output[2]['recent_observation']['rear_dynamic_risk_unresolved'] is True


@pytest.mark.parametrize('observation, speed', [(unknown_at(0),0.), (unknown_at(90),-.01),
                                             (unknown_at(90),1.2)])
def test_bad_input_overrides_positive_geometry(monkeypatch, observation, speed):
    follow = mocked_path(monkeypatch)
    output = follow.command(observation, history(), speed, 'left')
    assert output[0] == 0.
    assert output[2]['reason'] == 'follow_observation_unconfirmed'
    assert follow.held and follow.release_count == 0


def test_near_return_has_priority_even_with_positive_path(monkeypatch):
    follow = mocked_path(monkeypatch)
    original = scene(.25)
    missing = unknown_at(90)
    rays = tuple(missing.rays[i] if i==375 else r for i,r in enumerate(original.rays))
    output = follow.command(replace(original,rays=rays), history(), 0., 'left')
    assert output[0] == 0.
    assert output[2]['recent_observation']['reason'] == 'current_occupancy_overrides_history'


def test_clear_input_keeps_original_target_and_holds_are_not_overridden(monkeypatch):
    follow = mocked_path(monkeypatch)
    assert follow.command(scene(), history(), 0., 'left')[0] == .8
    monkeypatch.setattr(follow, '_path_command', lambda *args:(0., 0., {'reason':'original_hold'}))
    output = follow.command(unknown_at(90), history(), 0., 'left')
    assert output[0] == 0. and output[2]['reason'] == 'original_hold'
    assert not output[2]['recent_observation_selected']


def test_reset_clears_observation_history(monkeypatch):
    follow = mocked_path(monkeypatch)
    follow.command(scene(), history(), 0., 'left')
    assert len(follow.observation_cache.scenes) == 1
    follow.reset()
    assert not follow.observation_cache.scenes
