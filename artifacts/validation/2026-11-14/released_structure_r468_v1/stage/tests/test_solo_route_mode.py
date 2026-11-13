"""단독 모드는 교통 이력만 제외한다. 실제 장애물과 관측 누락은 계속 검사한다."""
from dataclasses import asdict

import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy
from test_low_speed_arc_reacquire import SOLO, corridor, launch_module, play_scene


def test_default_traffic_instances_and_solo_recovery_policy():
    traffic = LocalFollow(FollowConfig(**SOLO))
    solo = LocalFollow(FollowConfig(**SOLO, solo_route_only=True))
    assert traffic.parallel_trial is not None and traffic.pass_selector is not None
    assert solo.parallel_trial is None and solo.pass_selector is None
    assert isinstance(solo.partial_policy, RecoveryPolicy)
    assert solo.corridor_observer is not None
    assert asdict(solo.config)['solo_route_only'] is True
    assert FollowConfig().solo_route_only is False


@pytest.mark.parametrize('value', [1, 'true', None])
def test_solo_requires_boolean(value):
    with pytest.raises(ValueError, match='solo route'):
        FollowConfig(solo_route_only=value)


def test_registered_profile_changes_only_solo_selection():
    profiles = launch_module().SPEED_PROFILES
    assert profiles['g1_open_10kmh_solo'] == {
        **profiles['g1_open_10kmh_turn15_arc'], 'g1_solo_route_only': True}
    assert [name for name, p in profiles.items() if p.get('g1_solo_route_only')] == ['g1_open_10kmh_solo', 'g1_open_10kmh_solo_gap05', 'g1_open_10kmh_solo_gap05_arc', 'g1_open_10kmh_solo_gap05_arc_ray', 'g1_open_10kmh_solo_gap05_arc_preview', 'g1_open_10kmh_solo_gap05_arc_preview_bridge', 'g1_open_10kmh_solo_gap05_arc_preview_bridge_ray']


@pytest.mark.parametrize('headway', [.8, .5])
def test_solo_visible_corridor_progresses(headway):
    rows = play_scene(corridor(), True, config_changes={'solo_route_only': True, 'headway_s': headway})
    assert any(row.v > 0 for row in rows)


@pytest.mark.parametrize('distance', [.22, .30, .38])
@pytest.mark.parametrize('headway', [.8, .5])
def test_solo_keeps_observed_front_obstacle_stop(distance, headway):
    rows = play_scene(corridor(((distance, -.425, distance, .425),)), True,
                      config_changes={'solo_route_only': True, 'headway_s': headway})
    assert all(row.v == 0 for row in rows)


@pytest.mark.parametrize('headway', [.8, .5])
def test_solo_does_not_fill_front_no_returns(headway):
    rows = play_scene(corridor(), True, blind=((-.3, .3),), config_changes={'solo_route_only': True, 'headway_s': headway})
    assert all(row.v == 0 for row in rows)


def test_solo_headway_profile_changes_only_time_gap():
    profiles = launch_module().SPEED_PROFILES
    assert profiles['g1_open_10kmh_solo_gap05'] == {**profiles['g1_open_10kmh_solo'], 'g1_solo_headway_s': .5}
