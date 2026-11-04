"""앞차 가까이 멈춘 뒤의 우회 전이 선택과 측방 시작 합류의 경계."""
from dataclasses import replace
import math

import pytest

from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig, handoff_baseline
from traffic_merge import MergeProfile, merge_geometry


def guide(steerable):
    return BypassEntryFollow(handoff_baseline(), BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold',
                             steerable_transition=steerable), allow_simulation_trial=True).guide


def test_default_keeps_the_distance_transition_even_when_not_steerable():
    assert guide(False)._transition([.6], .267) == .6


def test_option_picks_the_shortest_steerable_registered_transition():
    assert guide(True)._transition([.6], .267) == .8
    assert guide(True)._transition([], .267) == .8


def test_option_never_shortens_a_transition_that_already_fits():
    assert guide(True)._transition([.6, .8, .9], .267) == .9
    assert guide(True)._transition([.6], .10) == .6


def test_chosen_transition_stays_inside_the_steering_limit():
    item = guide(True)
    config = item.config
    chosen = item._transition([.6], .27)
    curvature = .27*(10/math.sqrt(3))/chosen**2
    assert math.atan(config.wheelbase_m*curvature) < config.max_steering_rad


def test_handoff_keeps_the_option_after_replacing_the_guide():
    execution = HandoffExecutionConfig()
    baseline = replace(execution, bypass=replace(execution.bypass, corridor_policy='corridor_v2_signed_hold', steerable_transition=True))
    from arena_autonomy.bypass_handoff_corridor_trial import configure_guide
    follow = BypassHandoffFollow(handoff_baseline(), baseline, allow_simulation_trial=True)
    assert follow.bypass.guide.steerable_transition is True
    configure_guide(follow)
    assert follow.bypass.guide.steerable_transition is True


def test_side_start_needs_an_explicit_profile_limit():
    plain = MergeProfile(.5, .8, 0.)
    with pytest.raises(ValueError):
        merge_geometry(0., -.30, plain)
    side = MergeProfile(0., .9, 0., maximum_initial_offset_m=.32)
    assert merge_geometry(0., -.30, side)['rear_lateral_m'] == pytest.approx(-.30)
    with pytest.raises(ValueError):
        MergeProfile(0., .9, 0., maximum_initial_offset_m=.5)
