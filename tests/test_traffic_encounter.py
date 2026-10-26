import pytest
from traffic_encounter import encounter_metrics


class Straight:
    def nearest_s(self, x, y):
        return x, abs(y)
    def pose(self, s):
        return s, 0., 0.


def test_moving_actor_passes_stationary_ego_and_keeps_tight_gap_as_failure_data():
    ego = [(i*.01, 0., -.09, 0.) for i in range(201)]
    actor = [(i*.01, -.5+i*.005, .13, 0.) for i in range(201)]
    result = encounter_metrics(ego, actor, Straight(), 0., 2.)
    assert result['valid'] and result['behind_to_ahead_observed']
    assert result['minimum_gap_m'] == pytest.approx(.07)
    assert result['first_below_gap_motion']['ego_translation_m'] == 0
    assert result['first_below_gap_motion']['actor_translation_m'] > 0
    assert result['maximum_ego_road_lateral_change_m'] == 0


def test_side_presence_without_crossing_is_not_passing():
    ego = [(i*.01, 0., -.09, 0.) for i in range(101)]
    actor = [(i*.01, .1, .13, 0.) for i in range(101)]
    result = encounter_metrics(ego, actor, Straight(), 0., 1.)
    assert result['valid'] and not result['behind_to_ahead_observed']


def test_pose_gap_does_not_get_zero_missing():
    ego = [(i*.01, 0., 0., 0.) for i in range(101)]
    actor = [(0., -.5, .3, 0.), (1., .5, .3, 0.)]
    assert not encounter_metrics(ego, actor, Straight(), 0., 1.)['valid']


def test_empty_is_not_pass():
    assert not encounter_metrics([], [], Straight(), 0., 1.)['valid']


def test_missing_window_end_cannot_claim_valid_encounter():
    ego = [(i*.01, 0., 0., 0.) for i in range(81)]
    actor = [(i*.01, -.5+i*.005, .3, 0.) for i in range(101)]
    assert not encounter_metrics(ego, actor, Straight(), 0., 1.)['valid']
