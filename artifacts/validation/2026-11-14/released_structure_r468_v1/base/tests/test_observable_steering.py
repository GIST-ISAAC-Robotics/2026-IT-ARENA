import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from probe_observable_steering import path_deviation, inspect
from test_corridor_visibility import scene
from test_linear_observation import unknown_at


def straight():
    return np.column_stack((np.linspace(.02, .8, 40), np.zeros(40)))


def test_straight_path_agrees_but_large_shift_does_not():
    cfg = FollowConfig()
    assert path_deviation(straight(), 0., .3, cfg)['supported']
    assert not path_deviation(straight()+[0., .2], 0., .3, cfg)['supported']


def test_path_end_is_not_extended():
    assert not path_deviation(straight()[:5], 0., .6, FollowConfig())['supported']


@pytest.mark.parametrize('path', [None, [[0., 0.]], [[0., 0.]]*5])
def test_missing_or_degenerate_path_rejected(path):
    assert not path_deviation(path, 0., .4, FollowConfig())['supported']


def test_no_runtime_authority_and_front_unknown_stays_unknown():
    observation = unknown_at(0)
    result = inspect(observation, [observation], 0., 0., 0., straight(), FollowConfig())
    assert not result['supported']
    assert result['runtime_connected'] is False and result['driving_authorized'] is False


def test_current_obstacle_overrides_older_evidence():
    result = inspect(scene(.25), [scene(.25), scene()], 0., 0., 0., straight(), FollowConfig())
    assert result['reason'] == 'current_occupancy' and not result['supported']
