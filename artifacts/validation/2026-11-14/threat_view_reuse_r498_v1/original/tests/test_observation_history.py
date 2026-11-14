"""과거 깊이 여유만으로 측방 미관측의 시간 불확실성을 덮지 않는다."""
from dataclasses import replace

import numpy as np

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.observation_support import depth_support
from audit_observation_history import lateral_support
from test_corridor_visibility import scene
from test_linear_observation import unknown_at


def test_depth_away_from_unknown_stays_supported():
    supported, reachable, _ = lateral_support(scene(), [[.4, 0.]], FollowConfig())
    assert supported.tolist() == [True]
    assert reachable.tolist() == [False]


def test_finite_neighbors_do_not_cover_lateral_motion_from_unknown():
    observation = unknown_at(90)
    query = [[.07, .09]]
    assert depth_support(observation, query, np.full(500, .1)).tolist() == [True]
    supported, reachable, examples = lateral_support(observation, query, FollowConfig())
    assert supported.tolist() == [False]
    assert reachable.tolist() == [True]
    assert examples[0]['separation_m'] < examples[0]['age_motion_bound_m']


def test_age_never_refreshes_lateral_clearance():
    original = unknown_at(90)
    query = [[.09, .09]]
    young, _, _ = lateral_support(original, query, FollowConfig())
    old, _, _ = lateral_support(replace(original, t_ref_s=.55), query, FollowConfig())
    assert not np.any(old & ~young)


def test_unknown_itself_is_not_accepted_even_when_old_depth_is_long():
    supported, _, _ = lateral_support(unknown_at(90), [[.06, .09]], FollowConfig())
    assert supported.tolist() == [False]
