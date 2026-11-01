from dataclasses import replace
from unittest.mock import patch

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from test_local_follow import history, scene, settle
from test_bypass_handoff import context, follow_space, moving_monitor


def test_open_speed_is_for_an_observed_route_not_a_minimum_command():
    h = history()
    fast = LocalFollow(FollowConfig(open_route_speed_mps=2.))
    baseline = LocalFollow()
    assert settle(baseline,h)[0] == pytest.approx(1.)
    speed, _, detail = settle(fast,h)
    assert speed == pytest.approx(2.) and detail['open_route_speed_enabled']
    lost = fast.command(scene(.8, missing_wall=True),h,0.,'left')
    assert lost[0] <= 1.
    assert fast.command(scene(.9, obstacle_x=.1),h,0.,'left')[0] == 0


def test_length_approach_keeps_the_straight_fast_and_an_obstacle_still_stops():
    cfg = FollowConfig(open_route_speed_mps=2., blend_open_speed_by_length=True)
    h = history()
    assert settle(LocalFollow(cfg), h)[0] == pytest.approx(2.)
    base, trial = LocalFollow(replace(cfg, blend_open_speed_by_length=False)), LocalFollow(cfg)
    for now in (.4, .5, .6, .7):
        s = scene(now)
        short = replace(s, rays=tuple(ray for ray in s.rays if ray.hit_xy_m[0] <= 1.25))
        old = base.command(short, h, 0., 'left')
        new = trial.command(short, h, 0., 'left')
    assert 0. < new[0] < old[0] and new[1] == pytest.approx(old[1])
    assert trial.command(scene(.8, obstacle_x=.1), h, 0., 'left')[0] == 0.


@pytest.mark.parametrize('velocities,expect', [
    ([.25]*12, .25), ([-.25]*12, 0.), ([0.]*12, 0.),
    ([.25]*6+[0.]*10, 0.), ([0.,0.,0.,3.5,0.], 0.),
])
def test_reserve_allows_pacing_only_for_an_advancing_observed_patch(velocities,expect):
    controller=follow_space()
    controller.execution=replace(controller.execution,moving_follow_reserve=True)
    controller.monitor,_=moving_monitor(velocities)
    _,_,sc=context(.6)
    cloud=np.c_[np.ones(5),np.linspace(-.14,-.04,5)]
    with patch('arena_autonomy.bypass_handoff.front_patch',return_value=(cloud,'face')), patch(
            'arena_autonomy.bypass_handoff.straight_reference',return_value=({},{})):
        v,diag=controller._follow_entry_space(sc,(.8,0.,{}),0.,.8)
        assert v==pytest.approx(expect,abs=.01)
        assert controller._follow_entry_space(sc,(0.,0.,{}),0.,0.)[0]==0.
        assert not diag['free_space_assumed']
