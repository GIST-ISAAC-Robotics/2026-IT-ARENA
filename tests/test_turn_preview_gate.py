"""코너 예측을 통상 상한 위로 넓히는 단독 선택. 같은 재평가만 쓰고 정지/재개는 바꾸지 않는다."""
from dataclasses import replace

import numpy as np
import pytest

from arena_autonomy import turn_preview
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.turn_preview import preview_speed
from test_local_follow import history, scan, scene
from test_low_speed_arc_reacquire import launch_module

CONFIG = dict(open_route_speed_mps=2.78, blend_open_speed_by_length=True, solo_route_only=True,
              occupancy_timing_mode='per_ray_candidate', preview_turn_speed=True, headway_s=.5)
CFG = FollowConfig(**CONFIG, preview_above_base_speed=True)
PATH = np.column_stack((np.linspace(-.05, 2.4, 36), np.zeros(36)))


def open_scene(ranges=None):
    return motion_scene(scan(.5, [6.]*500 if ranges is None else ranges), history(), .6, timing_verified=True, epoch=0)


def trial(observed, target=1.2, cap=2.4, clearance=1.4, remaining=2.4):
    return preview_speed(observed, PATH, CFG, 2., 0., 0., .1, cap, target, clearance, remaining)


def drive(controller, obstacles):
    h = history()
    return [controller.command(scene(.4+.1*k, obstacle_x=x), h, 1.5, 'left') for k, x in enumerate(obstacles)]


def test_default_is_off_and_requires_solo_preview():
    assert FollowConfig().preview_above_base_speed is False
    for changes in ({'solo_route_only': False}, {'preview_turn_speed': False}):
        with pytest.raises(ValueError):
            FollowConfig(**{**CONFIG, 'preview_above_base_speed': True, **changes})
    with pytest.raises(ValueError):
        FollowConfig(**CONFIG, preview_above_base_speed=1)


def test_profile_adds_only_this_option_and_leaves_ray_candidate_out():
    profiles = launch_module().SPEED_PROFILES
    new = profiles['g1_open_10kmh_solo_gap05_arc_preview']
    assert new == {**profiles['g1_open_10kmh_solo_gap05_arc'], 'g1_preview_above_base_speed': True}
    assert 'g1_ray_wall_continuity' not in new
    assert [k for k, v in profiles.items() if 'g1_preview_above_base_speed' in v] == ['g1_open_10kmh_solo_gap05_arc_preview', 'g1_open_10kmh_solo_gap05_arc_preview_bridge']


def test_target_above_base_is_reevaluated_only_with_the_option(monkeypatch):
    calls = []

    def reviewed(observed, path, cfg, speed, angle, previous, age, cap, target, clearance, remaining):
        calls.append(target)
        return target+.1, 'observed_turn_clear'
    monkeypatch.setattr(turn_preview, 'preview_speed', reviewed)
    obstacles = [1.75]*8
    off = drive(LocalFollow(FollowConfig(**CONFIG)), obstacles)[-1]
    assert off[0] > 1. and 'turn_preview' not in off[2] and not calls
    on = drive(LocalFollow(CFG), obstacles)[-1]
    assert calls and min(calls) > 1. and on[0] == pytest.approx(off[0]+.1) and on[1] == off[1]
    assert on[2]['constant_arc_target_mps'] == pytest.approx(off[0])


def test_close_obstacle_and_release_wait_are_not_reopened(monkeypatch):
    controller, states = LocalFollow(CFG), []

    def reviewed(observed, path, cfg, speed, angle, previous, age, cap, target, clearance, remaining):
        states.append((controller.held, target, clearance))
        return target, 'observed_turn_clear'
    monkeypatch.setattr(turn_preview, 'preview_speed', reviewed)
    obstacles = [1.75]*6+[.10]*2+[1.75]*6
    on = drive(controller, obstacles)
    off = drive(LocalFollow(FollowConfig(**CONFIG)), obstacles)
    assert [row[0] for row in on] == [row[0] for row in off]
    assert all(row[0] == 0 for row in on[6:8]) and on[8][2]['reason'] == 'follow_release_wait'
    assert states and all(not held and target > 0 and clearance >= CFG.resume_clearance_m for held, target, clearance in states)


def test_above_base_target_uses_the_same_ray_and_cap_checks():
    observed = open_scene()
    lifted, label = trial(observed)
    assert 1.2 < lifted <= 2.4 and label == 'observed_turn_clear'
    assert trial(observed, cap=1.25)[0] == 1.25
    assert trial(observed, remaining=1.3) == (1.2, 'observed_path_too_short')
    unknown = replace(observed, rays=tuple(replace(r, status='unknown', hit_xy_m=None) if r.source_index in {249, 250, 251} else r
                                           for r in observed.rays))
    assert trial(unknown) == (1.2, 'current_ray_support_missing')
    ranges = [1.35 if 245 <= i <= 255 else 6. for i in range(500)]
    assert trial(open_scene(ranges)) == (1.2, 'observed_hit_remains')


def test_prediction_hold_grows_with_the_requested_speed():
    # 취득·응답 동안의 조향 유지와 3 rad/s 제한이 속도에 맞춰 늘어난다. 고속 안전 보장은 아니다.
    angle = np.linspace(-.05, 1.6, 36)
    bend = np.column_stack((np.sin(angle), 1.-np.cos(angle)))
    slow = turn_preview.rollout(bend, 1.4, 0., 1., vars(CFG), 1., .1)
    fast = turn_preview.rollout(bend, 1.4, 0., 2.4, vars(CFG), 2.4, .1)
    turning = [path[np.flatnonzero(np.abs(path[:, 2]) > 1e-6)[0], 3] for path in (slow, fast)]
    assert turning[0] == pytest.approx(.20, abs=.04) and turning[1] == pytest.approx(.48, abs=.04)
