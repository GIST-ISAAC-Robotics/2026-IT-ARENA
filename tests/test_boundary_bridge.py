"""경로 근거 전환 때문에 생긴 정지만 잇는 단독 선택. 실제 점유·관측 정지는 그대로다."""
import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import LocalScene, SensorRay
from test_local_follow import history, scene
from test_low_speed_arc_reacquire import launch_module

CONFIG = dict(open_route_speed_mps=2.78, blend_open_speed_by_length=True, solo_route_only=True, corridor_fallback_enabled=True,
              boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')


def controllers():
    return LocalFollow(FollowConfig(**CONFIG)), LocalFollow(FollowConfig(**CONFIG, boundary_bridge=True))


def settle(controller, h):
    for now in (.4, .5, .6, .7, .8):
        result = controller.command(scene(now), h, 0., 'left')
    assert result[0] > 0
    return result


def short_scene(now, reach=.55, offset=0.):
    xs = np.linspace(-.25, reach, 60)
    points = np.vstack((np.column_stack((xs, xs*0+.425+offset)), np.column_stack((xs, xs*0-.425+offset))))
    first = now-.1
    rays = []
    for i, point in enumerate(points):
        d = point-np.array([.06, 0.])
        d /= np.linalg.norm(d)
        rays.append(SensorRay((.06, 0.), tuple(point), first, 'return', 'laser_frame', 0, i, tuple(d)))
    return LocalScene(now, first, first, 'laser_frame', 0, tuple(rays))


def test_default_is_off_and_requires_solo_corridor():
    assert FollowConfig().boundary_bridge is False
    for changes in ({'solo_route_only': False}, {'corridor_fallback_enabled': False}):
        with pytest.raises(ValueError):
            FollowConfig(**{**CONFIG, 'boundary_bridge': True, **changes})
    with pytest.raises(ValueError):
        FollowConfig(**CONFIG, boundary_bridge=1)


def test_profile_adds_only_this_option():
    profiles = launch_module().SPEED_PROFILES
    new = profiles['g1_open_10kmh_solo_gap05_arc_preview_bridge']
    assert new == {**profiles['g1_open_10kmh_solo_gap05_arc_preview'], 'g1_boundary_bridge': True}
    assert 'g1_ray_wall_continuity' not in new
    assert [k for k, v in profiles.items() if 'g1_boundary_bridge' in v] == ['g1_open_10kmh_solo_gap05_arc_preview_bridge']


def test_wall_side_change_keeps_the_observed_path_without_raising_its_cap():
    off, on = controllers()
    h = history()
    settle(off, h), settle(on, h)
    # 같은 스캔 중에 마커가 바뀐 경우. 새 후보가 없어 직전 경로를 통상 상한으로 잇는다.
    assert off.command(scene(.84, source=.8-.1), h, 1., 'right')[2]['reason'] == 'follow_boundary_unconfirmed'
    speed, _, detail = on.command(scene(.84, source=.8-.1), h, 1., 'right')
    assert speed > 0 and detail['path_wall'] == 'left' and detail['open_route_speed_enabled'] is False
    assert speed <= on.config.max_speed_mps
    # 새 벽은 다음 스캔에서 같은 기준으로 고른다. 직전 경로가 없으면 기존처럼 초기 확인을 거친다.
    assert on.command(scene(.9), h, 1., 'right')[2]['path_wall'] == 'right'
    fresh = LocalFollow(FollowConfig(**CONFIG, boundary_bridge=True))
    assert fresh.command(scene(.9), h, 0., 'right')[0] == 0


def test_visible_close_obstacle_still_holds_after_a_side_change():
    _, on = controllers()
    h = history()
    settle(on, h)
    speed, _, detail = on.command(scene(.84, obstacle_x=.10), h, 1., 'right')
    assert speed == 0 and detail['behavior'] == 'HOLD' and detail['reason'] != 'follow_boundary_unconfirmed'


def test_only_far_lever_of_a_long_path_is_excused():
    off, on = controllers()
    h = history(0., 1.8)
    long_path = np.column_stack((np.linspace(-.05, 2.75, 36), np.zeros(36)))
    for controller in (off, on):
        settle(controller, history())
        controller.path, controller.path_ref, controller.path_source = long_path, .8, .7
    assert off.command(scene(.92), h, 1., 'left')[2]['reason'] == 'follow_boundary_unconfirmed'
    assert on.command(scene(.92), h, 1., 'left')[2]['reason'] != 'follow_boundary_unconfirmed'
    # 1.8 m 안의 점이 크게 움직인 경우는 기존처럼 기억을 지운다.
    _, near = controllers()
    settle(near, history())
    near.path, near.path_ref, near.path_source = long_path[:24], .8, .7
    assert near.command(scene(.92), history(0., 9.), 1., 'left')[2]['reason'] == 'follow_boundary_unconfirmed'


def test_short_current_wall_must_agree_with_recent_path():
    off, on = controllers()
    h = history()
    settle(off, h), settle(on, h)
    for controller in (off, on):
        controller.command(short_scene(.9), h, 1., 'left'), controller.command(short_scene(1.), h, 1., 'left')
    assert off.command(short_scene(1.1), h, 1., 'left')[2]['reason'] == 'follow_boundary_unconfirmed'
    speed, _, detail = on.command(short_scene(1.1), h, 1., 'left')
    assert speed > 0 and detail['boundary_selection'] == 'short_current_history_agreement'
    assert detail['observed_path_ahead_m'] < 1.
    # 앵커 허용 안이어도 직전 경로와 겹치지 않으면 쓰지 않는다.
    _, moved = controllers()
    settle(moved, h)
    moved.command(short_scene(.9, offset=.11), h, 1., 'left'), moved.command(short_scene(1., offset=.11), h, 1., 'left')
    assert moved.command(short_scene(1.1, offset=.11), h, 1., 'left')[2]['reason'] == 'follow_boundary_unconfirmed'


def positive(controller):
    def evaluate(observed, *args, **kwargs):
        return dict(positive=True, reason='confirmed_shadow_candidate_only', fresh=True, proposed_speed_mps=.7, steering_rad=.1,
                    path=[[0., 0.], [1., 0.]], swept_clearance_m=1., confirmations=2, release_count=2,
                    source_first_s=observed.source_first_s, reference_s=observed.t_ref_s)
    controller.corridor_observer.evaluate = evaluate


@pytest.mark.parametrize('reason,bridged', [('follow_boundary_unconfirmed', True), ('follow_gap_hold', False),
                                            ('follow_same_scan_missing_wait', False), (None, False)])
def test_release_wait_is_bridged_only_after_a_path_evidence_hold(reason, bridged):
    _, on = controllers()
    h = history()
    settle(on, h)
    positive(on)
    on.held, on.hold_reason, on.release_count = True, reason, 0
    speed, _, detail = on.command(scene(.84), h, 1., 'left')
    assert detail['corridor_selected'] is bridged
    assert speed == (.7 if bridged else 0.)
    if bridged:
        assert detail['underlying_reason'] == 'follow_release_wait'
    else:
        assert detail['reason'] == 'follow_release_wait'


def test_option_off_keeps_release_wait_even_with_positive_corridor():
    off, _ = controllers()
    h = history()
    settle(off, h)
    positive(off)
    off.held, off.hold_reason, off.release_count = True, 'follow_boundary_unconfirmed', 0
    speed, _, detail = off.command(scene(.84), h, 1., 'left')
    assert speed == 0 and detail['reason'] == 'follow_release_wait'
