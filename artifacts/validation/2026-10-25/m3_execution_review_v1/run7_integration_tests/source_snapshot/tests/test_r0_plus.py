"""R0+ 운영 미연결 순수 비교. 임의 점 단위 검사는 폐루프 주행 증거가 아니다."""
from dataclasses import replace
import math

import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import pursuit_command
from arena_autonomy.r0_plus import R0Plus
from test_local_follow import scene, history, settle


def test_clean_corridor_retains_r0_command_and_all_occupancy():
    result = settle(R0Plus(), history())
    raw = pursuit_command(scene(.7).occupied_endpoints_xy_m, 'left', 0., wheelbase=.135, max_speed=1.)
    assert result[:2] == pytest.approx(raw[:2])
    assert result[2]['behavior'] == 'CRUISE'
    assert result[2]['occupied_points'] == 280 and result[2]['unfiltered_occupancy']
    assert result[2]['boundary_from_memory'] is False


def test_same_scan_does_not_release():
    ctl = R0Plus()
    for now in (.4, .42, .44, .46):
        assert ctl.command(scene(now, source=.3), None, 0., 'left')[0] == 0.
    assert ctl.release_count == 1


@pytest.mark.parametrize('kind', ['epoch', 'side', 'clock', 'source', 'reset'])
def test_reset_and_discontinuities_require_new_confirmations(kind):
    ctl = R0Plus()
    assert settle(ctl, history())[0] > 0
    s, side = scene(.8), 'left'
    if kind == 'epoch': s = scene(.8, epoch=1)
    if kind == 'side': side = 'right'
    if kind == 'clock': s = scene(.6)
    if kind == 'source': s = scene(.8, source=.59)
    if kind == 'reset': ctl.reset()
    assert ctl.command(s, None, 0., side)[0] == 0.


def test_short_side_surface_cannot_steer_away_from_corridor():
    output = settle(R0Plus(), history(), side_car=True)
    # R0 원래 구간 조건이 짧은 자동차 면을 거절하거나 양측 검사가 HOLD해야 한다.
    assert output[0] == 0 or abs(output[1]) < 1e-8


def test_long_false_side_surface_is_not_corroborated():
    # 왼쪽 면만 .425 -> .22로 옮겨 R0가 그것을 경계로 쓸 수 있게 만든다.
    original = scene(.5)
    from arena_autonomy.local_scene import SensorRay, LocalScene
    rays = []
    for r in original.rays:
        x, y = r.hit_xy_m
        y = .22 if y > 0 else y
        dx, dy = x-.06, y
        norm = math.hypot(dx, dy)
        rays.append(SensorRay(r.ray_origin_xy_m, (x, y), r.acquired_at_s, r.status,
            r.source_frame_id, r.source_epoch, r.source_index, (dx/norm, dy/norm)))
    s = LocalScene(.5, .4, .4, 'laser_frame', 0, tuple(rays))
    result = R0Plus().command(s, None, 0., 'left')
    assert result[0] == 0 and result[2]['reason'] == 'r0plus_bilateral_unconfirmed'


def test_missing_one_wall_does_not_use_memory_or_unilateral_fallback():
    ctl = R0Plus()
    assert settle(ctl, history())[0] > 0
    result = ctl.command(scene(.8, missing_wall=True), None, 0., 'left')
    assert result[0] == 0 and result[2]['reason'] == 'r0plus_bilateral_unconfirmed'


def test_close_front_object_is_kept_and_holds_then_releases():
    ctl = R0Plus()
    settle(ctl, history())
    close = ctl.command(scene(.8, obstacle_x=.25), None, .1, 'left')
    assert close[0] == 0 and close[2]['reason'] == 'r0plus_gap_hold'
    assert close[2]['occupied_points'] == 300
    assert ctl.command(scene(.9), None, 0., 'left')[0] == 0
    assert ctl.command(scene(1.), None, 0., 'left')[0] > 0


def test_reverse_stale_unknown_stop():
    ctl = R0Plus()
    settle(ctl, history())
    assert ctl.command(scene(.8), None, -.01, 'left')[2]['reason'] == 'r0plus_reverse_motion'
    assert ctl.command(scene(1., source=.4), None, 0., 'left')[2]['reason'] == 'r0plus_scene_stale'
    s = scene(1.)
    r = next(r for r in s.rays if abs(math.atan2(r.direction_xy[1], r.direction_xy[0])) < 1.)
    missing = replace(r, status='unknown', hit_xy_m=None)
    s = replace(s, rays=tuple(missing if q is r else q for q in s.rays))
    assert ctl.command(s, None, 0., 'left')[2]['reason'] == 'r0plus_front_unobserved'


@pytest.mark.parametrize('kwargs', [{'max_speed_mps': 1.1}, {'occupancy_timing_mode': 'oldest'},
                                  {'boundary_evidence_mode': 'fragmented_candidate'}])
def test_screen_does_not_enable_g1_options_or_higher_speed(kwargs):
    with pytest.raises(ValueError):
        R0Plus(replace(FollowConfig(occupancy_timing_mode='per_ray_candidate'), **kwargs))
