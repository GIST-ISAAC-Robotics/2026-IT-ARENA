"""얇은 후반부 결측 후보의 반례. 허용은 자유 공간 관측을 뜻하지 않는다."""
import math
import numpy as np
import pytest

from arena_autonomy.rear_fringe_trial import bounded_rear_fringe
from arena_autonomy.scene_motion import motion_scene
from test_local_follow import scan, history


def case(*, missing=(451, 452), value=math.inf, neighbor=4., point=(-.047, .0753), mismatch=False):
    ranges = [4.]*500
    for i in missing:
        ranges[i] = value
    ranges[min(missing)-1] = neighbor
    raw = scan(.5, ranges)
    scene = motion_scene(raw, history(), .6, timing_verified=True)
    if mismatch:
        raw.header.stamp.nanosec += 1000000
    result = bounded_rear_fringe(scene, raw, [point], np.full(500, .10), .20, .15)
    return result, raw, scene


def test_narrow_far_bounded_rear_strip_is_explicit_risk_not_free():
    out, raw, scene = case()
    assert out['accepted'] and not out['unknown_filled'] and out['residual_hidden_object_risk']
    assert out['exterior_max_m'] == pytest.approx(.0003)
    assert out['gaps'][0]['first'] == 451 and out['gaps'][0]['last'] == 452
    assert np.isposinf(raw.ranges[451]) and scene.rays[451].status == 'unknown'
    assert scene.rays[451].hit_xy_m is None


@pytest.mark.parametrize('point', [(0.001, .0753), (-.047, .076001), (-.101, .0753), (-.047, .075)])
def test_larger_forward_and_already_occupied_body_are_not_this_candidate(point):
    assert case(point=point)[0]['reason'] == 'outside_rear_fringe'


@pytest.mark.parametrize('value', [math.nan, -math.inf, .02, -1.])
def test_invalid_near_data_is_not_far_nonreturn(value):
    out, _, _ = case(value=value)
    assert not out['accepted'] and out['reason'] == 'invalid_near_or_nonfinite_return'


@pytest.mark.parametrize('neighbor', [.19, .4])
def test_near_return_or_known_occupied_margin_remains_blocking(neighbor):
    out, _, _ = case(neighbor=neighbor)
    assert not out['accepted'] and out['reason'] in ('known_occupied_padding', 'near_gap_neighbor')


def test_wide_gap_and_mask_boundary_are_not_small_nonreturn():
    assert not case(missing=tuple(range(449, 455)))[0]['accepted']
    assert not case(missing=tuple(range(451, 460)))[0]['accepted']


def test_raw_input_must_match_current_scene():
    assert case(mismatch=True)[0]['reason'] == 'raw_scene_mismatch'
    _, _, scene = case()
    assert not bounded_rear_fringe(scene, None, [[-.047, .0753]], np.full(500, .1), .2, .15)['accepted']


def test_reflected_geometry_has_the_same_limit():
    out, _, _ = case(missing=(48, 49), point=(-.047, -.0753))
    assert out['accepted'] and not out['unknown_filled']
