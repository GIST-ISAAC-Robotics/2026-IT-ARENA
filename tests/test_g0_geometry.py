import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from g0_geometry import Footprint, TruthBox, cross_section_gaps, rectangle_section_y, ray_box_distance, ray_direction, cast_rays


@pytest.mark.parametrize('length,width', [(0., .15), (.2, -1.), (math.inf, .15), (.2, math.nan)])
def test_invalid_dimensions(length, width):
    with pytest.raises(ValueError):
        Footprint(length, width)


def test_central_zero_clearance_is_not_possible_and_edge_has_positive_margin():
    target = TruthBox(1., 0., .12)
    touching = cross_section_gaps(target, padding_m=0.)
    assert touching['left_residual_m'] == pytest.approx(0.)
    assert not touching['cross_section_possible']
    result = cross_section_gaps(target)
    assert result['left_gap_m'] == pytest.approx(.15)
    assert result['left_residual_m'] == pytest.approx(-.05)
    assert not result['cross_section_possible']
    edge = cross_section_gaps(TruthBox(1., -.15, .12))
    assert edge['left_gap_m'] == pytest.approx(.30)
    assert edge['left_residual_m'] == pytest.approx(.10)
    assert edge['cross_section_possible'] and edge['target_fully_inside_road']


def test_signed_yaw_envelope_and_real_section():
    footprint = Footprint()
    for yaw in (math.pi/6, -math.pi/6):
        assert footprint.lateral_width(yaw) == pytest.approx(.15*math.sqrt(3)/2+.1)
    polygon = footprint.polygon(1., 0., math.pi/2)
    assert rectangle_section_y(polygon, 1.) == pytest.approx((-.1, .1))
    assert rectangle_section_y(polygon, 3.) is None
    result = cross_section_gaps(TruthBox(1., -.15, .12), ego_yaw_rad=math.pi/2)
    assert result['required_gap_m'] == pytest.approx(.25)
    assert result['left_residual_m'] == pytest.approx(.05)
    outside = cross_section_gaps(TruthBox(1., -.35, .12))
    assert outside['target_completely_outside_road'] and not outside['target_fully_inside_road']


def test_low_height_pitch_and_nearest_surface_range():
    origin = (.06, 0., .095)
    target = TruthBox(1., 0., .12)
    low = TruthBox(1., 0., .06)
    assert ray_box_distance(origin, (1., 0., 0.), target) == pytest.approx(.84)
    assert ray_box_distance(origin, (1., 0., 0.), low) is None
    assert ray_box_distance(origin, ray_direction(0., -.05), low) is not None
    assert ray_box_distance(origin, ray_direction(0., .1), target) is None
    near = TruthBox(.07, 0., .12, Footprint(.01, .15))
    assert ray_box_distance(origin, (1., 0., 0.), near) is None


def test_rear_mask_is_per_ray_and_seam_has_two_source_sides():
    # 폭이 큰 후방 표적은 가린 중앙점 외에 양옆 외곽이 보일 수 있다.
    rear = cast_rays([TruthBox(-.5, 0., .12)])
    indices = [i for i, label in enumerate(rear['truth_labels']) if label == 'target']
    assert 0 in indices and 499 in indices
    assert all(rear['masked'][i] for i in indices)
    wide = cast_rays([TruthBox(-.1, 0., .12, Footprint(.10, .30))])
    hits = [i for i, label in enumerate(wide['truth_labels']) if label == 'target']
    assert any(wide['masked'][i] for i in hits)
    assert any(not wide['masked'][i] for i in hits)
    # .06 m 센서 원점이 있어 sensor→target의 상대 x는 -.16 m이다.
    assert ray_box_distance((.06, 0., .095), (-1., 0., 0.), TruthBox(-.1, 0., .12, Footprint(.10, .30))) == pytest.approx(.11)


def test_pitched_scan_is_plane_and_near_occluder_does_not_reveal_wall():
    pitch = .2
    front, back, left = (ray_direction(a, pitch) for a in (0., math.pi, math.pi/2))
    assert front[2] == pytest.approx(-back[2])
    assert left[2] == pytest.approx(0., abs=1e-12)
    assert left[1] == pytest.approx(1.)
    assert sum(v*v for v in front) == pytest.approx(1.)
    near = TruthBox(.08, 0., .12, Footprint(.01, .15), label='near')
    far = TruthBox(1., 0., .30, label='wall')
    rays = cast_rays([near, far], samples=500)
    assert math.isnan(rays['ranges'][250])
    assert rays['truth_labels'][250] == 'near'

