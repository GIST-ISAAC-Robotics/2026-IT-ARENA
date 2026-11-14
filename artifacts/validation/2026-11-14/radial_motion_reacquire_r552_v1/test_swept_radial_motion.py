"""취득 후 운동 반경의 기하 계약. 실차·시간 경계·주행 안전 증명이 아니다."""
import math

import numpy as np
import pytest

from arena_autonomy.local_path import swept_limit


def legacy_limit(points, steering, wheelbase=.145, horizon=3., margin=.025, length=.20, width=.15):
    """선택 인자 추가 전 판정을 순차 표본으로 고정한 독립 대조."""
    points = np.asarray(points, dtype=float)
    curvature = math.tan(steering) / wheelbase
    for distance in np.arange(0., horizon + .001, .04):
        yaw = curvature * distance
        c, s = math.cos(yaw), math.sin(yaw)
        if abs(curvature) < 1e-5:
            rx, ry = distance - wheelbase / 2, 0.
        else:
            rx, ry = s / curvature - wheelbase / 2, (1 - c) / curvature
        dx, dy = points[:, 0] - rx, points[:, 1] - ry
        local_x, local_y = c * dx + s * dy - wheelbase / 2, -s * dx + c * dy
        if np.any((np.abs(local_x) <= length/2 + margin) & (np.abs(local_y) <= width/2 + margin)):
            return max(0., float(distance) - .04)
    return horizon


@pytest.mark.parametrize('steering', [0., 1e-8, .2, -.2])
def test_omitted_none_and_zero_keep_exact_fixed_margin_sweep(steering):
    clouds = [np.empty((0, 2)), np.array([[.325, .1], [.50, .11], [1., -.08]]),
              np.array([[0., .1], [-.125, -.1]]),
              np.random.default_rng(552).uniform([.2, -.8], [2., .8], (40, 2))]
    for points in clouds:
        for horizon in [.0, .03, .04, .39, 1.2, 3.]:
            options = dict(horizon=horizon)
            expected = legacy_limit(points, steering, **options)
            assert swept_limit(points, steering, **options) == expected
            assert swept_limit(points, steering, radial_margin=None, **options) == expected
            assert swept_limit(points, steering, radial_margin=0., **options) == expected
            assert swept_limit(points, steering, radial_margin=np.zeros(len(points)), **options) == expected


@pytest.mark.parametrize('point,radius', [([0., .15], .05), ([.175, 0.], .05),
                                        ([0., -.15], .05), ([.05, .02], .0)])
def test_side_front_and_actual_body_points_still_stop(point, radius):
    assert swept_limit([point], 0., horizon=.03, radial_margin=radius) == 0.


def test_only_square_corner_overlap_is_not_current_interference():
    # 고정 사각형 (.125,.100)에서 각 축 .04m 밖: 각축 .05m 팽창은 겹치지만
    # 실제 운동 상계 .05m의 원판으로는 sqrt(.04²+.04²) 위치까지 도달하지 못한다.
    point = [[.165, .140]]
    assert swept_limit(point, 0., horizon=.03, margin=.025 + .05) == 0.
    # 4cm 전진 뒤에는 전면 바로 위의 실제 원판 간섭이 생긴다. 표본/차감은 유지한다.
    assert swept_limit(point, 0., horizon=.03, radial_margin=.05) == .03
    assert swept_limit(point, 0., horizon=.08, radial_margin=.05) == 0.


def test_disk_corner_inside_or_tangent_still_stops():
    # 이진수로 정확한 3:4:5 경계로 반올림 오차를 시험 가정에 섞지 않는다.
    assert swept_limit([[.21875, .25]], 0., horizon=.03, length=.25, width=.25,
                       margin=0., radial_margin=.15625) == 0.
    assert swept_limit([[.155, .130]], 0., horizon=.03, radial_margin=.05) == 0.


def test_first_interference_keeps_four_cm_sampling_and_subtraction():
    # 첫 hit는 .28m 표본(.40-.28=.12 <= .125), 반환은 .24m다.
    assert swept_limit([[.4, 0.]], 0., radial_margin=0.) == float(np.arange(0., 3.001, .04)[7]) - .04
    # 정면 운동 반경 .05m를 추가하면 첫 hit .24m, 반환 .20m다.
    assert swept_limit([[.4, 0.]], 0., radial_margin=.05) == float(np.arange(0., 3.001, .04)[6]) - .04


def test_per_ray_radius_stays_bound_to_its_point_without_dropping_points():
    points = np.array([[0., .16], [0., .20]])
    radii = np.array([.02, .08])
    assert swept_limit(points, 0., horizon=.2, radial_margin=radii) == .2
    assert swept_limit(points, 0., horizon=.2, radial_margin=radii[::-1]) == 0.
    assert swept_limit(points[::-1], 0., horizon=.2, radial_margin=radii[::-1]) == .2
    original = points.copy()
    assert swept_limit(np.vstack([points, [0., 0.]]), 0., radial_margin=[.02, .08, 0.]) == 0.
    assert np.array_equal(points, original)


@pytest.mark.parametrize('radius', [True, np.bool_(False), [True, .1], [False, 0],
    -.01, [0., -.01], float('nan'), float('inf'), [0., float('nan')],
    [.1], [[.1, .1]], [[.1], [.1]], [0., 0., 0.], '0.1', [.1, '0.1'],
    1j, {'radius': .1}, [[0.], [.1, .2]]])
def test_invalid_radius_rejected_before_any_sweep(radius):
    with pytest.raises(ValueError):
        swept_limit([[0., 0.], [1., 1.]], 0., horizon=0., radial_margin=radius)


@pytest.mark.parametrize('radius', [0, np.float64(.05), np.array(.05), [.05, .05], np.array([0, 1])])
def test_valid_numeric_scalar_and_matching_vector(radius):
    assert swept_limit([[0., 0.], [1., 1.]], 0., radial_margin=radius) == 0.
