"""직각 경로의 기하·관측 범위와 옵션 OFF 재현성."""
import numpy as np
import pytest

from arena_autonomy.local_path import local_path
from arena_autonomy.piecewise_wall import corner_path
from arena_autonomy.local_follow import FollowConfig


def polyline(*vertices):
    return np.vstack([np.linspace(a, b, max(6, int(np.linalg.norm(np.array(b)-a)/.02)+1))[:-1]
                      for a, b in zip(vertices, vertices[1:])] + [np.array(vertices[-1:])])


@pytest.mark.parametrize('mirror', [1, -1])
def test_inner_outer_corner_agree_and_respect_steering(mirror):
    inner = polyline([-.4, .505], [.5, .505], [.5, 2.0])*[1, mirror]
    outer = polyline([-.4, -.505], [1.51, -.505], [1.51, 2.0])*[1, mirror]
    p = corner_path(inner, 'left' if mirror == 1 else 'right', .505, .347)
    q = corner_path(outer, 'right' if mirror == 1 else 'left', .505, .347)
    assert p is not None and q is not None
    assert np.max(np.linalg.norm(p-q, axis=1)) < .025
    assert np.min(np.diff(p[:, 0])) > -1e-9
    assert np.min(mirror*np.diff(p[:, 1])) > -1e-9
    arc = p[(p[:, 0] > .52) & (mirror*p[:, 1] < .48)]
    assert np.max(np.abs(np.linalg.norm(arc-[.5, mirror*.505], axis=1)-.505)) < .002
    assert corner_path(outer, 'right' if mirror == 1 else 'left', .505, .6) is None


def test_clipped_corner_does_not_append_reverse_segment():
    cloud = polyline([.8, -.505], [1.51, -.505], [1.51, .2])
    path = corner_path(cloud, 'right', .505, .347)
    assert path is not None
    assert np.min(np.diff(path[:, 0])) >= -1e-9
    assert np.min(np.diff(path[:, 1])) >= -1e-9
    assert path[-1, 1] < .505  # observed outgoing face ends before the arc tangent


@pytest.mark.parametrize('vertices', [
    [[-.4, -.505], [2., -.505]],
    [[-.4, -.505], [.8, -.505], [.8, -.42]],
    [[-.4, -.505], [.8, -.505], [.8, .5], [1.5, .5]],
])
def test_no_invented_corner_from_straight_short_or_multiple_faces(vertices):
    assert corner_path(polyline(*vertices), 'right', .505, .347) is None


def test_straight_and_default_output_unchanged_and_missing_wall_is_not_filled():
    cloud = polyline([-.4, -.505], [2., -.505])
    old, old_info = local_path(cloud, 'right', .505)
    trial, info = local_path(cloud, 'right', .505, piecewise_corner=True, minimum_radius=.347)
    np.testing.assert_array_equal(old, trial)
    # r23: 선택형 호출의 일반 곡선 반환에는 앵커 진단 path_anchor_error_m 하나만 추가된다.
    assert 'path_anchor_error_m' not in old_info
    assert old_info == {k: v for k, v in info.items() if k != 'path_anchor_error_m'}
    assert local_path(cloud, 'left', .505, piecewise_corner=True)[0] is None
    assert FollowConfig().piecewise_wall_trial is False
    with pytest.raises(ValueError, match='boolean'):
        FollowConfig(piecewise_wall_trial=1)


def path_ahead(path):
    arc = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    return arc[-1]-arc[np.argmin(np.linalg.norm(path, axis=1))]


def segment_distance(path, points):
    a, d = path[:-1], np.diff(path, axis=0)
    t = np.clip(np.sum((points[:, None]-a[None])*d[None], axis=2)/np.sum(d*d, axis=1), 0., 1.)
    return np.min(np.linalg.norm(a[None]+t[:, :, None]*d[None]-points[:, None], axis=2), axis=1)


def outer_corner(mirror, *outgoing):
    """안쪽 벽은 x=0.1에서 끝나고 바깥 벽이 x=1.1에서 꺾인다(교통 한 랩 정지 장면의 축소형)."""
    outer = polyline([-.4, -.505], [1.1, -.505], *outgoing)
    return np.vstack([outer, polyline([-.4, .505], [.1, .505])])*[1, mirror]


@pytest.mark.parametrize('mirror', [1, -1])
def test_outer_corner_reads_more_of_same_wall_until_path_budget(mirror):
    side = 'right' if mirror == 1 else 'left'
    cloud = outer_corner(mirror, [1.1, 2.0])
    frozen = cloud.copy()
    path, info = local_path(cloud, side, .505, piecewise_corner=True, minimum_radius=.347)
    np.testing.assert_array_equal(cloud, frozen)
    budget = info['corner_path_budget']
    # 벽 1.8 m를 읽은 원 후보는 오프셋 경로가 경계 길이 0.65 m보다 짧았다.
    assert budget['wall_ahead_before_m'] == pytest.approx(1.8) and budget['path_ahead_before_m'] < .65
    assert path_ahead(path) == pytest.approx(budget['path_ahead_m']) and path_ahead(path) >= 1.8
    assert info['boundary_observed_ahead_m'] > 1.8 and info['wall_model'] == 'piecewise_corner'
    # 원점 부근과 코너 반경은 그대로다. 실제 바깥 벽에서 오프셋 거리를 지킨다.
    assert np.max(np.abs(path[path[:, 0] < .09, 1])) < 1e-3
    arc = path[(path[:, 0] > .1) & (mirror*path[:, 1] < .5)]
    assert np.max(np.abs(np.linalg.norm(arc-[.09, mirror*.505], axis=1)-.505)) < .002
    assert mirror*path[-1, 1] <= 2.0 + 1e-9 and np.all(path[:, 0] <= .595 + 1e-9)
    # 관측이 1.8 m에서 끝나는 같은 벽의 원 후보를 새 경로가 벽 적합 허용량 안에서 포함한다.
    original, original_info = local_path(outer_corner(mirror, [1.1, .195]), side, .505,
                                         piecewise_corner=True, minimum_radius=.347)
    assert 'corner_path_budget' not in original_info and path_ahead(original) == pytest.approx(budget['path_ahead_before_m'])
    assert np.max(segment_distance(path, original)) <= .03
    off, off_info = local_path(cloud, side, .505)
    assert 'corner_path_budget' not in off_info and 'wall_model' not in off_info


@pytest.mark.parametrize('mirror', [1, -1])
def test_short_observed_wall_end_is_not_extrapolated(mirror):
    side = 'right' if mirror == 1 else 'left'
    path, info = local_path(outer_corner(mirror, [1.1, .395]), side, .505, piecewise_corner=True, minimum_radius=.347)
    # 같은 연결 구간의 실제 끝(앵커부터 벽 2.0 m)까지만 읽는다. 요청 1.8 m 경로를 채우지 않는다.
    assert info['boundary_observed_ahead_m'] == pytest.approx(2.0, abs=.03)
    assert path_ahead(path) < 1.0 and mirror*path[-1, 1] < .395+.02


@pytest.mark.parametrize('mirror', [1, -1])
def test_separate_far_wall_and_second_corner_keep_original_candidate(mirror):
    side = 'right' if mirror == 1 else 'left'
    kwargs = dict(piecewise_corner=True, minimum_radius=.347)
    # 관측 끝에서 끊긴 먼 벽 조각은 다른 구간이다. 연장하거나 붙이지 않는다.
    alone, alone_info = local_path(outer_corner(mirror, [1.1, .395]), side, .505, **kwargs)
    far = np.vstack([outer_corner(mirror, [1.1, .395]), polyline([1.1, .9], [1.1, 1.6])*[1, mirror]])
    path, info = local_path(far, side, .505, **kwargs)
    np.testing.assert_array_equal(path, alone)
    assert info == alone_info
    # 추가 창에 두 번째 모서리가 들어오면 원 후보(벽 1.8 m, 짧은 경로)를 그대로 둔다.
    second, second_info = local_path(outer_corner(mirror, [1.1, .3], [.5, .3]), side, .505, **kwargs)
    assert 'corner_path_budget' not in second_info and second_info['wall_model'] == 'piecewise_corner'
    assert second_info['boundary_observed_ahead_m'] == pytest.approx(1.8) and path_ahead(second) < .65
