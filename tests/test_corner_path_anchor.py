"""한 모서리 경로의 앵커 오차: 원호 위 차량은 벽 y좌표 대신 생성 경로까지의 거리로 판정한다."""
import dataclasses
import importlib.util
import math
import zipfile
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig, anchor_error
from arena_autonomy.local_path import local_path

ROOT = Path(__file__).resolve().parents[1]
BEFORE = ROOT/'artifacts/validation/2026-11-08/corner_path_anchor_v1/before.zip'
OFFSET, RADIUS = .505, .347
ON = dataclasses.replace(FollowConfig(), piecewise_wall_trial=True, wall_offset_m=OFFSET)
OFF = dataclasses.replace(ON, piecewise_wall_trial=False)


def polyline(*vertices):
    return np.vstack([np.linspace(a, b, max(6, int(np.linalg.norm(np.array(b)-a)/.02)+1))[:-1]
                      for a, b in zip(vertices, vertices[1:])] + [np.array(vertices[-1:])])


def corner_scan(mirror, degrees, lateral=0.):
    """바깥 직각 벽을 원호 경로 중간의 차량 좌표로 옮긴다. lateral은 원호 법선 방향 이탈(+는 바깥)."""
    wall = polyline([-1., -.505], [1.51, -.505], [1.51, 3.])*[1, mirror]
    a = math.radians(degrees)
    center = np.array([.5, .505*mirror])  # 오프셋 0.505 경로의 원호 중심. 반경도 0.505다.
    radial = np.array([math.sin(a), -mirror*math.cos(a)])
    pose = center+(OFFSET+lateral)*radial
    h = mirror*a
    rotation = np.array([[math.cos(h), math.sin(h)], [-math.sin(h), math.cos(h)]])
    return (wall-pose)@rotation.T, pose, center


def old_local_path():
    spec = importlib.util.spec_from_loader('before_local_path', loader=None)
    module = importlib.util.module_from_spec(spec)
    with zipfile.ZipFile(BEFORE) as archive:
        exec(archive.read('src/arena_autonomy/arena_autonomy/local_path.py').decode('utf-8'), module.__dict__)
    return module.local_path


@pytest.mark.parametrize('mirror', [1, -1])
@pytest.mark.parametrize('degrees', [35, 40])
def test_vehicle_on_arc_passes_anchor_although_wall_y_exceeds_offset(mirror, degrees):
    side = 'right' if mirror == 1 else 'left'
    cloud, pose, center = corner_scan(mirror, degrees)
    raw = cloud.copy()
    path, info = local_path(cloud, side, OFFSET, piecewise_corner=True, minimum_radius=RADIUS)
    np.testing.assert_array_equal(cloud, raw)  # 측정 벡터를 바꾸지 않는다.
    assert info['wall_model'] == 'piecewise_corner'
    # 원 검사는 벽 y좌표만 본다. 원호 위에서는 오프셋+0.12를 넘는다.
    assert info['wall_distance_m'] >= OFFSET+ON.boundary_anchor_error_m
    assert anchor_error(info, OFF) == abs(info['wall_distance_m']-OFFSET) > ON.boundary_anchor_error_m
    # 실제 경로(회전된 직선+원호)와 원점의 거리. 세계 좌표 원호와 독립 대조한다.
    assert abs(np.linalg.norm(pose-center)-OFFSET) < 1e-9
    assert info['path_anchor_error_m'] < .01
    assert anchor_error(info, ON) == info['path_anchor_error_m'] <= ON.boundary_anchor_error_m
    assert info['boundary_observed_ahead_m'] >= ON.boundary_min_ahead_m
    # 경로·벽 거리·관측 길이 등 기존 출력은 수정 전 코드와 같고 진단 하나만 추가된다.
    old_path, old_info = old_local_path()(cloud, side, OFFSET, piecewise_corner=True, minimum_radius=RADIUS)
    np.testing.assert_array_equal(old_path, path)
    assert old_info == {k: v for k, v in info.items() if k != 'path_anchor_error_m'}


@pytest.mark.parametrize('mirror', [1, -1])
@pytest.mark.parametrize('lateral', [-.2, .2])
def test_vehicle_off_the_generated_path_is_rejected(mirror, lateral):
    side = 'right' if mirror == 1 else 'left'
    cloud, pose, center = corner_scan(mirror, 35, lateral)
    path, info = local_path(cloud, side, OFFSET, piecewise_corner=True, minimum_radius=RADIUS)
    assert info['wall_model'] == 'piecewise_corner'
    expected = abs(np.linalg.norm(pose-center)-OFFSET)  # 원호 구간 안이므로 원호까지의 거리
    assert abs(info['path_anchor_error_m']-expected) < .01
    assert anchor_error(info, ON) > ON.boundary_anchor_error_m


def test_straight_keeps_path_and_default_off_outputs_have_no_new_field():
    straight = polyline([-.4, -.505], [2., -.505])
    old, old_info = local_path(straight, 'right', OFFSET)
    trial, info = local_path(straight, 'right', OFFSET, piecewise_corner=True, minimum_radius=RADIUS)
    np.testing.assert_array_equal(old, trial)
    # 선택형 호출의 일반 곡선 반환에는 진단 하나만 추가되고 나머지는 기본 출력과 같다(r23).
    assert old_info == {k: v for k, v in info.items() if k != 'path_anchor_error_m'}
    assert 'path_anchor_error_m' not in old_info and 'wall_model' not in info
    assert info['path_anchor_error_m'] == pytest.approx(0., abs=1e-6)
    cloud, _, _ = corner_scan(1, 35)
    _, off_info = local_path(cloud, 'right', OFFSET)
    assert 'path_anchor_error_m' not in off_info and 'wall_model' not in off_info
    assert FollowConfig().piecewise_wall_trial is False
    assert FollowConfig().boundary_anchor_error_m == .12


@pytest.mark.parametrize('value', [math.nan, math.inf, -.01, None, 'x'])
def test_invalid_path_error_is_rejected_not_passed(value):
    info = {'wall_model': 'piecewise_corner', 'wall_distance_m': OFFSET, 'path_anchor_error_m': value}
    assert anchor_error(info, ON) == math.inf
    assert anchor_error(info, OFF) == 0.  # 기본 OFF 설정은 원 검사


@pytest.mark.parametrize('info', [
    {'wall_model': 'piecewise_corner', 'wall_distance_m': .70},  # 옛 기록/스텁: 새 필드 없음
    {'wall_distance_m': .70},  # 곡선 fallback의 옛 기록/스텁: 새 필드 없음
    {'wall_model': 'other', 'wall_distance_m': .70, 'path_anchor_error_m': 0.},
])
def test_other_models_and_missing_field_keep_wall_distance_check(info):
    assert anchor_error(info, ON) == pytest.approx(.195)
    assert anchor_error(info, OFF) == pytest.approx(.195)


def test_curve_fallback_with_field_uses_path_error_only_when_trial_on():
    # r23: 곡선 fallback(wall_model 없음)은 선택형 호출에서 같은 진단을 낸다. r22에서는 벽 거리로 돌아갔다.
    info = {'wall_distance_m': .70, 'path_anchor_error_m': 0.}
    assert anchor_error(info, ON) == 0.
    assert anchor_error(info, OFF) == pytest.approx(.195)
    for value in (math.nan, -.01, 'x'):
        assert anchor_error({**info, 'path_anchor_error_m': value}, ON) == math.inf


def segment_distance(path):
    """반환 경로 선분과 원점의 최소 거리. local_path 구현과 독립으로 시험 안에서 계산한다."""
    best = math.inf
    for a, b in zip(path[:-1], path[1:]):
        d = b-a
        t = min(1., max(0., -float(a@d)/max(1e-12, float(d@d))))
        best = min(best, float(np.linalg.norm(a+t*d)))
    return best


def approach_scan(mirror, degrees, lateral=0., outgoing=True):
    """직각 코너 앞 직선 구간(x=-0.3)의 차량이 벽에 대해 degrees만큼 회전한 장면.

    lateral은 오프셋 경로(y=0)에서 벽 쪽(-)/반대(+) 이탈. outgoing=False면 바깥 면을 관측하지 못한
    같은 벽이다. 같은 실제 경로가 한 모서리 표현과 일반 곡선 표현으로 바뀌는 조건을 만든다.
    """
    wall = polyline([-1., -.505], [1.51, -.505], [1.51, 3.])
    if not outgoing:
        wall = wall[wall[:, 0] <= 1.]
    wall = wall*[1, mirror]
    h = mirror*math.radians(degrees)
    rotation = np.array([[math.cos(h), math.sin(h)], [-math.sin(h), math.cos(h)]])
    return (wall-[-.3, mirror*lateral])@rotation.T


@pytest.mark.parametrize('mirror', [1, -1])
@pytest.mark.parametrize('lateral', [0., -.2, .2])
def test_same_path_keeps_anchor_metric_when_representation_switches(mirror, lateral):
    side = 'right' if mirror == 1 else 'left'
    corner = local_path(approach_scan(mirror, 30, lateral), side, OFFSET, piecewise_corner=True, minimum_radius=RADIUS)[1]
    cloud = approach_scan(mirror, 30, lateral, outgoing=False)
    path, info = local_path(cloud, side, OFFSET, piecewise_corner=True, minimum_radius=RADIUS)
    assert corner['wall_model'] == 'piecewise_corner' and 'wall_model' not in info  # 표현 전환
    for details in (corner, info):
        # 원점은 실제 오프셋 직선에서 |lateral| 떨어져 있다. 표현과 무관하게 같은 값이어야 한다.
        assert details['path_anchor_error_m'] == pytest.approx(abs(lateral), abs=.01)
        assert (anchor_error(details, ON) <= ON.boundary_anchor_error_m) is (lateral == 0.)
    assert info['path_anchor_error_m'] == pytest.approx(segment_distance(path), abs=1e-9)
    # 일반 곡선 경로 배열과 나머지 info는 기본 OFF 호출 및 수정 전 선택형 호출과 같다.
    off_path, off_info = local_path(cloud, side, OFFSET)
    old_path, old_info = old_local_path()(cloud, side, OFFSET, piecewise_corner=True, minimum_radius=RADIUS)
    np.testing.assert_array_equal(path, off_path)
    np.testing.assert_array_equal(path, old_path)
    assert off_info == old_info == {k: v for k, v in info.items() if k != 'path_anchor_error_m'}


@pytest.mark.parametrize('mirror', [1, -1])
def test_arc_vehicle_with_short_outgoing_face_uses_returned_curve_distance(mirror):
    # r22 원호 35° 장면에서 바깥 면이 0.3 m만 보이면 corner_path가 거부하고 일반 곡선이 반환된다.
    side = 'right' if mirror == 1 else 'left'
    wall = polyline([-1., -.505], [1.51, -.505], [1.51, -.205])*[1, mirror]
    a, h = math.radians(35), mirror*math.radians(35)
    pose = np.array([.5, .505*mirror])+OFFSET*np.array([math.sin(a), -mirror*math.cos(a)])
    cloud = (wall-pose)@np.array([[math.cos(h), math.sin(h)], [-math.sin(h), math.cos(h)]]).T
    path, info = local_path(cloud, side, OFFSET, piecewise_corner=True, minimum_radius=RADIUS)
    assert info['reason'] == 'local_path' and 'wall_model' not in info
    assert anchor_error(info, OFF) > ON.boundary_anchor_error_m  # r22에서 이 표현은 벽 y거리로 거부됐다.
    # 반환 곡선은 실제 원호보다 모서리를 완만하게 깎으므로 거리 0이 아니다. 그 곡선까지의 거리만 쓴다.
    assert info['path_anchor_error_m'] == pytest.approx(segment_distance(path), abs=1e-9)
    assert anchor_error(info, ON) == info['path_anchor_error_m'] <= ON.boundary_anchor_error_m
    np.testing.assert_array_equal(path, local_path(cloud, side, OFFSET)[0])
