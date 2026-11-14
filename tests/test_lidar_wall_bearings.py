"""실제 광선 방향 정렬의 작은 기하/연결 계약. 주행·자유 공간 증명은 아니다."""
import math

import numpy as np

import arena_autonomy.local_follow as follow
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_path import break_adjacent_occluders, local_path
from arena_autonomy.local_scene import LocalScene, SensorRay


TWO_DEPTH_WALLS = [(-1., .1, .36, .508), (.4, 2.2, 3., 2.2)]


def cast(segments, origins):
    """실제 선분과 광선의 첫 교차만 반환한다. 교차 없는 광선은 점을 만들지 않는다."""
    angles = -math.pi + np.arange(500)*math.tau/500
    directions = np.column_stack((np.cos(angles), np.sin(angles)))
    origins = np.broadcast_to(origins, (500, 2))
    best = np.full(500, np.inf)
    for x0, y0, x1, y1 in segments:
        edge = np.array([x1-x0, y1-y0])
        offset = np.array([x0, y0])-origins
        denominator = directions[:, 0]*edge[1]-directions[:, 1]*edge[0]
        parallel = np.abs(denominator) < 1e-12
        safe = np.where(parallel, 1., denominator)
        distance = (offset[:, 0]*edge[1]-offset[:, 1]*edge[0])/safe
        along = (offset[:, 0]*directions[:, 1]-offset[:, 1]*directions[:, 0])/safe
        hit = ~parallel & (distance > .05) & (along >= 0.) & (along <= 1.)
        best = np.where(hit & (distance < best), distance, best)
    valid = np.isfinite(best) & (np.abs(angles) < math.radians(150))
    return origins[valid]+directions[valid]*best[valid, None], angles[valid]


def wall_path(points, bearings=None):
    return local_path(points, 'left', .505, horizon=.65,
                      narrow_occluder_bridge=True, bearings_rad=bearings)


def test_origin_zero_preserves_default_path_and_diagnostics_exactly():
    points, bearings = cast(TWO_DEPTH_WALLS, [0., 0.])
    old, old_info = wall_path(points)
    current, info = wall_path(points, bearings)
    assert old is not None
    np.testing.assert_array_equal(current, old)
    assert info == old_info


def test_sensor_offsets_and_continuous_origins_keep_observed_wall_extent():
    origins = ([.06, .002], [.075, .002],
               np.column_stack((np.linspace(.06, .075, 500), np.full(500, .002))))
    for origin in origins:
        points, bearings = cast(TWO_DEPTH_WALLS, origin)
        before = points.copy()
        _, old_info = wall_path(points)
        path, info = wall_path(points, bearings)
        assert path is not None
        # 유한 벽 끝 .36 m보다 뒤를 생성하지 않고 실제 관측한 앞부분을 보존한다.
        assert .32 < info['boundary_observed_ahead_m'] < .35
        assert info['boundary_observed_ahead_m'] > old_info.get('boundary_observed_ahead_m', 0.)+.04
        np.testing.assert_array_equal(points, before)


def test_two_depth_returns_are_not_interleaved_by_base_origin_parallax():
    points, bearings = cast(TWO_DEPTH_WALLS, [.075, .002])
    near = points[:, 1] < 1.
    base_order = np.argsort(-np.arctan2(points[:, 1], points[:, 0]))
    ray_order = np.argsort(-bearings)
    assert np.count_nonzero(np.diff(near[base_order])) > 1
    assert np.count_nonzero(np.diff(near[ray_order])) == 1
    assert wall_path(points)[0] is None
    assert wall_path(points, bearings)[0] is not None


def test_narrow_post_stays_in_raw_occupancy_while_wall_path_bridges_it():
    points, bearings = cast([(-1., .5, 2., .5), (.28, .35, .29, .35)], [.06, 0.])
    before = points.copy()
    path, info = local_path(points, 'left', .5, narrow_occluder_bridge=True, bearings_rad=bearings)
    assert path is not None and info['narrow_occluders_bridged'] == 1
    assert np.any(points[:, 1] < .45)
    np.testing.assert_array_equal(points, before)


def test_wide_vehicle_face_is_not_removed_as_a_narrow_occluder():
    # 15 cm 실제 면 양끝과 벽 반환을 포함한다. 경로용 처리도 차량을 조각내 제거하지 않는다.
    cluster = np.column_stack((np.linspace(.375, .525, 12), np.full(12, .37)))
    points = np.vstack(([.33, .53], [.36, .51], cluster, [.55, .52], [.58, .53]))
    before = points.copy()
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    limits = .10+.035*np.minimum(np.linalg.norm(points[:-1], axis=1), np.linalg.norm(points[1:], axis=1))
    removed, joins = break_adjacent_occluders(points, gaps > limits)
    assert not removed.any() and not joins
    np.testing.assert_array_equal(points, before)


def test_bearings_match_point_shape_and_finite_point_filter():
    points, bearings = cast(TWO_DEPTH_WALLS, [.075, .002])
    for invalid in (bearings[:-1], bearings[:, None], np.full(len(points), np.nan), np.full(len(points), np.inf)):
        path, info = wall_path(points, invalid)
        assert path is None and info['reason'] == 'wall_bearings_invalid'
    original, info = wall_path(points, bearings)
    filtered, filtered_info = wall_path(np.vstack(([np.nan, 0.], points)), np.r_[.5, bearings])
    np.testing.assert_array_equal(filtered, original)
    assert filtered_info == info


def test_local_follow_subset_keeps_each_actual_return_bearing(monkeypatch):
    # 경계 선택 호출만 격리한다. 순서를 뒤집은 실제 부분집합에도 원 광선 방향이 전달돼야 한다.
    xs = np.linspace(-.25, 2., 40)
    points = np.vstack((np.column_stack((xs, np.full(40, .425))),
                        np.column_stack((xs, np.full(40, -.425)))))
    origins = np.column_stack((np.linspace(.06, .075, len(points)), np.zeros(len(points))))
    directions = points-origins
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    rays = tuple(SensorRay(tuple(origin), tuple(point), .5, 'return', 'laser_frame', 0,
                           3*index+10, tuple(direction))
                 for index, (origin, point, direction) in enumerate(zip(origins, points, directions)))
    # 후방 UNKNOWN도 그대로 남긴다. 반환점/bearing 배열에 들어가면 대응이 틀린다.
    rays += (SensorRay((.06, 0.), None, .5, 'unknown', 'laser_frame', 0, 999, (-1., 0.)),)
    scene = LocalScene(.6, .5, .5, 'laser_frame', 0, rays)
    before = scene.rays
    expected = {tuple(ray.hit_xy_m): math.atan2(ray.direction_xy[1], ray.direction_xy[0])
                for ray in rays if ray.status == 'return'}
    subset = points[::3][::-1].copy()
    calls = []

    def observed(cloud, side, *args, **kwargs):
        supplied = np.asarray(kwargs['bearings_rad'])
        np.testing.assert_array_equal(supplied, [expected[tuple(point)] for point in cloud])
        calls.append(np.asarray(cloud).copy())
        return None, {'reason': 'wall_segment_short'}

    monkeypatch.setattr(follow, 'local_path', observed)
    monkeypatch.setattr(follow, 'boundary_support', lambda *args: subset.copy())
    monkeypatch.setattr(follow, 'drop_protrusions', lambda selected, *args: selected)
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
    for time in np.arange(0., .701, .01):
        history.add_mean_wheel(float(time), 0.)
        history.add_gyro(float(time), 0.)
    controller = follow.LocalFollow(follow.FollowConfig())
    controller.side, controller.epoch = 'left', 0
    controller.path = np.column_stack((np.linspace(-.2, 1.8, 36), np.zeros(36)))
    controller.path_ref = controller.path_source = .5
    controller.path_wall = 'left'
    controller._baseline_command(scene, history, 0., 'left')
    assert any(np.array_equal(cloud, subset) for cloud in calls)
    assert scene.rays == before and len(scene.occupied_endpoints_xy_m) == len(points)
