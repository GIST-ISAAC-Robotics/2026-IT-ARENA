import math

import numpy as np
import pytest

from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable
from arena_autonomy.lidar_point_review import MAX_POINTS, project_acquisition_points


def history(v=1.2, w=0.):
    config = MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675,
                          lidar_x_m=.06, lidar_y_m=.01, lidar_yaw_rad=.03)
    result = MotionHistory(config)
    for t in np.arange(0, .301, .005):
        # 모터축 환산 뒤 평균 바퀴 속도 한 채널. 좌우 차이는 만들지 않는다.
        result.add_mean_wheel(float(t), v/config.wheel_radius_m)
        result.add_gyro(float(t), w)
    return result


def project(h=None, **changes):
    args = dict(ranges=[1., 2., 3.], angles_rad=[.3, -.1, .2],
                stamps_s=[.1, .13, .2], now=.25, range_min=.05, range_max=12.,
                mode='both', timing_verified=True)
    args.update(changes)
    return project_acquisition_points(h or history(), **args)


@pytest.mark.parametrize('mode', ['both', 'deskew'])
@pytest.mark.parametrize('increment', [.07, -.07])
def test_uniform_path_matches_original_exactly(mode, increment):
    h = history(w=.6)
    ranges = [1., np.inf, 2., np.nan, 3., .01, -1., 4.]
    times = .1+np.arange(len(ranges))*.01
    angles = -.3+np.arange(len(ranges))*increment
    old, old_i, old_m = h.project(ranges, -.3, increment, .05, 12., .1, .01, .25, mode, True)
    new, new_i, new_m = project(h, ranges=ranges, angles_rad=angles, stamps_s=times, mode=mode)
    np.testing.assert_array_equal(new_i, old_i)
    np.testing.assert_array_equal(new, old)
    for key in ('scan_stamp_s', 'last_ray_stamp_s', 'reference_stamp_s', 'oldest_observation_age_s', 'newest_observation_age_s'):
        assert new_m[key] == old_m[key]


def test_nonuniform_points_on_independent_static_wall():
    h = history()
    angles = np.array([.3, -.2, .25, -.4])
    times = np.array([.1, .111, .147, .2])
    ranges = (2.-1.2*times-h.config.lidar_x_m)/np.cos(angles+h.config.lidar_yaw_rad)
    result, indices, meta = project(h, ranges=ranges, angles_rad=angles, stamps_s=times)
    np.testing.assert_allclose(result[:, 0], 2.-1.2*.25, atol=1e-12)
    np.testing.assert_array_equal(indices, np.arange(4))
    assert meta['review_only'] and not meta['ready_to_drive'] and not meta['resampled']


def test_turning_points_match_independent_constant_twist_geometry():
    v, w = 1.2, .8
    h = history(v=v, w=w)
    times = np.array([.1, .109, .132, .175, .2])
    angles = np.array([2.8, -3., -2., -.8, .4])  # 각도 wrap/비균일, 취득 시각은 증가
    ranges = np.array([1., 2., 1.5, .7, 3.])
    def pose(t):
        c, s = math.cos(w*t), math.sin(w*t)
        return np.array([v/w*s+.0675*c, v/w*(1-c)+.0675*s]), np.array([[c,-s],[s,c]])
    world = []
    for t,a,r in zip(times,angles,ranges):
        p, rot = pose(t)
        local = np.array([r*math.cos(a+.03)+.06, r*math.sin(a+.03)+.01])
        world.append(p+rot@local)
    p, rot = pose(.25)
    expected = (np.array(world)-p)@rot
    result, _, _ = project(h, ranges=ranges, angles_rad=angles, stamps_s=times)
    np.testing.assert_allclose(result, expected, atol=1e-12)


@pytest.mark.parametrize('flag', [False, None, 1, 'true', np.bool_(True)])
def test_timing_contract_cannot_be_inferred_from_plausible_values(flag):
    with pytest.raises(MotionUnavailable, match='unverified'):
        project(timing_verified=flag)


@pytest.mark.parametrize('changes,reason', [
    (dict(stamps_s=[.1,.1,.2]), 'order'), (dict(stamps_s=[.2,.13,.1]), 'order'),
    (dict(stamps_s=[-.1,.1,.2]), 'order'), (dict(stamps_s=[.1,.13,.3]), 'future'),
    (dict(now=.55), 'stale'), (dict(stamps_s=[.1,np.nan,.2]), 'stamps'),
    (dict(angles_rad=[.1,np.inf,.2]), 'angles'), (dict(angles_rad=[True,False,True]), 'angles'),
    (dict(stamps_s=[.1,True,.2]), 'stamps'), (dict(ranges=[1.,False,2.]), 'ranges'),
    (dict(angles_rad=[[.1],[.1,.2]]), 'angles'),
    (dict(ranges=['1','2','3']), 'ranges'), (dict(stamps_s=[.1,.2]), 'length'),
    (dict(range_max=0.), 'metadata'), (dict(range_min=-1.), 'metadata'),
    (dict(now=True), 'metadata'), (dict(now=math.inf), 'metadata'),
    (dict(stamps_s=[[.1,.13,.2]]), 'stamps'), (dict(ranges=[1.]), 'ranges'),
    (dict(ranges=np.ones(MAX_POINTS+1)), 'ranges')])
def test_invalid_contract_rejected(changes, reason):
    with pytest.raises(MotionUnavailable, match=reason):
        project(**changes)


def test_invalid_range_does_not_remove_its_bad_time():
    with pytest.raises(MotionUnavailable, match='future'):
        project(ranges=[1., 2., np.inf], stamps_s=[.1,.13,.3])


def test_missing_returns_keep_indices_and_do_not_create_free_points():
    h = history()
    before = (list(h.wheels.times), list(h.gyro.times))
    points, indices, meta = project(h, ranges=[np.inf, 2., np.nan])
    assert points.shape == (1,2)
    np.testing.assert_array_equal(indices, [1])
    assert meta['scan_stamp_s'] == .1 and meta['last_ray_stamp_s'] == .2
    empty, indices, _ = project(h, ranges=[np.inf, np.nan, -.1])
    assert empty.shape == (0,2) and not len(indices)
    assert before == (h.wheels.times, h.gyro.times)


def test_missing_motion_history_is_not_an_implicit_uncompensated_fallback():
    with pytest.raises(MotionUnavailable, match='waiting'):
        project(MotionHistory())


def test_no_approximate_mode_or_angle_sort_is_introduced():
    with pytest.raises(ValueError):
        project(mode='shift')
    with pytest.raises(MotionUnavailable, match='order'):
        project(angles_rad=sorted([.3, -.1, .2]), stamps_s=[.13,.2,.1])
