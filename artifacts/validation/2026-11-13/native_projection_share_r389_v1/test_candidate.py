"""r389 대표 입력/실패 반례. 실행은 배정된 Linux 담당자가 수행한다."""
from dataclasses import replace
import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
_spec = importlib.util.spec_from_file_location('r389_candidate_tests', A/'candidate.py')
candidate = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(candidate)
from arena_autonomy import native_geometry as ng
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.scan_input import project_scan
from arena_autonomy.scene_motion import motion_scene


def inputs(n=508, clockwise=True):
    rng = np.random.default_rng(389+n)
    ranges = rng.uniform(.01, 13., n).tolist()
    for i, value in enumerate((math.nan, math.inf, -math.inf, 0.)):
        ranges[i*7] = value
    sign = -1 if clockwise else 1
    scan = MeasurementScan(tuple(ranges), tuple(sign*(i*math.tau/n+.0001*math.sin(i)) for i in range(n)),
                           tuple(1.+i*.1/n for i in range(n)), time_basis='simulation_acquisition')
    config = MotionConfig(lidar_x_m=.06, lidar_y_m=.013, lidar_yaw_rad=-.1)
    return scan, rng.normal(0, .1, (n, 3)), config


def motion_history():
    h = MotionHistory(MotionConfig(lidar_x_m=.06, lidar_y_m=.013, lidar_yaw_rad=-.1))
    for stamp in np.arange(.9, 1.2, .005):
        h.add_mean_wheel(float(stamp), 1./h.config.wheel_radius_m)
        h.add_gyro(float(stamp), .4)
    return h


MASKS = [0., 30., {'rear_half_angle_deg': 0., 'sectors_deg': [[80., 4.], [-120., 3.]],
                   'range_limits_m': [[.05, 2.], [.05, 3.]]}]


@pytest.mark.parametrize('n,clockwise', [(424, False), (508, True), (524, True)])
@pytest.mark.parametrize('mask', MASKS)
def test_scene_and_unmasked_points_exact(n, clockwise, mask):
    scan, poses, cfg = inputs(n, clockwise)
    expected_scene = ng.native_motion_scene(scan, poses, 1.12, cfg, 0, mask)
    expected_points = ng.native_motion_points(scan, poses, 1.12, cfg, 0)
    with candidate.enabled(verify=True) as stats:
        scene = ng.native_motion_scene(scan, poses, 1.12, cfg, 0, mask)
        actual = ng.native_motion_points(scan, poses.copy(), 1.12, cfg, 0)
        assert scene == expected_scene
        assert candidate.same_arrays(actual, expected_points)
        assert stats['shared_points_calls'] == stats['verified_points_calls'] == 1
        assert all(ray.hit_xy_m is None for ray in scene.rays if ray.status != 'return')
    assert scene.source_angles_rad == scan.angles_rad
    assert tuple(ray.source_index for ray in scene.rays) == tuple(range(n))
    assert tuple(ray.acquired_at_s for ray in scene.rays) == scan.acquired_at_s


def test_masked_valid_returns_still_in_raw_points_and_no_alias():
    scan = MeasurementScan((2., 3., math.inf), (0., 1., 2.), (1., 1.01, 1.02),
                           time_basis='simulation_acquisition')
    cfg, poses = MotionConfig(), np.zeros((3, 3))
    mask = {'rear_half_angle_deg': 0., 'sectors_deg': [[0., 10.]]}
    with candidate.enabled() as stats:
        scene = ng.native_motion_scene(scan, poses, 1.12, cfg, 0, mask)
        first = ng.native_motion_points(scan, poses, 1.12, cfg, 0)
        first[0][:] = 777.; first[1][:] = -1
        second = ng.native_motion_points(scan, poses, 1.12, cfg, 0)
        assert second[1].tolist() == [0, 1]
        assert scene.rays[0].status == 'masked' and scene.rays[0].hit_xy_m is None
        assert stats['shared_points_calls'] == 2
    assert candidate.same_arrays(second, ng.native_motion_points(scan, poses, 1.12, cfg, 0))


def test_no_return_keeps_empty_shape_and_unknowns():
    scan, poses, cfg = inputs()
    scan = replace(scan, ranges_m=(math.inf,)*len(scan.ranges_m))
    with candidate.enabled():
        scene = ng.native_motion_scene(scan, poses, 1.12, cfg, 0, 0.)
        points, ids = ng.native_motion_points(scan, poses, 1.12, cfg, 0)
    assert points.shape == (0, 2) and ids.shape == (0,)
    assert all(ray.status == 'unknown' for ray in scene.rays)


def failure(call):
    with pytest.raises(Exception) as caught:
        call()
    return type(caught.value), str(caught.value)


@pytest.mark.parametrize('change', ['scan', 'reference', 'epoch', 'config', 'poses', 'signed_zero', 'dtype'])
def test_different_input_falls_back(change):
    scan, poses, cfg = inputs()
    poses[0, 0] = 0.
    base = (scan, poses, 1.12, cfg, 0)
    args = list(base)
    if change == 'scan': args[0] = replace(scan)
    if change == 'reference': args[2] = 1.13
    if change == 'epoch': args[4] = 1
    if change == 'config': args[3] = replace(cfg)
    if change == 'poses': args[1] = poses.copy(); args[1][0, 1] += .01
    if change == 'signed_zero': args[1] = poses.copy(); args[1][0, 0] = -0.
    if change == 'dtype': args[1] = poses.astype(np.float32)
    expected = ng.native_motion_points(*args)
    with candidate.enabled() as stats:
        ng.native_motion_scene(*base, 30.)
        actual = ng.native_motion_points(*args)
    assert stats['shared_points_calls'] == 0 and stats['fallback_points_calls'] == 1
    assert candidate.same_arrays(actual, expected)


@pytest.mark.parametrize('fault', ['nan_pose', 'bool_pose', 'shape', 'future', 'bad_mask', 'negative_epoch'])
def test_scene_failure_type_and_message_preserved(fault):
    scan, poses, cfg = inputs()
    now, epoch, mask = 1.12, 0, 0.
    if fault == 'nan_pose': poses[0, 0] = math.nan
    if fault == 'bool_pose': poses = poses.astype(bool)
    if fault == 'shape': poses = poses[:-1]
    if fault == 'future': now = 1.05
    if fault == 'bad_mask': mask = 180.
    if fault == 'negative_epoch': epoch = -1
    call = lambda: ng.native_motion_scene(scan, poses, now, cfg, epoch, mask)
    expected = failure(call)
    with candidate.enabled():
        assert failure(call) == expected


def test_masked_overflow_is_rejected_at_points_not_scene():
    scan = MeasurementScan((1e308, 1e308), (0., math.pi/2), (1., 1.01), range_max_m=1.5e308,
                           time_basis='simulation_acquisition')
    cfg, poses = MotionConfig(lidar_x_m=0.), np.array([[1e308, 0., 0.], [0., 0., 0.]])
    mask = {'rear_half_angle_deg': 0., 'sectors_deg': [[0., 10.]]}
    scene = ng.native_motion_scene(scan, poses, 1.12, cfg, 0, mask)
    expected = failure(lambda: ng.native_motion_points(scan, poses, 1.12, cfg, 0))
    with candidate.enabled() as stats:
        assert ng.native_motion_scene(scan, poses, 1.12, cfg, 0, mask) == scene
        assert failure(lambda: ng.native_motion_points(scan, poses, 1.12, cfg, 0)) == expected
        assert stats['shared_points_calls'] == 1


def test_nonstandard_config_and_single_use_mask_keep_original_path():
    scan, poses, cfg = inputs()
    unusual = SimpleNamespace(lidar_x_m=cfg.lidar_x_m, lidar_y_m=cfg.lidar_y_m, lidar_yaw_rad=cfg.lidar_yaw_rad)
    def mask():
        return {'rear_half_angle_deg': 0., 'sectors_deg': iter([[80., 4.]])}
    for config, masks in [(unusual, lambda: 0.), (cfg, mask)]:
        expected = ng.native_motion_scene(scan, poses, 1.12, config, 0, masks())
        with candidate.enabled() as stats:
            actual = ng.native_motion_scene(scan, poses, 1.12, config, 0, masks())
            assert actual == expected and stats['shareable_scenes'] == 0
    class ArrayView(np.ndarray):
        pass
    unusual_poses = poses.view(ArrayView)
    expected = ng.native_motion_scene(scan, unusual_poses, 1.12, cfg, 0, 0.)
    with candidate.enabled() as stats:
        assert ng.native_motion_scene(scan, unusual_poses, 1.12, cfg, 0, 0.) == expected
        assert stats['shareable_scenes'] == 0


def test_measurement_subclass_uses_existing_general_path():
    class Derived(MeasurementScan):
        pass
    scan, _, _ = inputs()
    derived = Derived(scan.ranges_m, scan.angles_rad, scan.acquired_at_s, time_basis=scan.time_basis)
    h = motion_history()
    expected = project_scan(derived, h, 1.12, timing_verified=True)
    with candidate.enabled() as stats:
        motion_scene(derived, h, 1.12, timing_verified=True)
        actual = project_scan(derived, h, 1.12, timing_verified=True)
    assert stats['scene_calls'] == stats['points_calls'] == 0
    assert candidate.same_arrays(actual[:2], expected[:2]) and actual[2] == expected[2]


@pytest.mark.parametrize('change', ['unverified', 'future', 'history_empty', 'history_changed', 'motion_epoch'])
def test_high_level_revalidation_metadata_and_epoch(change):
    scan, _, _ = inputs(); h = motion_history()
    now, verified = 1.12, True
    epoch = 1 if change == 'motion_epoch' else 0
    with candidate.enabled() as stats:
        scene = motion_scene(scan, h, 1.12, timing_verified=True, epoch=epoch)
        if change == 'unverified': verified = False
        if change == 'future': now = 1.05
        if change == 'history_empty': h.gyro.times.clear(); h.gyro.values.clear()
        if change == 'history_changed': h.gyro.values[-24] += .1
        call = lambda: project_scan(scan, h, now, timing_verified=verified)
        if change in ('unverified', 'future', 'history_empty'):
            actual = failure(call)
        else:
            actual = call()
    if change in ('unverified', 'future', 'history_empty'):
        assert failure(call) == actual and stats['points_calls'] == 0
    else:
        expected = call()
        assert candidate.same_arrays(actual[:2], expected[:2]) and actual[2] == expected[2]
        assert stats['shared_points_calls'] == 0 and stats['fallback_points_calls'] == 1
    if change == 'motion_epoch':
        assert scene.source_epoch == 2


def test_scope_exit_restores_functions_and_has_no_cross_call_state():
    scan, poses, cfg = inputs()
    before = ng.native_motion_scene, ng.native_motion_points
    with candidate.enabled():
        ng.native_motion_scene(scan, poses, 1.12, cfg, 0, 0.)
    with candidate.enabled() as stats:
        ng.native_motion_points(scan, poses, 1.12, cfg, 0)
        assert stats['shared_points_calls'] == 0
    assert before == (ng.native_motion_scene, ng.native_motion_points)
    with pytest.raises(RuntimeError, match='deliberate'):
        with candidate.enabled():
            raise RuntimeError('deliberate')
    assert before == (ng.native_motion_scene, ng.native_motion_points)
