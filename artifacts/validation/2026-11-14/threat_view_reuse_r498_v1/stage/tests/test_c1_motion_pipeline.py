"""분석기의 정답 기하/인덱스 계약 검사. 실제 C1/SDK 통신 검사가 아니다."""
import numpy as np
import pytest

from audit_c1_motion_pipeline import (
    CONFIG, COUNT, HZ, NOW, ROOM, START, block, build_native,
    error_metrics, motion, pose, reference_points, run, scene,
)
from arena_autonomy.lidar_point_review import project_acquisition_points


def test_pose_straight_and_nonzero_rear_axle_offset():
    p, rotation = pose(.1, 2., 0.)
    np.testing.assert_allclose(p, [.2675, 0.], atol=1e-14)
    np.testing.assert_array_equal(rotation, np.eye(2))


def test_pose_left_right_are_mirrors():
    left, lrot = pose(.17, 1.2, .9)
    right, rrot = pose(.17, 1.2, -.9)
    np.testing.assert_allclose(left, right*[1, -1], atol=1e-14)
    np.testing.assert_allclose(lrot, rrot.T, atol=1e-14)


def test_cardinal_rays_follow_publisher_frame_convention():
    data = scene(0., 0., 0.)
    # Native 0/90/180/270 degrees -> ROS +pi/+pi/2/0/-pi/2.
    # Independent cardinal hits also catch a hidden 180-degree frame swap.
    sensor_x = -CONFIG.rear_axle_x_m+CONFIG.lidar_x_m
    np.testing.assert_allclose(data['hits'][[0, 125, 250, 375]],
        [[-3., 0.], [sensor_x, 1.5], [3., 0.], [sensor_x, -1.5]], atol=1e-12)


@pytest.mark.parametrize('v,w', [(0., 0.), (1., 0.), (1., 1.), (1., -1.)])
@pytest.mark.parametrize('phase', [0., 93.])
def test_box_first_hit_and_original_point_compensation(v, w, phase):
    data = scene(v, w, phase)
    assert np.all(np.abs(data['hits']) <= ROOM+1e-12)
    normal_distance = np.abs(data['hits'][np.arange(COUNT), data['wall_axes']])
    np.testing.assert_allclose(normal_distance, ROOM[data['wall_axes']], atol=1e-12)
    assert np.all(data['range_q2'] > 0)
    assert data['times'][0] == START
    assert data['times'][-1] == START+(COUNT-1)/(COUNT*HZ)
    p, indices, _ = project_acquisition_points(motion(v, w), data['range_q2']/4000.,
        data['angles'], data['times'], NOW, .05, 12., timing_verified=True)
    expected = reference_points(data, indices, v, w)
    assert np.linalg.norm(p-expected, axis=1).max() <= .000126


def test_quarter_mm_rounding_bound_and_missing_not_free():
    data = scene(1., 1., 93., missing=True)
    ranges = data['range_q2']/4000.
    ranges[::13] = np.inf
    _, indices, _ = project_acquisition_points(motion(1., 1.), ranges, data['angles'],
        data['times'], NOW, .05, 12., timing_verified=True)
    assert len(indices) == COUNT-len(range(0, COUNT, 13))
    assert not np.any(indices % 13 == 0)


def test_endpoint_error_is_not_confused_with_wall_normal_error():
    result = error_metrics(np.array([[0., .5], [0., .3]]), np.zeros((2, 2)),
                           np.array([0, 0]), 0.)
    assert result['endpoint_max_m'] == .5
    assert result['wall_normal_max_m'] == 0


def test_extraction_is_unique_and_includes_nested_body():
    full, body = block('prefix void f() {if (x) {y();}} suffix', 'void f()')
    assert full == 'void f() {if (x) {y();}}'
    assert body == 'if (x) {y();}'
    with pytest.raises(ValueError, match='not unique'):
        block('void f() {} void f() {}', 'void f()')


def test_unknown_upstream_source_rejected_before_compile(tmp_path):
    source = tmp_path/'upstream/src'
    source.mkdir(parents=True)
    (source/'sllidar_node.cpp').write_text('different source')
    with pytest.raises(ValueError, match='hash mismatch'):
        build_native(source.parent, tmp_path)
    assert not (tmp_path/'probe').exists()


def test_existing_evidence_directory_is_not_overwritten(tmp_path):
    sentinel = tmp_path/'sentinel.txt'
    sentinel.write_text('keep')
    with pytest.raises(FileExistsError):
        run(tmp_path/'not_needed', tmp_path)
    assert sentinel.read_text() == 'keep'


def test_scan_time_offset_is_visible_in_straight_motion():
    data = scene(1., 0., 0.)
    h = motion(1., 0.)
    kwargs = dict(history=h, ranges=data['range_q2']/4000., angles_rad=data['angles'],
                  now=NOW, range_min=.05, range_max=12., timing_verified=True)
    correct, _, _ = project_acquisition_points(stamps_s=data['times'], **kwargs)
    shifted, _, _ = project_acquisition_points(stamps_s=data['times']+.01, **kwargs)
    np.testing.assert_allclose(shifted-correct, np.tile([.01, 0.], (COUNT, 1)), atol=1e-12)
