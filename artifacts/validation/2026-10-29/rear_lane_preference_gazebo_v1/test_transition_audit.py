"""평가기의 사각/무반환 구분만 확인한다. 실제 센서나 차량 시험이 아니다."""
import math
import numpy as np
import pytest

from audit_transition import box_distances, ordered, poses_at, scan_row


def fixture(position):
    angles = -math.pi+np.arange(500)*(2*math.pi/500)
    direction = np.column_stack((np.cos(angles), np.sin(angles)))
    actor = np.tile([*position, 0.], (500, 1))
    ranges = box_distances(np.tile([.06, 0.], (500, 1)), direction, actor)
    raw = dict(ranges=ranges.tolist(), sec=1, nanosec=0, time_increment=.0002,
               angle_min=-math.pi, angle_increment=2*math.pi/500, range_min=.05, range_max=12.)
    times = np.arange(0., 1.151, .01)
    ego = [[t, 0., 0., 0.] for t in times]
    moving = [[t, *position, 0.] for t in times]
    return raw, dict(lidar_x_m=.06, lidar_y_m=0., lidar_yaw_rad=0.), ego, moving


@pytest.mark.parametrize('position,state', [((0., .29), 'alongside'),
    ((-.3, .2), 'behind_visible'), ((-.8, .2), 'blind'), ((.5, .2), 'ahead')])
def test_acquisition_geometry_and_mask_state(position, state):
    raw, motion, ego, actor = fixture(position)
    row = scan_row(raw, motion, 30., ego, actor, 1.1)
    assert row['state'] == state and row['pose_coverage_valid']


def test_no_return_outside_mask_is_unmatched_not_blind():
    raw, motion, ego, actor = fixture((-.3, .2))
    raw['ranges'] = [math.inf]*500
    row = scan_row(raw, motion, 30., ego, actor, 1.1)
    assert row['state'] == 'behind_unmatched' and row['unmasked_geometric_rays'] > 0


def test_large_pose_gap_is_invalid():
    raw, motion, ego, actor = fixture((-.8, .2))
    row = scan_row(raw, motion, 30., [ego[0], ego[-1]], actor, 1.1)
    assert not row['pose_coverage_valid'] and row['state'] == 'invalid_pose_coverage'


def test_pose_yaw_wrap_interpolation():
    values, valid = poses_at([[1., 0., 0., math.radians(179)],
                             [1.02, .02, 0., math.radians(-179)]], np.array([1.01]))
    assert valid[0] and values[0, 2] == pytest.approx(math.pi)


def test_order_does_not_accept_missing_blind():
    assert not ordered(['alongside', 'behind_visible', 'alongside', 'ahead'],
                       ['alongside', 'behind_visible', 'blind', 'ahead'])
