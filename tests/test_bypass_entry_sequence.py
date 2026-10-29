"""가상 센서 생성과 평가/제어 경계. 실물/동역학 검증 아님."""
import math
import pytest
from probe_bypass_entry_sequence import physical_pose, render_scan, evaluation_boxes, evaluation_metrics


def rows():
    return [dict(t=t, rear_pose=[-.0675, 0., 0.], speed=0., steering=0.) for t in (0., .1, .2)]


def test_generated_scan_keeps_sequential_stamps_and_no_truth_fields():
    scan = render_scan(rows(), .2)
    assert len(scan.ranges) == 500 and scan.time_increment == .0002
    assert scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9 == pytest.approx(.1)
    assert not hasattr(scan, 'targets') and not hasattr(scan, 'pose')
    assert any(math.isnan(v) for v in scan.ranges)


def test_future_or_uncovered_physical_pose_is_rejected():
    with pytest.raises(ValueError): physical_pose(rows(), .21)
    with pytest.raises(ValueError): physical_pose(rows(), -.01)


def test_restart_motion_changes_only_evaluation_boxes():
    assert evaluation_boxes(1., restart=True)[-1].x_m == 1.2
    assert evaluation_boxes(2., restart=True)[-1].x_m == pytest.approx(1.325)
    assert evaluation_boxes(2., restart=False)[-1].x_m == 1.2


def test_contact_and_grass_are_evaluated_separately():
    mid = evaluation_metrics((-.0675, 0., 0.), 0., .5, 0., False)
    side = evaluation_metrics((-.0675, .24, 0.), 0., .5, 0., False)
    assert not mid['contact_sample'] and mid['wheels_touching_grass']==0
    assert not side['contact_sample'] and side['wheels_touching_grass']==2
    hit = evaluation_metrics((1.2-.0675, 0., 0.), 0., .5, 0., False)
    assert hit['contact_sample']
