"""마지막 정지 구간의 부분 자료를 전 구간 증거로 오인하지 않아야 한다."""
from test_g1_traffic_coverage_audit import stationary_fixture
import audit_g1_traffic_coverage as audit
import pytest


def test_stop_pose_tail_truncation_is_not_a_complete_half_second():
    report, trace, raw, ego, stop = stationary_fixture()
    ego = [p for p in ego if p[0] <= 2.55]
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['checks']['stop_matches_dense_pose']


def test_stop_pose_internal_gap_is_not_zero_motion_proof():
    report, trace, raw, ego, stop = stationary_fixture()
    ego = [p for p in ego if not 2.60 < p[0] < 2.91]
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['checks']['stop_matches_dense_pose']


def test_late_tiny_stop_segment_does_not_cover_missing_start():
    report, trace, raw, ego, stop = stationary_fixture()
    ego = [p for p in ego if p[0] <= 2.40 or p[0] >= 2.96]
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['checks']['stop_matches_dense_pose']


def test_complete_stationary_window_and_original_limits():
    result = audit.audit_rows(*stationary_fixture())
    assert result['checks']['stop_matches_dense_pose']
    assert result['stop_pose_coverage']['complete']
    assert result['stop_pose_coverage']['maximum_allowed_gap_s'] == .05
    assert result['stop_pose_coverage']['required_last_s']-result['stop_pose_coverage']['required_first_s'] == .5


def test_stop_threshold_not_relaxed():
    segments = [dict(t0=1.+i*.01, t1=1.+(i+1)*.01, average_speed_mps=.01) for i in range(60)]
    coverage = audit.stop_pose_window(segments, [{'time': 1.}, {'time': 1.6}])
    assert coverage['complete'] and coverage['maximum_speed_mps'] == .01
    report, trace, raw, ego, stop = stationary_fixture()
    ego = [(t, .011*(t-1.), y, yaw) for t, _, y, yaw in ego]
    assert not audit.audit_rows(report, trace, raw, ego, stop)['checks']['stop_matches_dense_pose']


@pytest.mark.parametrize('times', [[], [1.], [2., 1.], [1., 1.], [1., float('nan')], [True, 2.]])
def test_invalid_stop_timeline_does_not_claim_complete(times):
    segments = [dict(t0=0., t1=3., average_speed_mps=0.)]
    assert not audit.stop_pose_window(segments, [dict(time=t) for t in times])['complete']
