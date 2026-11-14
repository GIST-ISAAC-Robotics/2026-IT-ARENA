"""고정 G1-b 런타임 밖 사후 상태/자세 감사의 반례."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import audit_g1_traffic_coverage as audit


def stationary_fixture():
    raw, trace = [], []
    for i in range(8):
        t = 1.+i*.2
        raw.append({'sim_time_s': t, 'state': 'FOLLOW_GAP_HOLD', 'reason': 'follow_gap_hold',
                    'behavior': 'HOLD', 'mode': 'g1_follow', 'speed_command_mps': 0.,
                    'source_first_s': t-.1, 'reference_stamp_s': t,
                    'path_source_s': t-.1, 'source_epoch': 0})
        trace.append({'t': t+.05, 'autonomy': raw[-1].copy(), 'ground_speed_mps': 0., 'safe_speed_mps': 0.})
    ego = [(1.+i*.01, 0., 0., 0.) for i in range(202)]
    stop = [{'time': 2.4+i*.01} for i in range(62)]
    report = {'green_observed_sim_s': 1., 'stop_request_sim_s': 2.5, 'case': 'stopped_center'}
    return report, trace, raw, ego, stop


def test_stationary_pose_and_current_raw_status_pass():
    result = audit.audit_rows(*stationary_fixture())
    assert result['passed'] and result['max_held_pose_speed_mps'] == 0.
    assert result['stop_max_pose_speed_mps'] == 0.
    assert any('freshness is unverified' in s for s in result['limits'])


def test_old_state_reuse_and_source_stale_are_rejected():
    report, trace, raw, ego, stop = stationary_fixture()
    trace[-1]['autonomy'] = raw[0].copy()
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['passed'] and not result['checks']['status_age_within_existing_observation_ttl']
    report, trace, raw, ego, stop = stationary_fixture()
    raw[-1]['speed_command_mps'] = .2
    raw[-1]['path_source_s'] = .2
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['checks']['boundary_times_valid']


def test_recorded_stationary_truth_with_moving_dense_pose_fails():
    report, trace, raw, ego, stop = stationary_fixture()
    ego = [(t, .02*(t-1.), y, yaw) for t, _, y, yaw in ego]
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['checks']['hold_matches_dense_pose']
    assert not result['checks']['stop_matches_dense_pose']


def test_slow_case_without_hold_is_not_applicable_but_stop_still_required():
    report, trace, raw, ego, stop = stationary_fixture()
    report['case'] = 'slow_lead'
    for row in raw:
        row['behavior'], row['reason'] = 'FOLLOW', 'local_follow'
    for row, status in zip(trace, raw):
        row['autonomy'] = status.copy()
        row['ground_speed_mps'] = row['safe_speed_mps'] = .35
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert result['passed'] and result['held_pose_segments'] == 0
    assert result['applicability']['hold_pose_check'] == 'not_applicable_no_hold_in_case'
    assert result['checks']['stop_matches_dense_pose']
    report['case'] = 'stop_restart'
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['passed'] and not result['checks']['hold_matches_dense_pose']
    report['case'] = 'initial_occlusion'
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['passed'] and result['applicability']['hold_required_by_case']


def test_stale_observation_holding_zero_is_diagnostic_not_positive_command_error():
    report, trace, raw, ego, stop = stationary_fixture()
    raw[-1]['reference_stamp_s'] += 1.
    raw[-1]['reason'] = 'follow_scene_stale'
    trace[-1]['autonomy'] = raw[-1].copy()
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert result['checks']['boundary_times_valid'] and not result['boundary_time_errors']
    assert len(result['rejected_observation_times']) == 1
    raw[-1]['speed_command_mps'] = .2
    trace[-1]['autonomy'] = raw[-1].copy()
    result = audit.audit_rows(report, trace, raw, ego, stop)
    assert not result['checks']['boundary_times_valid']
