from copy import deepcopy

import pytest

from audit_g1_trace_timing import audit, pose_neighborhood


def example():
    diagnostic = dict(sim_time_s=1., truth_longitudinal_mps=.002, truth_lateral_mps=0.)
    trace = [dict(t=1.02, ground_speed_mps=.002,
                  braking_diagnostics=dict(**diagnostic, safe_command_stamp_s=1.01, mcu_now_us=1_015_000))]
    poses = [[1., 0., 0., 0.], [1.01, .00002, 0., 0.], [1.02, .00014, 0., 0.], [1.03, .00027, 0., 0.]]
    return trace, poses, [diagnostic], [(1.01, 1.03)]


def test_time_misalignment_is_reported_not_relabelled_as_zero_motion():
    result = audit(*example())
    assert result['raw_source_consistent']
    assert result['source_offset_from_pose_s']['dynamics']['maximum'] == pytest.approx(.02)
    assert result['source_offset_from_pose_s']['safe_command']['maximum'] == pytest.approx(.01)
    sample = result['windows'][0]['trace_samples'][0]
    assert sample['stored_source_speed_mps'] == .002
    assert sample['pose_near_source']['segments'][0]['average_speed_mps'] == pytest.approx(.002)
    assert min(s['average_speed_mps'] for s in sample['pose_near_trace']['segments']) > .01
    assert result['windows'][0]['time_over_existing_0p01_mps_s'] == pytest.approx(.02)
    assert not result['original_verdict_changed'] and not result['continuous_stop_certified']


def test_missing_diagnostics_remain_unknown():
    args = example()
    args[0][0].pop('braking_diagnostics')
    result = audit(*args)
    assert result['raw_source_consistent']  # 모순 없음은 자료 완비/합격이 아니다.
    assert result['available_diagnostic_samples'] == 0
    assert result['source_offset_from_pose_s']['dynamics'] is None
    assert result['missing_source_indices']['dynamics'] == [0]
    assert not result['freshness_certified']


def test_channel_ahead_of_pose_is_not_corrupt_or_proven_future_capture():
    trace, poses, dynamics, windows = example()
    trace[0]['braking_diagnostics']['safe_command_stamp_s'] = 1.04
    result = audit(trace, poses, dynamics, windows)
    assert result['raw_source_consistent'] and not result['freshness_certified']
    assert result['source_after_pose_reference'][0]['stream'] == 'safe_command'
    assert result['source_after_pose_reference'][0]['offset_s'] == pytest.approx(-.02)


@pytest.mark.parametrize('fault,reason', [('changed', 'copied_diagnostic'),
                                       ('missing', 'raw_diagnostic_source'), ('speed', 'copied_speed'),
                                       ('nan', 'nonfinite_speed')])
def test_bad_evidence_reported(fault, reason):
    trace, poses, dynamics, windows = deepcopy(example())
    if fault == 'changed': trace[0]['braking_diagnostics']['truth_longitudinal_mps'] = .003
    elif fault == 'missing': trace[0]['braking_diagnostics']['sim_time_s'] = .98
    elif fault == 'speed': trace[0]['ground_speed_mps'] = .003
    elif fault == 'nan': trace[0]['ground_speed_mps'] = float('nan')
    report = audit(trace, poses, dynamics, windows)
    assert not report['raw_source_consistent']
    assert any(reason in e['reason'] for e in report['source_errors'])


def test_no_pose_extrapolation_or_gap_filling():
    poses = [[1., 0., 0., 0.], [2., 1., 0., 0.]]
    assert not pose_neighborhood(poses, [1., 2.], .99)['available']
    segment = pose_neighborhood(poses, [1., 2.], 1.5)['segments'][0]
    assert segment['duration_s'] == 1.  # 큰 공백도 숨기지 않는다.


@pytest.mark.parametrize('kind', ['trace', 'pose', 'diagnostics', 'window'])
def test_invalid_timeline_rejected(kind):
    trace, poses, dynamics, windows = example()
    if kind == 'trace': trace.append(deepcopy(trace[0]))
    elif kind == 'pose': poses.append(poses[-1])
    elif kind == 'diagnostics': dynamics.append(dynamics[0])
    elif kind == 'window': windows = [(2., 1.)]
    with pytest.raises(ValueError): audit(trace, poses, dynamics, windows)
