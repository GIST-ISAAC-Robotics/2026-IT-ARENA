from copy import deepcopy
import json
from pathlib import Path

import pytest

from audit_hardware_capture import audit, read_capture, SCHEMA_V2
from test_hardware_capture_audit import header, records, stationary_header


def v2_header():
    return {**header(), 'schema': SCHEMA_V2, 'accel_unit': 'm/s^2', 'accel_frame': 'imu_link',
            'accel_semantics': 'specific_force_including_gravity',
            'accel_timing': 'independent_acquisition_stream', 'accel_full_scale_mps2': 8*9.80665}


def v2_rows():
    rows = records()
    for i in range(5):
        rows.append(dict(kind='accel', boot='first', seq=i, capture_us=100500+i*10000,
                         receive_ns=8_000_500_000+i*10_000_000, status='available',
                         specific_force_mps2=[0., 0., 9.80665]))
    return sorted(rows, key=lambda r: r['receive_ns'])


def accel(rows):
    return [row for row in rows if row['kind'] == 'accel']


def test_v2_retains_sensor_specific_force_without_tilt_or_rest_claim():
    result = audit(v2_header(), v2_rows())
    assert result['record_integrity_valid'] and result['recorded_sequence_complete']
    stream = next(s for s in result['streams'] if s['kind'] == 'accel')
    assert stream['norm_specific_force_mps2']['median'] == pytest.approx(9.80665)
    assert stream['axis_specific_force_mps2'][2]['median'] == pytest.approx(9.80665)
    assert not stream['gravity_removed'] and not stream['attitude_estimated']
    assert result['acceleration_review']['all_recorded_samples_available']
    for flag in ('attitude_estimated', 'automatic_bias_application', 'gyro_accel_synchronization_verified',
                 'chassis_stationarity_verified', 'hardware_full_scale_verified'):
        assert result['acceleration_review'][flag] is False
    assert not result['complete_capture_verified'] and not result['ready_to_drive']


@pytest.mark.parametrize('status,value', [('unavailable', None), ('saturated', None),
                                        ('saturated', [0., 0., 8*9.80665])])
def test_absence_saturation_recorded_not_zero_filled(status, value):
    rows = v2_rows()
    accel(rows)[1].update(status=status, specific_force_mps2=value)
    result = audit(v2_header(), rows)
    assert result['record_integrity_valid']  # valid record of invalid/unavailable measurement
    assert not result['acceleration_review']['all_recorded_samples_available']
    stream = next(s for s in result['streams'] if s['kind'] == 'accel')
    assert stream['status_counts'][status] == 1 and stream['numeric_available_samples'] == 4
    assert stream['norm_specific_force_mps2']['minimum'] == pytest.approx(9.80665)


def test_all_unavailable_has_no_statistics_or_synthetic_gravity():
    rows = v2_rows()
    for row in accel(rows):
        row.update(status='unavailable', specific_force_mps2=None)
    result = audit(v2_header(), rows)
    stream = next(s for s in result['streams'] if s['kind'] == 'accel')
    assert stream['norm_specific_force_mps2'] is None
    assert stream['axis_specific_force_mps2'] == [None]*3


@pytest.mark.parametrize('value', [78.4532, -78.4532, 1000.])
def test_available_at_or_beyond_fullscale_is_issue_not_clamped(value):
    rows = v2_rows()
    accel(rows)[0]['specific_force_mps2'][0] = value
    result = audit(v2_header(), rows)
    assert not result['record_integrity_valid']
    assert any(i['reason'] == 'accel_at_or_beyond_declared_full_scale' for i in result['issues'])
    assert not result['acceleration_review']['all_recorded_samples_available']
    assert accel(rows)[0]['specific_force_mps2'][0] == value


@pytest.mark.parametrize('field,value', [('accel_unit', 'g'), ('accel_frame', 'world'),
    ('accel_semantics', 'gravity_removed'), ('accel_timing', 'assumed_same_as_gyro'),
    ('accel_full_scale_mps2', True), ('accel_full_scale_mps2', 0), ('accel_full_scale_mps2', float('inf'))])
def test_explicit_accel_convention_required(field, value):
    h = v2_header()
    h[field] = value
    with pytest.raises(ValueError):
        audit(h, v2_rows())


@pytest.mark.parametrize('status,value', [('bad', [0, 0, 0]), ('available', None),
    ('unavailable', [0, 0, 0]), ('available', [True, 0, 0]), ('saturated', [0, float('nan'), 0]),
    ('available', [0, 0]), ('available', [10**1000, 0, 0])])
def test_invalid_status_vector_not_silently_repaired(status, value):
    rows = v2_rows()
    accel(rows)[0].update(status=status, specific_force_mps2=value)
    with pytest.raises(ValueError):
        audit(v2_header(), rows)


def test_accel_missing_in_boot_separate_from_unavailable():
    result = audit(v2_header(), records())
    assert not result['record_integrity_valid']
    assert dict(reason='missing_stream_in_boot', stream='accel', boot='first') in result['issues']


def test_accel_has_independent_sequence_and_capture_time():
    rows = v2_rows()
    for row in accel(rows):
        row['seq'] += 100
    result = audit(v2_header(), rows)
    assert result['recorded_sequence_complete']
    accel(rows)[2]['seq'] += 10
    result = audit(v2_header(), rows)
    assert not result['record_integrity_valid']
    assert any(i.get('stream') == 'accel' for i in result['issues'])


def test_accel_sequence_gap_remains_record_gap_not_sensor_loss():
    rows = v2_rows()
    for row in accel(rows)[2:]:
        row['seq'] += 4
    result = audit(v2_header(), rows)
    assert result['record_integrity_valid'] and not result['recorded_sequence_complete']
    assert next(s for s in result['streams'] if s['kind'] == 'accel')['missing_sequence_numbers'] == 4


def test_v1_results_not_extended_or_rewritten():
    result = audit(header(), records())
    assert result['schema'] == 'arena_hardware_capture_v1'
    assert 'acceleration_review' not in result
    assert len(result['streams']) == 2
    with pytest.raises(ValueError):
        audit(header(), v2_rows())


def test_accel_failure_does_not_turn_gyro_candidate_into_attitude_calibration():
    h = {**v2_header(), 'externally_stationary_windows': stationary_header()['externally_stationary_windows']}
    result = audit(h, v2_rows())
    assert result['stationary_windows'][0]['candidate_only']
    assert result['stationary_windows'][0]['candidate_bias_rad_s'] == pytest.approx([.01, -.02, .03])
    assert not result['acceleration_review']['attitude_estimated']


def test_v2_roundtrip_input_immutable(tmp_path):
    h, rows = v2_header(), v2_rows()
    before = deepcopy((h, rows))
    path = tmp_path/'capture.jsonl'
    path.write_text(''.join(json.dumps(r, allow_nan=False)+'\n' for r in [h, *rows]))
    result = read_capture(path)
    assert result['record_integrity_valid'] and (h, rows) == before
    assert json.loads(json.dumps(result, allow_nan=False)) == result


def test_new_boot_without_accel_not_masked_by_prior_boot():
    rows = v2_rows()
    for row in records():
        rows.append({**row, 'boot': 'second', 'receive_ns': row['receive_ns']+1000000000})
    result = audit(v2_header(), rows)
    assert not result['record_integrity_valid']
    assert dict(reason='missing_stream_in_boot', stream='accel', boot='second') in result['issues']


@pytest.mark.parametrize('field,value,reason', [('capture_us', 100, 'device_capture_not_increasing'),
                                             ('receive_ns', 100, 'host_receive_clock_regressed')])
def test_accel_clock_regression_is_not_retimed(field, value, reason):
    rows = v2_rows()
    accel(rows)[2][field] = value
    result = audit(v2_header(), rows)
    assert not result['record_integrity_valid']
    assert any(i['reason'] == reason for i in result['issues'])


def test_fixture_is_explicit_synthetic_with_invalid_measurements_preserved():
    path = Path(__file__).resolve().parents[1]/'config/tests/hardware_capture_accel_v2.jsonl'
    result = read_capture(path)
    assert result['record_integrity_valid'] and result['origin'] == 'synthetic'
    stream = next(s for s in result['streams'] if s['kind'] == 'accel')
    assert stream['status_counts'] == dict(available=1, unavailable=1, saturated=1)
    assert not result['acceleration_review']['all_recorded_samples_available']
