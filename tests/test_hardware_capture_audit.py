from copy import deepcopy
import math

import pytest

from audit_hardware_capture import audit, read_capture, main, digest


def header():
    return dict(kind='header', schema='arena_hardware_capture_v1', origin='synthetic',
                device_clock='mcu_monotonic_us', receive_clock='host_monotonic_ns',
                encoder_unit='extended_signed_motor_counts', gyro_unit='rad/s', gyro_frame='imu_link',
                mounting_note='synthetic x forward y left z up; not a mounting decision',
                motor_spec=dict(counts_per_motor_revolution=28, gear_ratio=15., wheel_radius_m=.0325))


def records(count_step=0):
    result = []
    for i in range(5):
        common = dict(boot='first', seq=i, capture_us=100000+i*10000, receive_ns=8_000_000_000+i*10_000_000)
        result += [dict(kind='encoder', motor_counts=count_step*i, **common),
                   dict(kind='imu', gyro_rad_s=[.01, -.02, .03], **common)]
    return result


def stationary_header():
    config = header()
    config['externally_stationary_windows'] = [dict(
        boot='first', first_us=100000, last_us=140000,
        basis='synthetic externally immobilized fixture; not calibration recommendation',
        review_requirements=dict(minimum_span_us=40000, max_sample_gap_us=10000,
                                 minimum_imu_samples=5, minimum_encoder_samples=5))]
    return config


def test_valid_capture_is_not_drive_authorization_or_absolute_delay():
    result = audit(header(), records(10))
    assert result['record_integrity_valid']
    assert not result['ready_to_drive'] and not result['absolute_transport_delay_measured']
    assert result['counts_per_mean_wheel_revolution'] == 420
    assert result['mean_wheel_travel_mm_per_count'] == pytest.approx(math.pi*65/420)
    encoder = result['streams'][0]
    assert encoder['mean_wheel_speed_mps']['median'] == pytest.approx(math.pi*.065/420*1000)


@pytest.mark.parametrize('key,value', [('gyro_unit', 'deg/s'), ('encoder_unit', 'wheel_counts'),
                                     ('receive_clock', 'ros_clock'), ('origin', 'unknown')])
def test_units_are_not_silently_converted(key, value):
    config = header()
    config[key] = value
    with pytest.raises(ValueError):
        audit(config, records())


@pytest.mark.parametrize('field,value,reason', [('seq', 0, 'sequence_not_increasing'),
                                              ('capture_us', 999, 'device_capture_not_increasing'),
                                              ('receive_ns', 1, 'host_receive_clock_regressed')])
def test_regressions_are_reported_not_retimed(field, value, reason):
    rows = records()
    rows[2][field] = value
    result = audit(header(), rows)
    assert not result['record_integrity_valid']
    assert reason in [issue['reason'] for issue in result['issues']]


def test_sequence_gap_is_counted_without_claiming_sensor_loss():
    rows = records()
    for row in rows[2:]:
        if row['kind'] == 'encoder':
            row['seq'] += 3
    result = audit(header(), rows)
    assert result['record_integrity_valid']
    assert result['streams'][0]['missing_sequence_numbers'] == 3
    assert not result['recorded_sequence_complete']
    assert not result['complete_capture_verified']


def test_new_boot_splits_counter_and_time_without_false_speed():
    rows = records(10)
    for row in records(2):
        rows.append({**row, 'boot': 'second', 'receive_ns': row['receive_ns']+1_000_000_000})
    result = audit(header(), rows)
    assert result['record_integrity_valid'] and result['boot_changes_require_explicit_rearm']
    assert len(result['streams']) == 4
    assert all(s['capture_interval_s']['maximum'] == .01 for s in result['streams'])


def test_old_boot_reappearance_rejected():
    rows = records()
    rows[4]['boot'] = 'second'
    result = audit(header(), rows)
    assert any(issue['reason'] == 'old_boot_reappeared' for issue in result['issues'])


def test_no_stationary_inference_from_zero_counts():
    result = audit(header(), records())
    assert result['stationary_windows'] == []


def test_external_stationary_window_only_proposes_unapplied_bias():
    config = stationary_header()
    result = audit(config, records())
    candidate = result['stationary_windows'][0]
    assert candidate['candidate_bias_rad_s'] == pytest.approx([.01, -.02, .03])
    assert not candidate['applied'] and not candidate['chassis_stationarity_verified']
    assert not candidate['calibration_ready']
    moving = audit(config, records(1))['stationary_windows'][0]
    assert moving['candidate_bias_rad_s'] is None


@pytest.mark.parametrize('value', [True, 1.2, 2**63, -(2**63)-1])
def test_counter_requires_extended_signed_integer(value):
    rows = records()
    rows[0]['motor_counts'] = value
    with pytest.raises(ValueError):
        audit(header(), rows)


def test_input_objects_are_not_mutated():
    config, rows = header(), records()
    before = deepcopy((config, rows))
    audit(config, rows)
    assert (config, rows) == before


def test_missing_stream_and_empty_capture_are_not_valid():
    assert not audit(header(), [])['record_integrity_valid']
    assert not audit(header(), records()[::2])['record_integrity_valid']


def test_file_reader_rejects_nonfinite_json_and_nonobjects(tmp_path):
    path = tmp_path/'bad.jsonl'
    path.write_text('{"value":NaN}\n')
    with pytest.raises(ValueError):
        read_capture(path)
    path.write_text('[]\n')
    with pytest.raises(ValueError, match='objects'):
        read_capture(path)


def test_cli_refuses_existing_output_or_input_replacement(tmp_path, monkeypatch):
    path = tmp_path/'record.jsonl'
    path.write_text('original')
    monkeypatch.setattr('sys.argv', ['audit', str(path), '--output', str(path)])
    with pytest.raises(SystemExit):
        main()
    assert path.read_text() == 'original'


def test_example_remains_explicitly_synthetic():
    from pathlib import Path
    path = Path(__file__).resolve().parents[1]/'config/tests/hardware_capture_nominal_v1.jsonl'
    result = read_capture(path)
    assert result['origin'] == 'synthetic' and result['record_integrity_valid']
    assert not result['ready_to_drive']


def test_hash_works_without_python_311_file_digest(tmp_path, monkeypatch):
    import hashlib
    monkeypatch.delattr(hashlib, 'file_digest', raising=False)
    path = tmp_path/'bytes'
    path.write_bytes(b'counts-clock')
    assert digest(path) == hashlib.sha256(b'counts-clock').hexdigest()


def test_stationary_requirements_are_not_invented_when_absent():
    config = stationary_header()
    del config['externally_stationary_windows'][0]['review_requirements']
    candidate = audit(config, records())['stationary_windows'][0]
    assert candidate['candidate_bias_rad_s'] is None
    assert 'review_requirements_missing' in candidate['rejection_reasons']
    assert candidate['coverage']['imu']['observed_span_us'] == 40000


@pytest.mark.parametrize('kind', ['imu', 'encoder'])
def test_window_sequence_holes_reject_bias_candidate_without_rewriting_order(kind):
    rows = records()
    for row in rows[4:]:
        if row['kind'] == kind:
            row['seq'] += 3
    result = audit(stationary_header(), rows)
    assert result['record_integrity_valid'] and not result['recorded_sequence_complete']
    candidate = result['stationary_windows'][0]
    assert not candidate['candidate_only']
    assert f'{kind}_sequence_gap' in candidate['rejection_reasons']
    assert candidate['coverage'][kind]['missing_sequence_numbers'] == 3


def test_truncated_requested_window_is_not_a_complete_bias_interval():
    config = stationary_header()
    config['externally_stationary_windows'][0].update(first_us=0, last_us=3_000_000)
    candidate = audit(config, records())['stationary_windows'][0]
    assert not candidate['candidate_only']
    assert 'imu_coverage_gap' in candidate['rejection_reasons']
    assert 'encoder_coverage_gap' in candidate['rejection_reasons']
    assert candidate['coverage']['imu']['end_unobserved_us'] == 2_860_000


def test_reboot_with_only_one_stream_is_invalid_even_if_first_boot_had_both():
    rows = records()
    rows.append({**rows[-1], 'boot': 'second', 'seq': 0, 'capture_us': 0,
                 'receive_ns': rows[-1]['receive_ns']+1000000})
    result = audit(stationary_header(), rows)
    assert not result['record_integrity_valid']
    assert dict(reason='missing_stream_in_boot', stream='encoder', boot='second') in result['issues']
    assert not result['stationary_windows'][0]['candidate_only']


def test_long_capture_gap_rejected_even_with_consecutive_sequence_numbers():
    config, rows = stationary_header(), records()
    config['externally_stationary_windows'][0]['last_us'] = 160000
    for row in rows[4:]:
        row['capture_us'] += 20000
        row['receive_ns'] += 20000000
    candidate = audit(config, rows)['stationary_windows'][0]
    assert not candidate['candidate_only']
    assert candidate['coverage']['imu']['max_capture_gap_us'] == 30000
    assert candidate['coverage']['imu']['missing_sequence_numbers'] == 0


def test_many_samples_over_short_span_are_not_long_observation():
    rows = records()
    for i, row in enumerate(rows):
        row['capture_us'] = 100000+i//2
    candidate = audit(stationary_header(), rows)['stationary_windows'][0]
    assert 'imu_insufficient_observed_span' in candidate['rejection_reasons']
    assert not candidate['candidate_only']


def test_outside_window_motion_not_mixed_into_candidate_statistics():
    rows = records()
    rows += [dict(kind='encoder', boot='first', seq=5, capture_us=200000,
                  receive_ns=8100000000, motor_counts=100)]
    result = audit(stationary_header(), rows)
    assert result['stationary_windows'][0]['candidate_only']
    assert result['stationary_windows'][0]['motor_count_span'] == 0
    assert not result['ready_to_drive']


@pytest.mark.parametrize('field,value', [('minimum_span_us', 0), ('minimum_span_us', 40001),
                                       ('max_sample_gap_us', True), ('minimum_imu_samples', 2),
                                       ('minimum_encoder_samples', 1)])
def test_invalid_window_requirements_fail_explicitly(field, value):
    config = stationary_header()
    config['externally_stationary_windows'][0]['review_requirements'][field] = value
    with pytest.raises(ValueError):
        audit(config, records())


@pytest.mark.parametrize('config', [None, [], {'kind': 'header'},
                                  {**header(), 'motor_spec': None},
                                  {**header(), 'externally_stationary_windows': [None]}])
def test_malformed_header_does_not_crash_with_attribute_or_type_error(config):
    with pytest.raises(ValueError):
        audit(config, records())


def test_nonobject_records_and_overflowing_gyro_are_explicit_errors():
    with pytest.raises(ValueError, match='object'):
        audit(header(), [None])
    rows = records()
    rows[1]['gyro_rad_s'][0] = 10**1000
    with pytest.raises(ValueError, match='finite'):
        audit(header(), rows)


def test_derived_encoder_numeric_overflow_is_rejected():
    config = header()
    config['motor_spec']['gear_ratio'] = 1e308
    with pytest.raises(ValueError, match='derived'):
        audit(config, records())


def test_duplicate_json_fields_are_not_silently_last_wins(tmp_path):
    path = tmp_path/'ambiguous.jsonl'
    path.write_text('{"kind":"header","kind":"encoder"}\n')
    with pytest.raises(ValueError, match='duplicate'):
        read_capture(path)


def test_no_gap_in_file_is_not_proof_of_complete_capture():
    result = audit(stationary_header(), records())
    assert result['recorded_sequence_complete']
    assert not result['complete_capture_verified']
    assert not result['ready_to_drive']
