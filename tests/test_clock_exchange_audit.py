import copy
import json
import subprocess
import sys

import pytest

from audit_clock_exchanges import SCHEMA, audit_bytes


def rows():
    return [dict(kind='header', schema=SCHEMA, origin='synthetic',
                 host_clock='host_monotonic_ns', device_clock='mcu_monotonic_us',
                 assumptions=dict(max_rate_error_ppm=200, host_timestamp_error_ns=1000,
                                  mcu_timestamp_error_us=1, max_window_us=5_000_000,
                                  max_extrapolation_us=100_000)),
            dict(kind='exchange', boot='boot-A', sequence=0, host_send_ns=8_999_000_000,
                 mcu_receive_us=1_000_000, mcu_send_us=1_000_100, host_receive_ns=9_009_100_000),
            dict(kind='query', boot='boot-A', capture_us=1_000_050)]


def encode(value):
    return ('\n'.join(json.dumps(v) for v in value) + '\n').encode()


def test_audit_is_conditional_interval_not_drive_or_ros_stamp():
    report = audit_bytes(encode(rows()))
    assert report['evidence_consistent'] and report['origin'] == 'synthetic'
    assert report['maximum_interval_width_ns'] > 10_000_000
    assert report['mapped'][0]['host_lower_ns'] <= 9_000_050_000 <= report['mapped'][0]['host_upper_ns']
    for flag in ('ready_to_drive', 'ros_stamp_verified', 'clock_mapping_applied',
                 'assumptions_hardware_verified', 'complete_capture_verified'):
        assert report[flag] is False


@pytest.mark.parametrize('fault', ['unknown_field', 'unknown_kind', 'wrong_clock', 'origin',
                                 'missing_assumption', 'bool_time', 'cross_boot', 'too_far',
                                 'no_query', 'blank', 'duplicate_key', 'nan', 'oversize',
                                 'query_before_exchange', 'bad_utf8', 'too_many_queries'])
def test_reject_malformed_or_unsupported_records(fault):
    value = rows()
    if fault == 'unknown_field': value[1]['rtc'] = 3
    elif fault == 'unknown_kind': value[1]['kind'] = 'ping'
    elif fault == 'wrong_clock': value[0]['host_clock'] = 'ros_time_ns'
    elif fault == 'origin': value[0]['origin'] = 'measured_maybe'
    elif fault == 'missing_assumption': del value[0]['assumptions']['max_window_us']
    elif fault == 'bool_time': value[1]['host_send_ns'] = True
    elif fault == 'cross_boot': value[2]['boot'] = 'boot-B'
    elif fault == 'too_far': value[2]['capture_us'] = 3_000_000
    elif fault == 'no_query': value.pop()
    elif fault == 'query_before_exchange': value.append(copy.deepcopy(value[1]))
    elif fault == 'too_many_queries': value.extend([value[2]] * 64)
    data = encode(value)
    if fault == 'blank': data += b'\n'
    elif fault == 'duplicate_key': data = data.replace(b'"sequence": 0', b'"sequence": 0, "sequence": 0')
    elif fault == 'nan': data = data.replace(b'"sequence": 0', b'"sequence": NaN')
    elif fault == 'oversize': data += b' ' * 262_144
    elif fault == 'bad_utf8': data += b'\xff'
    with pytest.raises(ValueError): audit_bytes(data)


def test_cli_exclusive_output_and_rejection(tmp_path):
    from pathlib import Path
    script = Path(__file__).resolve().parents[1] / 'scripts/audit_clock_exchanges.py'
    capture, output = tmp_path / 'capture.jsonl', tmp_path / 'report.json'
    original = encode(rows())
    capture.write_bytes(original)
    cmd = [sys.executable, str(script), str(capture), '--output', str(output)]
    first = subprocess.run(cmd, capture_output=True, text=True)
    assert first.returncode == 0, first.stderr
    saved = output.read_bytes()
    assert json.loads(saved)['input_unchanged']
    assert subprocess.run(cmd, capture_output=True).returncode != 0
    assert output.read_bytes() == saved and capture.read_bytes() == original
    capture.write_bytes(b'{}\n')
    invalid = subprocess.run(cmd[:-1] + [str(tmp_path / 'bad.json')], capture_output=True)
    assert invalid.returncode == 2
    assert not json.loads((tmp_path / 'bad.json').read_bytes())['evidence_consistent']
