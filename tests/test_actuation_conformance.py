"""포팅 벡터의 재현성과 독립 계약 기대값. 실물/펌웨어를 실행하지 않는다."""
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('actuation_conformance', ROOT/'scripts/actuation_conformance.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


@pytest.fixture(scope='module')
def cases():
    return bench.make_cases()


def select(cases, name):
    return next(c for c in cases if c['name'] == name)


def test_all_cases_have_independent_checkpoints_and_replay(cases):
    assert len(cases) == 19
    assert len({c['name'] for c in cases}) == len(cases)
    for case in cases:
        assert case['checkpoints']
        assert bench.compare_case(case, bench.replay(case)) is None
        for point in case['checkpoints']:
            status = case['events'][point['event_index']]['expected']['status']
            for key, value in point['expected_status'].items():
                assert status[key] == value


def test_deterministic_json_roundtrip(cases):
    assert bench.make_cases() == cases
    restored = json.loads(json.dumps(cases, allow_nan=False))
    for case in restored:
        assert bench.compare_case(case, bench.replay(case)) is None


def test_motor_baseline_matches_vehicle_yaml():
    vehicle = yaml.safe_load((ROOT/'src/arena_description/config/vehicle.yaml').read_text(encoding='utf-8'))['vehicle']
    drive = vehicle['drivetrain']
    cfg = bench.motor_config()
    assert cfg.feedback_mode == 'drive_motor_shaft'
    assert cfg.wheel_radius_m == drive['wheel_radius_m'] == .0325
    assert cfg.gear_ratio == drive['motor']['gear_ratio'] == 15
    assert cfg.ticks_per_rev == vehicle['sensors']['drive_feedback']['drive_motor_encoder']['counts_per_motor_revolution'] == 28


def test_speed_conversion_and_int64_preservation(cases):
    for name, sign in [('signed_motor_counts_not_unsigned_speed', -1),
                       ('int64_counts_above_double_exact_range', 1)]:
        case = select(cases, name)
        status = case['events'][-1]['expected']['status']
        assert status['mean_wheel_mps'] == pytest.approx(sign * 20 * math.pi * .065 / (28*15*.01))
        assert status['left_wheel_mps'] is None and status['right_wheel_mps'] is None
    case = select(cases, 'int64_counts_above_double_exact_range')
    observed = bench.replay(case)
    observed['responses'][-1]['status']['motor_counts'] = float(2**53+131)
    assert 'type mismatch' in bench.compare_case(case, observed)


@pytest.mark.parametrize('name,reason', [
    ('command_timeout_at_100ms', 'command_expired'),
    ('feedback_timeout_at_50ms', 'encoder_timeout'),
    ('control_gap_even_if_feedback_fresh', 'control_deadline_missed'),
    ('fresh_stuck_counts_fault_at_500ms', 'motor_feedback_no_progress'),
    ('clock_regression_latches_fault', 'clock_regressed'),
    ('legacy_feedback_rejected_in_motor_mode', 'feedback_mode_mismatch'),
])
def test_fault_categories_are_not_collapsed(cases, name, reason):
    status = select(cases, name)['events'][-1]['expected']['status']
    assert status['state'] == 'FAULT_LATCHED'
    assert status['reason'] == reason
    assert status['target_speed_mps'] == 0


def test_zero_stop_and_clear_are_distinct(cases):
    zero = select(cases, 'forward_and_zero_is_not_stop')['events'][-1]['expected']['status']
    stop = select(cases, 'software_stop_rejects_queued_drive')['events'][-1]['expected']['status']
    recovery = select(cases, 'clear_requires_new_arm')
    assert zero['state'] == 'ARMED' and stop['state'] == 'ESTOP_LATCHED'
    assert any(p['expected_status'].get('reason') == 'explicit_clear' and
               p['expected_status']['state'] == 'DISARMED' for p in recovery['checkpoints'])


@pytest.mark.parametrize('mutation', ['drop', 'extra', 'rename', 'speed', 'boolean', 'clock_float'])
def test_observation_mismatch_is_rejected(cases, mutation):
    case = select(cases, 'forward_and_zero_is_not_stop')
    observed = bench.replay(case)
    if mutation == 'drop':
        observed['responses'].pop()
    elif mutation == 'extra':
        observed['ignored'] = 1
    elif mutation == 'rename':
        observed['name'] = 'other'
    elif mutation == 'speed':
        observed['responses'][-1]['status']['target_speed_mps'] = 1.
    elif mutation == 'boolean':
        observed['responses'][-1]['status']['generation'] = True
    elif mutation == 'clock_float':
        value = observed['responses'][-1]['status']['feedback_capture_us']
        observed['responses'][-1]['status']['feedback_capture_us'] = float(value)
    assert bench.compare_case(case, observed)


@pytest.mark.parametrize('value', [True, float('nan'), float('inf'), 10**400, '0', None])
def test_numeric_comparison_rejects_nonfinite_or_wrong_type(value):
    assert bench.compare_values(0., value)


def test_float_tolerance_never_applies_to_integer_fields():
    assert bench.compare_values(1., 1.+.5e-9) is None
    assert bench.compare_values(1., 1.+2e-9)
    assert bench.compare_values(1, 1.)
    assert bench.compare_values(True, 1)


def test_wrong_wire_payload_rejected(cases):
    case = deepcopy(select(cases, 'software_stop_rejects_queued_drive'))
    row = next(r for r in case['events'] if r['input']['op'] == 'command')
    row['input']['args']['frame_hex'] = bench.encode({'v': 1, 'kind': 'STOP'}).hex()
    with pytest.raises(ValueError, match='does not match'):
        bench.replay(case)
