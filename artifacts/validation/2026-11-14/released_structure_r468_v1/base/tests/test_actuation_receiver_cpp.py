"""C++ 수신 상태기계와 기존 Python ActuationReceiver 이벤트별 대조. 보드/ROS/전기 출력 검증 아님."""
import importlib.util
import os
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
spec = importlib.util.spec_from_file_location('validate_actuation_receiver_cpp',
                                              ROOT/'scripts/validate_actuation_receiver_cpp.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


@pytest.fixture(scope='module')
def plan():
    return bench.build_plan()


@pytest.fixture(scope='module')
def cpp_result(plan):
    path = os.environ.get('ARENA_RECEIVER_CPP_BINARY')
    if not path:
        pytest.skip('C++ receiver requires explicit prebuilt ARENA_RECEIVER_CPP_BINARY')
    return bench.evaluate(Path(path), plan)


def test_plan_reuses_existing_vectors_and_adds_manual_cases(plan):
    kinds = {}
    for item in plan:
        kinds[item['kind']] = kinds.get(item['kind'], 0) + 1
    assert kinds == {'contract': 19 + len(bench.manual_cases()) + len(bench.legacy_cases()), 'transport': 12, 'link': 10}
    names = [item['name'] for item in plan]
    assert len(names) == len(set(names))
    for required in ('wrong_generation_and_invalid_drive_is_session', 'expired_lease_and_invalid_drive_is_expired_lease',
                     'wrong_owner_and_invalid_drive_is_not_owner', 'old_seq_and_invalid_drive_is_sequence',
                     'valid_owner_invalid_drive_faults_and_zeroes_target', 'stop_at_same_time_as_command_deadline',
                     'physical_release_then_stale_clear_and_arm', 'lease_capacity_evicts_oldest',
                     'int64_count_from_min_to_max'):
        assert 'manual_' + required in names


def test_manual_checkpoints_hold_in_python(plan):
    for item in plan:
        for checkpoint in item['case']['checkpoints']:
            status = item['case']['events'][checkpoint['event_index']]['expected']['status']
            for key, value in checkpoint['expected_status'].items():
                assert status[key] == value, (item['name'], key)


def test_legacy_mode_keeps_left_right_and_motor_mode_has_none(plan):
    legacy = next(i for i in plan if i['name'] == 'legacy_legacy_drive_and_left_right')
    status = legacy['case']['events'][-1]['expected']['status']
    assert status['feedback_mode'] == 'rear_wheel_pair_legacy' and status['left_wheel_mps'] is not None
    motor = next(i for i in plan if i['name'] == 'conformance_boot_requires_explicit_arm')
    status = motor['case']['events'][-1]['expected']['status']
    assert status['left_wheel_mps'] is None and status['right_wheel_mps'] is None


def test_config_is_motor_shaft_reference(plan):
    case = next(i for i in plan if i['name'] == 'conformance_boot_requires_explicit_arm')['case']
    assert (case['config']['ticks_per_rev'], case['config']['gear_ratio'], case['config']['wheel_radius_m']) == (28, 15., .0325)


def test_cpp_matches_python_event_by_event(cpp_result):
    report, stdin, stdout, stderr = cpp_result
    assert report['passed'], (report['mismatches'][:3], stderr[:1000])
    assert report['binary_exit_code'] == 0 and not stderr
    assert report['events_compared'] > 8000 and report['receive_cases'] == 3667


def test_cpp_boundaries_are_explicit_divergences(cpp_result):
    rows = {row['name']: row for row in cpp_result[0]['boundary_checks']}
    assert all(row['passed'] for row in rows.values())
    # Python은 시계 상한 초과와 세대 2^63 진입을 허용한다. C++은 wrap 대신 고장으로 고정한다.
    assert rows['now_above_limit_invalid_clock']['python_same_input']['state'] == 'DISARMED'
    assert rows['generation_exhausted_latches']['python_same_input']['generation'] == 2**63


def test_cpp_report_scope(cpp_result):
    report = cpp_result[0]
    assert report['state_machine_ported'] is True
    for field in ('ros_or_runtime_connected', 'wcet_measured', 'mcu_toolchain_verified', 'firmware_verified',
                  'ready_to_drive'):
        assert report[field] is False
