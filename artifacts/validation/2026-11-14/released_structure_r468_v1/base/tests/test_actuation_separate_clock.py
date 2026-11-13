"""별도 시계 후보를 C++ 수신·송신에 제한 교환으로 대조한다."""
import shutil

import pytest

from audit_actuation_separate_clock import CASES, compile_driver, run_case


@pytest.fixture(scope='module')
def binary(tmp_path_factory):
    if not shutil.which('g++'):
        pytest.fail('C++ separate clock integration requires g++')
    return compile_driver(tmp_path_factory.mktemp('separate-clock')/'tx-driver')


@pytest.mark.parametrize('name,options,expected', CASES, ids=[c[0] for c in CASES])
def test_bounded_exchange(binary, name, options, expected):
    result = run_case(binary, options)
    assert result['drive_sent_by_fixture'] is expected, (name, result)
    assert result['final_state'] == 'ESTOP_LATCHED'
    assert result['final_target_mps'] == 0
    assert result['ready_to_drive'] is False and result['hardware_executed'] is False
    assert result['runtime_connected'] is False
    sent = [event for event in result['events'] if event['direction'] == 'host_to_mcu']
    assert len(sent) == (3 if expected else 2)  # ARM, 선택 DRIVE, STOP. 자동 반복하지 않는다.
    if expected:
        assert result['before_stop']['state'] == 'ACTIVE'
        assert result['candidate']['timestamp_policy_satisfied']
        assert result['candidate']['command_created'] is False  # 패킷은 fixture에서만 생성
    if name == 'late_decision':
        assert result['before_stop']['state'] == 'FAULT_LATCHED'
    if name == 'missing_arm_ack':
        assert result['candidate'] is None
