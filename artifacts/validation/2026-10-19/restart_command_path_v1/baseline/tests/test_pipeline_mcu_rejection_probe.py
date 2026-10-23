"""시험 전용 진단은 원래 수신 결과와 고장 고정 의미를 바꾸지 않는다."""
import pytest

from pipeline_mcu_rejection_probe import install_test_encoder_probe
from test_motor_feedback_fault_classification import armed_motor_receiver
from arena_vehicle_interface.actuation_contract import ActuationReceiver
from validate_pipeline_faults_ros import child_bootstrap, FIXTURE_SOURCES


def test_probe_records_actual_old_sample_not_previous_accepted_sample():
    receiver = armed_motor_receiver()
    original = ActuationReceiver._ingest
    rows = []
    restore = install_test_encoder_probe(ActuationReceiver, rows.append)
    try:
        assert not receiver.update_motor_encoder(225_000, sequence=23, capture_us=175_000, motor_counts=9385)
        assert len(rows) == 1
        row = rows[0]
        assert row['age_us'] == row['feedback_timeout_us'] == 50_000
        assert row['capture_us'] == 175_000
        assert row['last_accepted_feedback_before'][1] == 220_000
        assert row['last_control_us_before'] == 220_000
        assert row['counts'] == [9385]
        assert row['reason_after'] == 'invalid_encoder'
        assert receiver.update_motor_encoder(225_000, sequence=24, capture_us=225_000, motor_counts=9385)
        assert receiver.reason == 'invalid_encoder'
        assert len(rows) == 1
    finally:
        restore()
    assert ActuationReceiver._ingest is original


def test_fresh_call_and_original_result_unchanged():
    receiver, reference = armed_motor_receiver(), armed_motor_receiver()
    expected = reference.update_motor_encoder(225_000, sequence=23, capture_us=225_000, motor_counts=9385)
    rows = []
    restore = install_test_encoder_probe(ActuationReceiver, rows.append)
    try:
        assert receiver.update_motor_encoder(225_000, sequence=23, capture_us=225_000, motor_counts=9385) == expected
        assert receiver.status() == reference.status()
        assert rows == []
    finally:
        restore()


def test_diagnostic_output_failure_does_not_change_fault_return():
    receiver = armed_motor_receiver()
    def failed_output(row):
        raise OSError('closed test log')
    restore = install_test_encoder_probe(ActuationReceiver, failed_output)
    try:
        assert not receiver.update_motor_encoder(225_000, sequence=23, capture_us=175_000, motor_counts=9385)
        assert receiver.reason == 'invalid_encoder'
        assert receiver.target_speed == 0.
    finally:
        restore()


def test_duplicate_install_is_rejected_and_original_exception_is_preserved():
    class Raising:
        state = type('State', (), {'value': 'ARMED'})()
        reason, generation, feedback, last_control_us, armed = 'ready', 1, None, 0, True
        def _ingest(self, *args):
            raise ValueError('original failure')
    restore = install_test_encoder_probe(Raising)
    try:
        with pytest.raises(RuntimeError, match='already installed'):
            install_test_encoder_probe(Raising)
        with pytest.raises(ValueError, match='original failure'):
            Raising()._ingest(0, 0, 0, (0,))
    finally:
        restore()


@pytest.mark.parametrize('role', ['controller', 'safety', 'host', 'mcu', 'sink'])
def test_only_validator_mcu_child_installs_probe_and_source_is_preserved(role):
    source = child_bootstrap(role)
    compile(source, '<validator child>', 'exec')
    assert ('install_test_encoder_probe(ActuationReceiver)' in source) == (role == 'mcu')
    assert 'RUNTIME_SOURCE_MANIFEST' in source
    assert 'scripts/pipeline_mcu_rejection_probe.py' in FIXTURE_SOURCES
