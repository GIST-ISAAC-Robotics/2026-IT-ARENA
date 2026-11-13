"""invalid_encoder는 카운트 손상만 뜻하지 않으며, 첫 고장 라벨은 순서에 의존한다.

10/17 ROS 실패의 원인을 재현했다는 시험이 아니다. 같은 기존 수신 계약에서
오래된 정상 정수 카운트도 해당 고장을 만들고, 후속 신선한 입력이 고장을
자동 해제하지 않는다는 분류/진단 한계를 고정한다. 런타임은 수정하지 않는다.
"""
import pytest

from arena_vehicle_interface.actuation_contract import (
    ActuationReceiver, CommandProducer, ContractConfig, State,
)


def armed_motor_receiver():
    receiver = ActuationReceiver(ContractConfig(
        feedback_mode='drive_motor_shaft', ticks_per_rev=28,
        gear_ratio=15., wheel_radius_m=.0325), boot_id='classification-test')
    for seq, now in enumerate(range(0, 220_001, 10_000)):
        receiver.update_motor_encoder(now, sequence=seq, capture_us=now, motor_counts=9385)
        receiver.tick(now)
    producer = CommandProducer(client_id='test')
    assert receiver.receive(producer.control(receiver.offer(220_000), 'ARM'), 220_000)[0]
    return receiver


@pytest.mark.parametrize('age_us,expected_fault', [(49_999, False), (50_000, True), (175_000, True)])
def test_old_integer_counts_can_trigger_invalid_encoder_before_duplicate_check(age_us, expected_fault):
    receiver = armed_motor_receiver()
    # 과거 표본의 정수 카운트는 정상이다. 동일/과거 시각은 신선도부터 검사된다.
    assert not receiver.update_motor_encoder(225_000, sequence=23,
                                            capture_us=225_000-age_us, motor_counts=9385)
    assert (receiver.state == State.FAULT) == expected_fault
    if expected_fault:
        assert receiver.reason == 'invalid_encoder'
    assert receiver.status()['motor_counts'] == 9385


@pytest.mark.parametrize('ingest_first', [True, False])
def test_first_fault_label_and_deadline_diagnostic_depend_on_call_order(ingest_first):
    receiver = armed_motor_receiver()
    def ingest():
        receiver.update_motor_encoder(395_000, sequence=23, capture_us=230_000, motor_counts=9385)
    if ingest_first:
        ingest()
        receiver.tick(395_000)
    else:
        receiver.tick(395_000)
        ingest()
    assert receiver.state == State.FAULT
    assert receiver.reason == ('invalid_encoder' if ingest_first else 'control_deadline_missed')
    assert (receiver.deadline_diagnostic is None) == ingest_first
    assert receiver.target_speed == 0.


def test_fresh_feedback_after_old_packet_does_not_clear_latched_fault():
    receiver = armed_motor_receiver()
    receiver.update_motor_encoder(225_000, sequence=23, capture_us=175_000, motor_counts=9385)
    assert receiver.reason == 'invalid_encoder'
    assert receiver.update_motor_encoder(225_000, sequence=24, capture_us=225_000, motor_counts=9385)
    receiver.tick(225_000)
    assert receiver.feedback_valid
    assert receiver.state == State.FAULT
    assert receiver.reason == 'invalid_encoder'
    assert receiver.target_speed == 0.
