"""ROS 노드 경계의 순수 콜백 회귀. ROS가 없는 PC에서는 이 파일만 건너뛴다."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
pytest.importorskip('rclpy')
from ackermann_msgs.msg import AckermannDriveStamped

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/arena_vehicle_interface'))
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer
from arena_vehicle_interface.actuation_ros import ActuationHost, VirtualMCU, stamp_us


def host():
    sent = []
    producer = CommandProducer(client_id='host')
    obj = SimpleNamespace(producer=producer, arm_barrier=100_000, tx=None,
                          now_us=lambda: 120_000, fresh_offer=lambda: True,
                          offer=dict(owner='host', state='ARMED', output_ready=True, boot='a', generation=1, lease=1),
                          send=lambda _, payload: sent.append(payload))
    return obj, sent


def drive(stamp=110_000, speed=.5):
    message = AckermannDriveStamped()
    message.header.stamp.sec = stamp // 1_000_000
    message.header.stamp.nanosec = stamp % 1_000_000 * 1000
    message.drive.speed = speed
    return message


def test_host_keeps_original_source_age():
    obj, sent = host()
    ActuationHost.on_drive(obj, drive())
    assert sent[0]['command']['source_age_us'] == 10_000


def test_host_does_not_refresh_duplicate():
    obj, sent = host()
    ActuationHost.on_drive(obj, drive())
    ActuationHost.on_drive(obj, drive())
    assert len(sent) == 1


@pytest.mark.parametrize('stamp', [50_000, 0, 121_000])
def test_host_rejects_pre_arm_or_future_source(stamp):
    obj, sent = host()
    ActuationHost.on_drive(obj, drive(stamp))
    assert not sent


def test_host_waits_for_output_arm_ack():
    obj, sent = host()
    obj.offer['output_ready'] = False
    ActuationHost.on_drive(obj, drive())
    assert not sent
    obj.offer['output_ready'] = True
    ActuationHost.on_drive(obj, drive())
    assert len(sent) == 1


def test_host_new_session_does_not_borrow_previous_owner():
    obj, sent = host()
    obj.offer['owner'] = 'old_host'
    ActuationHost.on_drive(obj, drive())
    assert not sent


def test_mcu_output_ready_needs_matching_recent_epoch():
    receiver = ActuationReceiver(boot_id='a')
    obj = SimpleNamespace(receiver=receiver, now_us=lambda: 200_000, output_ack=None)
    assert not VirtualMCU.output_ready(obj)
    obj.output_ack = dict(epoch=['a', 0], now_us=199_000, blocked=True)
    assert VirtualMCU.output_ready(obj)  # DISARMED 상태에서는 출력 금지 확인이 준비 조건이다.
    for epoch, stamp in [(['old', 0], 199_000), (['a', 1], 199_000), (['a', 0], 100_000), (['a', 0], 201_000)]:
        obj.output_ack.update(epoch=epoch, now_us=stamp)
        assert not VirtualMCU.output_ready(obj)


def test_nan_command_serialization_is_not_published():
    from arena_vehicle_interface.actuation_wire import encode
    obj, sent = host()
    def send(_, payload):
        encode(payload)
        sent.append(payload)
    obj.send = send
    ActuationHost.on_drive(obj, drive(speed=float('nan')))
    assert not sent


def test_stamp_conversion_keeps_capture_time():
    assert stamp_us(drive(1_234_567).header.stamp) == 1_234_567


def test_host_clock_regression_rotates_identity_and_disarms():
    obj, sent = host()
    obj.last_now = 130_000
    obj.pending = {}
    old_identity = obj.producer.client
    ActuationHost.watch(obj)
    assert obj.producer.client != old_identity
    assert obj.arm_barrier is None and obj.offer is None
    assert sent[0]['command'] == {'v': 1, 'kind': 'STOP'}


def test_malformed_ack_is_ignored_without_crashing_host():
    from arena_vehicle_interface.actuation_wire import Decoder, encode
    obj, _ = host()
    obj.decoder, obj.pending = Decoder(), {}
    ActuationHost.on_rx(obj, SimpleNamespace(data=encode({'type': 'ack', 'request': []})))
    assert obj.pending == {}
