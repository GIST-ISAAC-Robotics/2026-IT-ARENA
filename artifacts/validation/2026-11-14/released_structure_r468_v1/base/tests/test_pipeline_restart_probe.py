"""재시작 구간 명령 경로 진단은 원래 수락/거절·고장 고정 결과를 바꾸지 않는다.

v6 명령 만료의 원인을 재현하는 시험이 아니다. 진단이 실제 호출 결과와 같은 순서로
기록되는지, 기록 실패가 판정을 바꾸지 않는지만 고정한다.
"""
from types import SimpleNamespace

import pytest

from pipeline_restart_probe import (
    classify_host_drive, install_test_host_drive_probe, install_test_mcu_command_probe,
    install_test_publish_timing_probe,
)
from arena_vehicle_interface.actuation_contract import (
    ActuationReceiver, CommandProducer, ContractConfig, State,
)


def armed(producer):
    receiver = ActuationReceiver(ContractConfig(
        feedback_mode='drive_motor_shaft', ticks_per_rev=28,
        gear_ratio=15., wheel_radius_m=.0325), boot_id='restart-probe-test')
    for seq, now in enumerate(range(0, 220_001, 10_000)):
        receiver.update_motor_encoder(now, sequence=seq, capture_us=now, motor_counts=9204)
        receiver.tick(now)
    assert receiver.receive(producer.control(receiver.offer(220_000), 'ARM'), 220_000)[0]
    return receiver


def expiry_sequence(receiver, producer):
    """신선한 피드백과 5 ms tick을 유지하고 마지막 0 명령 뒤 명령만 멈춘다."""
    results = []
    offer = receiver.offer(225_000)
    frame = producer.drive(offer, speed_mps=0., steering_rad=0., source_us=224_000, host_now_us=225_000)
    results.append(receiver.receive(frame, 226_000))
    for seq, now in enumerate(range(230_000, 360_001, 5_000), start=100):
        results.append(receiver.update_motor_encoder(now, sequence=seq, capture_us=now, motor_counts=9204))
        results.append(receiver.tick(now)['state'])
    return results


def test_command_expiry_context_records_last_accepted_drive_without_changing_result():
    reference_producer = CommandProducer(client_id='host')
    reference = armed(reference_producer)
    expected = expiry_sequence(reference, reference_producer)
    rows = []
    restore = install_test_mcu_command_probe(ActuationReceiver, rows.append)
    try:
        producer = CommandProducer(client_id='host')
        receiver = armed(producer)
        assert expiry_sequence(receiver, producer) == expected
        assert receiver.status() == reference.status()
    finally:
        restore()
    assert receiver.state == State.FAULT and receiver.reason == 'command_expired'
    assert len(rows) == 1
    row = rows[0]
    assert row['reason'] == 'command_expired'
    drive = row['last_accepted_drive']
    assert drive['kind'] == 'DRIVE' and drive['accepted'] is True and drive['now_us'] == 226_000
    # 기존 수명식: min(임대 만료, 임대 발행 + ttl - 원천 나이)
    assert drive['deadline_after_us'] == min(drive['lease_window_before'][1],
                                             drive['lease_window_before'][0] + 100_000 - 1_000)
    assert row['before']['deadline_us'] == drive['deadline_after_us']
    assert row['before']['state'] == 'ARMED' and row['generation_after'] == row['before']['generation'] + 1
    ticks = row['recent_ticks']
    after_drive = [t for t in ticks if t['now_us'] > drive['now_us']]
    assert after_drive and all(t['state'] == 'ARMED' and t['deadline_us'] == drive['deadline_after_us'] for t in after_drive)
    assert ticks[-1]['now_us'] < drive['deadline_after_us'] <= row['before']['now_us']
    assert ActuationReceiver.receive.__name__ == 'receive'


def test_no_context_for_explicit_stop_or_already_latched_fault():
    rows = []
    restore = install_test_mcu_command_probe(ActuationReceiver, rows.append)
    try:
        producer = CommandProducer(client_id='host')
        receiver = armed(producer)
        assert receiver.receive({'v': 1, 'kind': 'STOP'}, 230_000) == (True, 'software_stop')
        receiver._fault('command_expired')
        assert receiver.reason == 'software_stop'
        assert rows == []
        latched = armed(CommandProducer(client_id='host'))
        latched._fault('invalid_encoder')
        latched._fault('command_expired')
        assert [row['reason'] for row in rows] == ['invalid_encoder']
    finally:
        restore()


def test_mcu_output_failure_does_not_change_fault_and_duplicate_install_is_rejected():
    def failed_output(row):
        raise OSError('closed test log')
    restore = install_test_mcu_command_probe(ActuationReceiver, failed_output)
    try:
        with pytest.raises(RuntimeError, match='already installed'):
            install_test_mcu_command_probe(ActuationReceiver)
        producer = CommandProducer(client_id='host')
        receiver = armed(producer)
        expiry_sequence(receiver, producer)
        assert receiver.reason == 'command_expired' and receiver.target_speed == 0.
    finally:
        restore()
    assert not hasattr(ActuationReceiver.tick, '_pipeline_restart_probe')


class FakeHost:
    """실제 ActuationHost 판단 함수를 그대로 쓰는 rclpy 없는 대역."""
    def __init__(self, actual, *, now=1_000_000):
        self.on_drive_impl = actual
        self.clock = now
        self.wall_timeout = 3.
        self.offer, self.offer_wall, self.arm_barrier = None, 0., None
        self.producer = CommandProducer(client_id='host')
        self.sent, self.tx = [], None

    def now_us(self):
        return self.clock

    def send(self, publisher, payload):
        self.sent.append(payload)


def stamped(source_us, speed=0.):
    stamp = SimpleNamespace(sec=source_us // 1_000_000, nanosec=source_us % 1_000_000 * 1000)
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp),
                           drive=SimpleNamespace(speed=speed, steering_angle=0.))


@pytest.fixture
def host_type():
    actuation_ros = pytest.importorskip('arena_vehicle_interface.actuation_ros')
    actual = actuation_ros.ActuationHost

    class Host(FakeHost):
        on_drive = actual.on_drive
        on_rx = actual.on_rx
        fresh_offer = actual.fresh_offer
    return Host


def ready(host, *, issued=990_000, output_ready=True, owner='host', state='ARMED', barrier=950_000):
    host.offer = dict(boot='b', generation=1, lease=7, lease_issued_us=issued, owner=owner,
                      state=state, output_ready=output_ready)
    host.offer_wall = __import__('time').monotonic()
    host.arm_barrier = barrier


@pytest.mark.parametrize('label,setup,source', [
    ('offer_missing', lambda h: None, 999_000),
    ('offer_from_future', lambda h: ready(h, issued=1_000_001), 999_000),
    ('offer_stale_sim', lambda h: ready(h, issued=900_000), 999_000),
    ('output_not_ready', lambda h: ready(h, output_ready=False), 999_000),
    ('arm_barrier_missing', lambda h: ready(h, barrier=None), 999_000),
    ('source_before_arm_barrier', lambda h: ready(h, barrier=999_500), 999_000),
    ('source_age_outside_ttl', lambda h: ready(h, barrier=800_000), 900_000),
    ('offer_not_owner_or_armed', lambda h: ready(h, owner='other'), 999_000),
    ('forward_expected', lambda h: ready(h), 999_000),
])
def test_host_label_matches_actual_forwarding_and_side_effects(host_type, label, setup, source):
    reference, traced = host_type(None), host_type(None)
    setup(reference)
    setup(traced)
    traced.offer_wall = reference.offer_wall
    reference.on_drive(stamped(source))
    rows = []
    restore = install_test_host_drive_probe(host_type, rows.append)
    try:
        traced.on_drive(stamped(source))
    finally:
        restore()
    assert [frame['command'] for frame in traced.sent] == [frame['command'] for frame in reference.sent]
    assert rows[0]['label'] == label
    assert rows[0]['forwarded'] is (label == 'forward_expected') is bool(traced.sent)


def test_host_repeated_source_is_named_and_only_transitions_are_logged(host_type):
    host = host_type(None)
    ready(host)
    rows = []
    restore = install_test_host_drive_probe(host_type, rows.append)
    try:
        host.on_drive(stamped(999_000))
        host.clock += 20_000
        ready(host, issued=1_015_000)
        host.on_drive(stamped(1_019_000))
        host.clock += 40_000
        ready(host, issued=1_055_000)
        host.on_drive(stamped(1_059_000))
        host.on_drive(stamped(1_059_000))
        assert classify_host_drive(host, host.clock, 1_059_000, host.offer_wall) == 'source_not_increasing'
    finally:
        restore()
    assert [(row['label'], row['forwarded']) for row in rows] == [
        ('forward_expected', True), ('source_not_increasing', False)]
    previous = rows[1]['previous_run']
    assert previous['count'] == 3 and previous['max_call_gap_us'] == 40_000
    assert len(host.sent) == 3
    # 전달된 호출은 원천 watermark를 바꾼다. 호출 전 값은 바뀌기 전 스칼라 복사본이어야 한다.
    assert rows[0]['last_source_us_before'] is None and rows[0]['last_source_us_after'] == 999_000
    assert rows[1]['last_source_us_before'] == rows[1]['last_source_us_after'] == 1_059_000


def test_host_before_value_is_copied_before_forwarding_changes_producer(host_type):
    host = host_type(None)
    ready(host)
    host.on_drive(stamped(990_000))
    rows = []
    restore = install_test_host_drive_probe(host_type, rows.append)
    try:
        host.clock += 10_000
        ready(host, issued=host.clock - 5_000)
        host.on_drive(stamped(1_009_000))
    finally:
        restore()
    assert rows[0]['forwarded'] is True
    assert rows[0]['last_source_us_before'] == 990_000 != rows[0]['last_source_us_after'] == 1_009_000


def test_host_log_limit_and_duplicate_install(host_type):
    host = host_type(None)
    rows = []
    restore = install_test_host_drive_probe(host_type, rows.append, limit=2)
    try:
        with pytest.raises(RuntimeError, match='already installed'):
            install_test_host_drive_probe(host_type)
        for index in range(5):
            if index % 2:
                ready(host, issued=host.clock - 5_000)
            else:
                host.offer = None
            host.on_drive(stamped(host.clock - 1_000))
            host.clock += 10_000
    finally:
        restore()
    assert len(rows) == 3 and rows[-1]['type'] == 'test_host_drive_decision_truncated'
    assert not hasattr(host_type.on_drive, '_pipeline_restart_probe')


def test_publish_timing_keeps_result_and_exception_and_logs_only_slow_calls(monkeypatch):
    import pipeline_restart_probe as probe
    wall = [10.]
    monkeypatch.setattr(probe.time, 'monotonic', lambda: wall[0])

    class Publisher:
        topic_name = '/sim/cmd_vel'

        def publish(self, msg):
            wall[0] += abs(msg)
            if msg < 0:
                raise TypeError('original failure')
            return 'sent'
    rows = []
    restore = install_test_publish_timing_probe(Publisher, rows.append, threshold_s=.015, limit=2)
    try:
        with pytest.raises(RuntimeError, match='already installed'):
            install_test_publish_timing_probe(Publisher)
        assert Publisher().publish(.001) == 'sent'
        assert rows == []
        assert Publisher().publish(.3) == 'sent'
        with pytest.raises(TypeError, match='original failure'):
            Publisher().publish(-.001)
        assert [row['duration_s'] for row in rows] == [pytest.approx(.3)]
        assert rows[0]['topic'] == '/sim/cmd_vel' and rows[0]['start_wall_s'] == pytest.approx(10.001)
        for _ in range(3):
            Publisher().publish(.02)
        assert len(rows) == 3 and rows[-1]['type'] == 'test_slow_publish_truncated'
    finally:
        restore()
    assert not hasattr(Publisher.publish, '_pipeline_restart_probe')
