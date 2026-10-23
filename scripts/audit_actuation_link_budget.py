#!/usr/bin/env python3
"""현행 ROS 어댑터의 실제 인코딩 크기를 이용한 8N1 링크 예산. 실물 처리율 아님."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace
import zipfile

from actuation_conformance import ROOT, Builder, digest
from arena_vehicle_interface.actuation_wire import MAX_PAYLOAD, encode

BAUDS = (115200, 230400, 460800, 921600)
OFFER_HZ = DRIVE_HZ = 50.  # 현행 offer 20 ms, /drive/safe 50 Hz 비교 조건.
LEASE_US = 100_000
COMPACT_KEYS = ('type', 'v', 'boot', 'generation', 'state', 'owner', 'lease',
                'lease_issued_us', 'lease_expires_us', 'output_ready')
FILES = [
    'scripts/audit_actuation_link_budget.py', 'tests/test_actuation_link_budget.py',
    'scripts/actuation_conformance.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py',
]


def sample_frames():
    # ROS 노드를 생성/구동하지 않고 운영 콜백을 작은 수집 객체에서 호출한다.
    from ackermann_msgs.msg import AckermannDriveStamped
    from arena_vehicle_interface.actuation_ros import ActuationHost, VirtualMCU
    rows = []
    for name, offset in [('near_boot', 0), ('after_100_seconds', 100_000_000),
                         ('after_24_hours', 86_400_000_000)]:
        b = Builder(name)
        b.receiver.boot_id = 'b' * 32
        b.producer.client = 'c' * 32
        b.now = offset
        b.prepare()
        b.advance(10_000, increment=20)
        b.drive(1.)
        offers = []
        mcu = SimpleNamespace(receiver=b.receiver, now_us=lambda: b.now,
                              output_ready=lambda: True, rx=None,
                              send=lambda _, payload: offers.append(deepcopy(payload)))
        VirtualMCU.send_offer(mcu)
        offer = offers[0]
        commands = []
        host = SimpleNamespace(producer=b.producer, offer=offer, arm_barrier=offset+220_000,
                               tx=None, now_us=lambda: b.now, fresh_offer=lambda: True,
                               send=lambda _, payload: commands.append(deepcopy(payload)))
        message = AckermannDriveStamped()
        message.header.stamp.sec = b.now // 1_000_000
        message.header.stamp.nanosec = (b.now % 1_000_000) * 1000
        message.drive.speed, message.drive.steering_angle = 1.2, .2
        ActuationHost.on_drive(host, message)
        if len(commands) != 1:
            raise AssertionError('expected actual host DRIVE envelope')
        command = commands[0]
        compact = {key: offer[key] for key in COMPACT_KEYS}
        # STOP 봉투는 control()의 구조를 크기 비교용으로 명시. 서비스/ACK 실행은 아님.
        stop = {'type': 'command', 'request': 'r' * 32, 'command': {'v': 1, 'kind': 'STOP'}}
        payloads = dict(offer=offer, drive=command, service_stop_shape=stop,
                        compact_offer_projection_not_implemented=compact)
        rows.append(dict(name=name, now_us=b.now, payloads=payloads,
                         frame_bytes={key: len(encode(value)) for key, value in payloads.items()}))
    return rows


def budget(offer_bytes, drive_bytes, baud, *, offer_hz=OFFER_HZ, drive_hz=DRIVE_HZ):
    """전이중 UART 각 방향을 따로 계산. 시작 1 + data 8 + 정지 1 = 10 bit/byte."""
    if min(offer_bytes, drive_bytes, baud, offer_hz, drive_hz) <= 0:
        raise ValueError('positive sizes/rates required')
    tx_us = drive_bytes * 10 * 1_000_000 / baud
    rx_us = offer_bytes * 10 * 1_000_000 / baud
    tx_load, rx_load = drive_bytes * 10 * drive_hz / baud, offer_bytes * 10 * offer_hz / baud
    return dict(baud=baud, drive_frame_ms=tx_us/1000, offer_frame_ms=rx_us/1000,
                host_to_mcu_load=tx_load, mcu_to_host_load=rx_load,
                both_directions_strictly_below_capacity=max(tx_load, rx_load) < 1.,
                zero_queue_offer_then_drive_ms=(tx_us+rx_us)/1000,
                remaining_lease_zero_queue_ms=(LEASE_US-tx_us-rx_us)/1000,
                required_baud_at_unit_load=max(drive_bytes*10*drive_hz, offer_bytes*10*offer_hz))


def build_report():
    samples = sample_frames()
    sizes = {name: max(row['frame_bytes'][name] for row in samples)
             for name in samples[0]['frame_bytes']}
    variants = {}
    for name, offer in [('current_full_offer', sizes['offer']),
                        ('compact_offer_projection_not_implemented', sizes['compact_offer_projection_not_implemented'])]:
        variants[name] = [budget(offer, sizes['drive'], baud) for baud in BAUDS]
    return dict(scope='PC encoded size arithmetic; no UART or USB throughput measured',
                assumptions=dict(bits_per_byte=10, uart='8N1 full duplex',
                                 offer_hz=OFFER_HZ, drive_hz=DRIVE_HZ, lease_us=LEASE_US,
                                 zero_queue_host_processing_us=0, identifiers='fixed 32 ASCII characters',
                                 other_telemetry_acks_logs_excluded=True),
                samples=samples, sample_max_frame_bytes=sizes, variants=variants,
                max_legal_frame_bytes=MAX_PAYLOAD+8,
                max_legal_frame_serialization_ms={str(b): (MAX_PAYLOAD+8)*10*1000/b for b in BAUDS},
                measured_baud_support=False, native_usb_budget_inferred=False,
                worst_case_all_field_widths_proven=False, transport_changed=False,
                firmware_verified=False, ready_to_drive=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = {p: digest(ROOT/p) for p in FILES}
    report = build_report()
    report.update(source_sha256=before, source_unchanged=all(digest(ROOT/p)==v for p,v in before.items()))
    with (args.output/'report.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.write(ROOT/path, path)
    print(json.dumps({key: report[key] for key in ('sample_max_frame_bytes', 'variants', 'source_unchanged')}))
    return 0 if report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
