#!/usr/bin/env python3
"""실제 ROS 콜백의 송신 횟수·크기 감사. DDS/직렬 장치 없이 합성 시간으로 콜백만 호출한다."""
import argparse
from collections import Counter, deque
from copy import deepcopy
import json
from pathlib import Path
from types import MethodType, SimpleNamespace
import zipfile

from actuation_conformance import ROOT, Builder, digest
from audit_actuation_link_budget import BAUDS
from arena_vehicle_interface.actuation_wire import Decoder, encode

FILES = [
    'scripts/audit_actuation_callback_budget.py', 'tests/test_actuation_callback_budget.py',
    'scripts/actuation_conformance.py', 'scripts/audit_actuation_link_budget.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py',
]
SCENARIOS = ('idle', 'drive50', 'duplicate50', 'drive50_ack')


def sample_schedule(scenario):
    from ackermann_msgs.msg import AckermannDriveStamped
    from arena_vehicle_interface.actuation_ros import ActuationHost, VirtualMCU
    if scenario not in SCENARIOS:
        raise ValueError('unknown callback schedule')
    b = Builder(scenario)
    b.receiver.boot_id = 'b' * 32
    b.producer.client = 'c' * 32
    b.prepare(arm=scenario != 'idle')
    now = b.now
    source = 'setup'
    rows, representative = [], {}
    channels = Counter()
    rejected = 0
    previous_wire = None

    def publish_status():
        channels['status_ros_only'] += 1
        return b.receiver.status()

    mcu = SimpleNamespace(receiver=b.receiver, decoder=Decoder(), now_us=lambda: now,
                          output_ready=lambda: True, rx=None, clock_failed=False, timer_entry=None,
                          last_now=now, last_tick=now, last_offer=now, clock_started=True,
                          progress_wall=0., wall_timeout=3., last_fault_publish_wall=0.,
                          wheels=deque(), feedback_seq=b.seq, motor_feedback=True,
                          receiver_ticks=deque(maxlen=16), publish_status=publish_status,
                          capture_deadline=lambda *args: None)
    mcu.send_offer = MethodType(VirtualMCU.send_offer, mcu)
    host = SimpleNamespace(producer=b.producer, decoder=Decoder(), offer=None, offer_wall=0., pending={},
                           arm_barrier=now, tx=None, now_us=lambda: now, wall_timeout=3.)
    host.fresh_offer = MethodType(ActuationHost.fresh_offer, host)

    def mcu_send(_, payload):
        wire = encode(payload)
        if source != 'setup':
            rows.append(dict(now_us=now, direction='mcu_to_host', source=source,
                             kind=payload['type'], bytes=len(wire)))
            representative.setdefault(source + '_' + payload['type'], deepcopy(payload))
        ActuationHost.on_rx(host, SimpleNamespace(data=wire))

    def host_send(_, payload):
        nonlocal previous_wire, rejected
        if scenario == 'drive50_ack':
            payload = {**payload, 'request': 'r' * 32}
        wire = encode(payload)
        if scenario == 'duplicate50' and previous_wire is not None:
            wire = previous_wire
        previous_wire = wire
        rows.append(dict(now_us=now, direction='host_to_mcu', source=source, kind='command', bytes=len(wire)))
        before = b.receiver.sequence
        VirtualMCU.on_tx(mcu, SimpleNamespace(data=wire))
        if b.receiver.sequence == before:
            rejected += 1

    mcu.send, host.send = mcu_send, host_send
    mcu.send_offer()  # 기준 시각의 최초 offer는 1초 관측 구간에서 제외한다.
    start = now
    for step in range(1, 201):
        now = start + step * 5000
        if step % 2 == 0:
            count = 0 if scenario == 'idle' else step // 2 * 10
            mcu.wheels.append((now, count))
        source = 'timer'
        VirtualMCU.control_tick(mcu)
        if step % 4 == 0 and scenario != 'idle':
            source = 'on_tx'
            if scenario == 'duplicate50' and previous_wire is not None:
                # 명시한 고장 입력: 이전 wire를 반복 배송한다. 실제 Host는 고장 offer 뒤 송신을 중단한다.
                host_send(None, {})
                continue
            message = AckermannDriveStamped()
            message.header.stamp.sec, microseconds = divmod(now, 1000000)
            message.header.stamp.nanosec = microseconds * 1000
            message.drive.speed, message.drive.steering_angle = .5, .2
            ActuationHost.on_drive(host, message)
    counts = Counter((row['direction'], row['source'], row['kind']) for row in rows)
    summary = [dict(direction=d, source=s, kind=k, frames=v,
                    bytes=sum(row['bytes'] for row in rows if (row['direction'], row['source'], row['kind']) == (d, s, k)))
               for (d, s, k), v in sorted(counts.items())]
    byte_counts = {direction: sum(row['bytes'] for row in rows if row['direction'] == direction)
                   for direction in ('host_to_mcu', 'mcu_to_host')}
    return dict(scenario=scenario, duration_us=now-start, trace=rows, summary=summary,
                byte_counts=byte_counts, representative_payloads=representative,
                rejected_command_callbacks=rejected, final_state=b.receiver.state.value,
                final_reason=b.receiver.reason, non_wire_publications=dict(channels),
                baud_load=[dict(baud=baud, **{direction: count*10/baud for direction, count in byte_counts.items()})
                           for baud in BAUDS])


def build_report():
    return dict(scope='actual callback invocation counts under a synthetic 1-second schedule; no ROS nodes or UART',
                assumptions=dict(timer_interval_us=5000, encoder_interval_us=10000, drive_interval_us=20000,
                                 full_duplex_8n1=True, clocks_shared_only_in_this_sim_fixture=True,
                                 output_ready_stubbed_true=True, status_topic_not_serialized=True,
                                 host_and_mcu_delivery_immediate=True, no_transport_queue=True,
                                 duplicate_scenario_replays_old_wire_not_nominal_host=True),
                scenarios=[sample_schedule(s) for s in SCENARIOS],
                runtime_changed=False, real_transport_measured=False, deadline_proven=False,
                firmware_verified=False, ready_to_drive=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = {path: digest(ROOT / path) for path in FILES}
    report = build_report()
    report.update(source_sha256=before, source_unchanged=all(digest(ROOT / path) == value for path, value in before.items()))
    with (args.output / 'report.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output / 'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.write(ROOT / path, path)
    print(json.dumps([dict(scenario=s['scenario'], summary=s['summary'], final_state=s['final_state'],
                           rejected=s['rejected_command_callbacks'], baud_load=s['baud_load']) for s in report['scenarios']]))
    return 0 if report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
