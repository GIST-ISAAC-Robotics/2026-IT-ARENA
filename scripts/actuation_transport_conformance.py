#!/usr/bin/env python3
"""바이트 프레임→MCU 참조 계약의 PC 대조. UART/펌웨어/실제 제동 검증 아님."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import struct
import zipfile

from actuation_conformance import (
    ROOT, FILES as CONTRACT_FILES, Builder, apply_event, compare_values, digest,
)
from arena_vehicle_interface.actuation_contract import ActuationReceiver, ContractConfig
from arena_vehicle_interface.actuation_wire import Decoder, MAGIC, MAX_PAYLOAD, encode

STOP = {"v": 1, "kind": "STOP"}
FILES = list(dict.fromkeys(CONTRACT_FILES + [
    'scripts/actuation_transport_conformance.py', 'tests/test_actuation_transport_conformance.py',
]))


def apply_transport_event(receiver, decoder, config, event):
    """각 chunk가 완성한 메시지만 순서대로 전달한다. tick/피드백은 별도 이벤트다.

    feed/reset은 제어 시계를 갱신하거나 만료 타이머를 대신 실행하지 않는다.
    실제 I/O 작업·큐 우선순위·부분 프레임 타이머를 흉내 내지 않는다.
    """
    op, now, args = event['op'], event['now_us'], event['args']
    if op == 'bytes':
        deliveries = []
        for message in decoder.feed(bytes.fromhex(args['hex'])):
            result = list(receiver.receive(message, now))
            deliveries.append(dict(message=message, result=result, status=receiver.status()))
        result = deliveries
    elif op == 'decoder_reset':
        decoder.reset()
        result = None
    else:
        receiver, response = apply_event(receiver, config, event)
        result = response['result']
        if op == 'reboot':
            decoder = Decoder()
    response = dict(result=result, status=receiver.status(),
                    decoder=dict(buffer_hex=bytes(decoder.buffer).hex(),
                                 buffered_bytes=len(decoder.buffer), errors=decoder.errors))
    return receiver, decoder, response


class TransportBuilder(Builder):
    def __init__(self, name):
        super().__init__(name)
        self.decoder = Decoder()
        self.transport_checkpoints = []

    def event(self, op, now=None, **args):
        if now is not None:
            self.now = now
        event = dict(op=op, now_us=self.now, args=args)
        self.receiver, self.decoder, response = apply_transport_event(
            self.receiver, self.decoder, self.config, event)
        self.events.append(dict(input=deepcopy(event), expected=deepcopy(response)))
        return response['result']

    def feed(self, data):
        return self.event('bytes', hex=data.hex())

    def send(self, message):
        deliveries = self.feed(encode(message))
        if len(deliveries) != 1:
            raise AssertionError('fixture expected one complete frame')
        return deliveries[0]['result']

    def check_transport(self, *, deliveries=None, buffered=None, min_errors=None):
        response = self.events[-1]['expected']
        expected = {}
        if deliveries is not None:
            observed = [item['result'] for item in response['result']]
            if observed != deliveries:
                raise AssertionError((self.name, observed, deliveries))
            expected['deliveries'] = deliveries
        if buffered is not None:
            if response['decoder']['buffered_bytes'] != buffered:
                raise AssertionError((self.name, 'buffer', buffered, response['decoder']))
            expected['buffered_bytes'] = buffered
        if min_errors is not None:
            if response['decoder']['errors'] < min_errors:
                raise AssertionError((self.name, 'decoder error not recorded'))
            expected['min_errors'] = min_errors
        self.transport_checkpoints.append(dict(event_index=len(self.events)-1, expected=expected))

    def finish(self):
        case = super().finish()
        case['transport_checkpoints'] = self.transport_checkpoints
        return case


def active(name):
    b = TransportBuilder(name).prepare()
    packet = b.drive(1.)
    b.check(state='ACTIVE', target_speed_mps=1., deadline_us=320_000)
    return b, packet


def blocked_stop_prefix():
    # 길이 값은 허용 범위지만 그 길이를 채우는 본문은 도착하지 않는다.
    return MAGIC + struct.pack('<H', MAX_PAYLOAD) + encode(STOP)


def make_cases():
    cases = []

    b, _ = active('fragmented_drive_keeps_issue_deadline')
    frame = encode(b.packet(.7, age=20_000))
    b.feed(frame[:17]); b.check_transport(deliveries=[], buffered=17)
    b.advance(30_000)
    b.feed(frame[17:]); b.check_transport(deliveries=[[True, 'drive']], buffered=0)
    b.check(state='ACTIVE', target_speed_mps=.7, deadline_us=300_000)
    b.advance(40_000); b.check(state='ACTIVE', deadline_us=300_000)
    b.advance(10_000); b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('coalesced_stop_rejects_queued_drive')
    first, queued = b.packet(.5), b.packet(.9)
    b.feed(encode(first) + encode(STOP) + encode(queued))
    b.check_transport(deliveries=[[True, 'drive'], [True, 'software_stop'], [False, 'session']], buffered=0)
    b.check(state='ESTOP_LATCHED', reason='software_stop', target_speed_mps=0.)
    cases.append(b.finish())

    b, original = active('duplicate_frame_does_not_keepalive')
    b.advance(90_000); b.feed(encode(original))
    b.check_transport(deliveries=[[False, 'sequence']], buffered=0)
    b.check(state='ACTIVE', target_speed_mps=1., deadline_us=320_000)
    b.advance(10_000); b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('bad_crc_then_complete_stop')
    damaged = bytearray(encode(b.packet(.9))); damaged[-1] ^= 1
    b.feed(bytes(damaged) + encode(STOP))
    b.check_transport(deliveries=[[True, 'software_stop']], buffered=0, min_errors=1)
    b.check(state='ESTOP_LATCHED', reason='software_stop', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('invalid_length_then_complete_stop')
    b.feed(MAGIC + struct.pack('<H', 65535) + encode(STOP))
    b.check_transport(deliveries=[[True, 'software_stop']], buffered=0, min_errors=1)
    b.check(state='ESTOP_LATCHED', reason='software_stop', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('plausible_partial_length_hides_software_stop')
    pending = blocked_stop_prefix()
    b.feed(pending); b.check_transport(deliveries=[], buffered=len(pending))
    b.check(state='ACTIVE', reason='drive', target_speed_mps=1., deadline_us=320_000)
    b.advance(90_000); b.check(state='ACTIVE', target_speed_mps=1.)
    b.advance(10_000); b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    b.check_transport(buffered=len(pending))
    cases.append(b.finish())

    b, queued = active('physical_stop_independent_of_pending_parser')
    pending = blocked_stop_prefix()
    b.feed(pending); b.advance(10_000)
    b.event('physical_stop', pressed=True)
    b.check(state='ESTOP_LATCHED', reason='physical_stop', target_speed_mps=0., estop_held=True)
    b.feed(encode(queued)); b.check_transport(deliveries=[], buffered=len(pending)+len(encode(queued)))
    b.event('physical_stop', pressed=False)
    b.advance(100_000)
    b.check(state='ESTOP_LATCHED', reason='physical_stop', target_speed_mps=0., estop_held=False)
    cases.append(b.finish())

    b, _ = active('stale_source_completed_before_current_lease_expiry')
    frame = encode(b.packet(.7, age=90_000))
    b.feed(frame[:17]); b.advance(20_000)
    b.feed(frame[17:]); b.check_transport(deliveries=[[False, 'source_expired']], buffered=0)
    b.check(state='ACTIVE', target_speed_mps=1., deadline_us=320_000)
    b.advance(80_000); b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('tail_arrives_at_command_deadline')
    frame = encode(b.packet(.7))
    b.feed(frame[:17]); b.advance(100_000)
    b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    b.feed(frame[17:]); b.check_transport(deliveries=[[False, 'session']], buffered=0)
    b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('decoder_reset_drops_tail_but_does_not_stop_or_rearm')
    frame = encode(b.packet(.7))
    b.feed(frame[:17]); b.event('decoder_reset')
    b.check_transport(buffered=0)
    b.check(state='ACTIVE', target_speed_mps=1., deadline_us=320_000)
    b.feed(frame[17:]); b.check_transport(deliveries=[], buffered=0)
    b.advance(100_000); b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    b.feed(frame); b.check_transport(deliveries=[[False, 'session']], buffered=0)
    b.check(state='FAULT_LATCHED', target_speed_mps=0.)
    cases.append(b.finish())

    b, original = active('byte_traffic_does_not_feed_control_watchdog')
    for _ in range(6):
        b.now += 10_000
        b.feedback()  # 통신/피드백만 살아 있고 제어 tick은 멎은 조건.
        b.feed(encode(original))
    b.check_transport(deliveries=[[False, 'session']], buffered=0)
    b.check(state='FAULT_LATCHED', reason='control_deadline_missed', target_speed_mps=0.)
    cases.append(b.finish())

    b, original = active('stop_clear_and_new_arm_are_separate_over_wire')
    b.send(STOP); b.advance(220_000)
    b.control('CLEAR'); b.check(state='DISARMED', reason='explicit_clear', target_speed_mps=0.)
    b.feed(encode(original)); b.check_transport(deliveries=[[False, 'session']], buffered=0)
    b.control('ARM'); b.check(state='ARMED', reason='explicit_arm', target_speed_mps=0.)
    b.drive(.5); b.check(state='ACTIVE', reason='drive', target_speed_mps=.5)
    cases.append(b.finish())
    return cases


def replay(case):
    config = ContractConfig(**case['config'])
    receiver = ActuationReceiver(config, boot_id=case['initial_boot'])
    decoder = Decoder()
    responses = []
    for row in case['events']:
        receiver, decoder, response = apply_transport_event(receiver, decoder, config, row['input'])
        responses.append(response)
    return dict(name=case['name'], responses=responses)


def compare_case(case, observed):
    expected = dict(name=case['name'], responses=[row['expected'] for row in case['events']])
    return compare_values(expected, observed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: digest(ROOT/p) for p in FILES}
    cases = make_cases()
    observations = [replay(case) for case in cases]
    failures = [dict(name=case['name'], error=compare_case(case, observation))
                for case, observation in zip(cases, observations) if compare_case(case, observation)]
    bundle = dict(schema_version=1, scope='PC decoder plus contract; not serial hardware or firmware',
                  clock='per-boot MCU monotonic integer microseconds', cases=cases,
                  source_sha256=hashes, partial_frame_timeout_implemented=False,
                  uart_baud_or_queue_modelled=False, firmware_verified=False, ready_to_drive=False)
    for name, value in [('vectors.json', bundle), ('python_observations.json', observations)]:
        with (args.output/name).open('x', encoding='utf-8') as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.write(ROOT/path, path)
    report = dict(cases=len(cases), events=sum(len(c['events']) for c in cases),
                  checkpoints=sum(len(c['checkpoints']) for c in cases),
                  transport_checkpoints=sum(len(c['transport_checkpoints']) for c in cases),
                  replay_failures=failures, source_sha256=hashes,
                  source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()),
                  runtime_changed=False, firmware_verified=False, ready_to_drive=False,
                  software_stop_can_be_delayed_by_partial_frame=True,
                  deadline_is_target_zero_not_physical_stopping_time=True)
    with (args.output/'report.json').open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in report.items() if k != 'source_sha256'}))
    return 0 if not failures and report['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
