#!/usr/bin/env python3
"""별도 시계 시간 후보와 C++ MCU의 제한 프레임 교환. 운영 호스트/연속 UART 시험 아님.

왕복 시각 fixture의 진짜 시계는 평가기만 쓴다. 후보에는 왕복 기록으로 구한 구간과
host 단조 입력만 준다. 한 번에 한 요청을 교환하며 추가 host TX 큐는 없는 가정이다.
"""
import argparse
from fractions import Fraction
import hashlib
import json
import math
from pathlib import Path
import zipfile

from actuation_host_observer import CompactHostObserver
from actuation_source_time_review import review_source_time
from audit_actuation_tx_policy import TxDriver, compile_driver, ns_per_byte
from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.clock_bounds import ClockAssumptions, ClockExchange, fit_clock
from arena_vehicle_interface.clock_freshness import HostTimeWindow

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ['scripts/audit_actuation_separate_clock.py', 'scripts/actuation_source_time_review.py',
           'scripts/actuation_host_observer.py', 'scripts/audit_actuation_offer_clock.py',
           'scripts/audit_actuation_tx_policy.py', 'scripts/actuation_tx_driver.cpp',
           'tests/test_actuation_separate_clock.py']
SOURCES += [str(p.relative_to(ROOT)).replace('\\', '/') for p in
            sorted((ROOT/'src/arena_vehicle_interface/portable').glob('*.hpp'))]
SOURCES += [f'src/arena_vehicle_interface/arena_vehicle_interface/{name}.py'
            for name in ('actuation_wire', 'clock_bounds', 'clock_freshness')]


class ExchangeFixture:
    def __init__(self, binary, *, ppm=0, up_ns=1_000_000, down_ns=9_000_000, pause_ns=0,
                 source_age_ns=2_000_000, drop_arm_ack=False):
        self.rate = Fraction(1_000_000+ppm, 1_000_000)
        self.offset = 8_000_000_000
        self.baud = 460800
        self.up_ns, self.down_ns = up_ns, down_ns
        self.pause_ns, self.source_age_ns, self.drop_arm_ack = pause_ns, source_age_ns, drop_arm_ack
        self.mcu = TxDriver(binary, boot='boot-A', line_ns_per_byte=ns_per_byte(self.baud))
        self.now = 0
        self.feedback_seq = 0
        self.events = []
        self.last = None
        self.observer = CompactHostObserver(boot='boot-A', host_epoch='host-A', connection_id='link-A', max_lease_us=100_000)
        self.decoder = Decoder()
        assumptions = ClockAssumptions(200, 1, 1, 5_000_000, 500_000)
        exchange = ClockExchange('boot-A', 1, math.floor(self.truth(900_000))-up_ns,
                                 900_000, 900_100, math.ceil(self.truth(900_100))+down_ns)
        self.fit = fit_clock([exchange], assumptions)
        self.exchange = exchange

    def truth(self, mcu_us):
        return self.offset+self.rate*mcu_us*1000

    def mcu_time(self, host_ns):
        return math.ceil(Fraction(host_ns-self.offset, 1000)/self.rate)

    def frame_time(self, byte_count):
        return math.ceil(Fraction(byte_count*10*1_000_000_000, self.baud))

    def window(self, ns):
        return HostTimeWindow('host-A', ns, ns)

    def command(self, line):
        self.last = self.mcu.cmd(line)
        return self.last

    def advance(self, target):
        if target < self.now:
            raise AssertionError('fixture clock regressed')
        while self.now < target:
            self.now = min(target, self.now+5000)
            self.feedback_seq += 1
            self.command(f'E {self.now} {self.feedback_seq} {self.now} 0')
            self.command(f'T {self.now}')

    def send(self, payload, host_send_ns):
        wire = encode(payload)
        arrival_ns = host_send_ns+self.up_ns+self.frame_time(len(wire))
        self.advance(self.mcu_time(arrival_ns))
        rows = self.mcu.feed(self.now, wire)
        self.last = rows[-1]
        deliveries = [item for row in rows for item in row['deliveries']]
        self.events.append(dict(direction='host_to_mcu', host_send_ns=host_send_ns,
                                complete_mcu_us=self.now, wire_hex=wire.hex(), deliveries=deliveries))
        return deliveries

    def collect(self):
        # pump 2회의 프레임을 같은 시각에 고정한다. 제한 교환 fixture의 가정이다.
        stream = bytearray()
        frames = []
        materialized = self.now
        for _ in range(2):
            row = self.command(f'W {self.now} 100000')
            stream.extend(bytes.fromhex(row['written']))
            frames.extend(row['frames'])
        payloads = self.decoder.feed(bytes(stream))
        assert not self.decoder.buffer and self.decoder.errors == 0 and len(payloads) == len(frames)
        offset = 0
        arm_ack_ns = None
        latest_ns = math.ceil(self.truth(materialized))
        for payload, frame in zip(payloads, frames):
            offset += frame['size']
            latest_ns = math.ceil(self.truth(materialized))+self.down_ns+self.frame_time(offset)
            result = None
            if not (self.drop_arm_ack and payload['type'] == 'ack' and payload['request'] == 'arm-fixture'):
                result = self.observer.observe(payload, received=self.window(latest_ns), connection_id='link-A')
                assert result['accepted'], (payload, result)
                if payload['type'] == 'ack' and payload['request'] == 'arm-fixture' and payload['accepted'] is True:
                    arm_ack_ns = latest_ns
            self.events.append(dict(direction='mcu_to_host', materialize_mcu_us=materialized,
                                    host_receive_ns=latest_ns, payload=payload, observer=result))
        return latest_ns, arm_ack_ns

    def execute(self):
        self.command('C 0')
        self.command('R 0 1')  # 시험 대체값. 실물 독립 출력 감시 아님.
        self.advance(1_000_000)
        receipt_ns, _ = self.collect()
        first = self.observer.latest('offer').payload
        arm = dict(v=1, kind='ARM', boot=first['boot'], generation=first['generation'],
                   lease=first['lease'], client='host-fixture', seq=1)
        arm_delivery = self.send(dict(type='command', request='arm-fixture', command=arm), receipt_ns)
        assert arm_delivery and arm_delivery[-1]['result'][0], arm_delivery
        receipt_ns, arm_ack_ns = self.collect()
        now_ns = receipt_ns+2_000_000+self.pause_ns
        current = self.observer.latest('offer')
        report = None
        drive_delivery = []
        command_sent = False
        # fixture의 알려진 요청만 대조한다. 운영 요청 관리의 대체가 아니다.
        if arm_ack_ns is not None and current.payload['state'] == 'ARMED' and current.payload['owner'] == 'host-fixture':
            options = dict(mapping_host_epoch='host-A', expected_boot='boot-A',
                           expected_generation=current.payload['generation'], max_lease_us=100_000,
                           max_issue_age_ns=100_000_000,
                           forward_delay_upper_ns=self.up_ns+self.frame_time(1544)+500_000,
                           source_ttl_ns=100_000_000,
                           policy_note='합성 8N1, host TX 추가 큐 없음, 전체 최대 프레임+0.5 ms 여유; 실측 아님')
            source = self.window(now_ns-self.source_age_ns)
            report = review_source_time(current.payload, self.fit, current.received, self.window(now_ns),
                                        source, self.window(arm_ack_ns), **options)
            if report['timestamp_policy_satisfied']:
                drive = dict(v=1, kind='DRIVE', boot=current.payload['boot'], generation=current.payload['generation'],
                             lease=current.payload['lease'], client='host-fixture', seq=2,
                             speed_mps=.5, steering_rad=.1, source_age_us=report['candidate_source_age_us'],
                             ttl_us=report['candidate_ttl_us'])
                drive_delivery = self.send(dict(type='command', request='', command=drive), now_ns)
                command_sent = True
                assert drive_delivery and drive_delivery[-1]['result'][0], drive_delivery
                assert self.last['deadline_us'] == report['receiver_deadline_us']
                assert self.truth(self.last['deadline_us']) <= source.lower_ns+100_000_000
        if not command_sent:
            self.advance(self.mcu_time(now_ns))
        before_stop = dict(state=self.last['state'], reason=self.last['reason'], target_speed_mps=self.last['target_speed_mps'])
        stop_send_ns = max(now_ns+1_000_000, math.ceil(self.truth(self.now))+1_000_000)
        stop_delivery = self.send(dict(type='command', request='stop-fixture', command={'v': 1, 'kind': 'STOP'}), stop_send_ns)
        assert stop_delivery[-1]['result'][0]
        assert self.last['state'] == 'ESTOP_LATCHED' and self.last['target_speed_mps'] == 0
        return dict(assumptions=dict(ppm_true=(self.rate-1)*1_000_000, up_ns=self.up_ns, down_ns=self.down_ns,
                                     pause_ns=self.pause_ns, source_age_ns=self.source_age_ns,
                                     drop_arm_ack=self.drop_arm_ack, output_ready_stub=True,
                                     feedback='synthetic motor counts zero at <=5ms ticks; no motor physics',
                                     host_tx_queue='none', baud=self.baud),
                    candidate=report, drive_sent_by_fixture=command_sent,
                    drive_delivery=drive_delivery, before_stop=before_stop,
                    final_state=self.last['state'], final_target_mps=self.last['target_speed_mps'],
                    events=self.events, ready_to_drive=False, runtime_connected=False, hardware_executed=False)


CASES = [
    ('nominal', {}, True), ('clock_slow', dict(ppm=-200), True), ('clock_fast', dict(ppm=200), True),
    ('reverse_asymmetry', dict(up_ns=9_000_000, down_ns=1_000_000), True),
    ('conservative_forward_bound', dict(up_ns=45_000_000), False),
    ('late_decision', dict(pause_ns=120_000_000), False),
    ('old_source', dict(source_age_ns=120_000_000), False),
    ('missing_arm_ack', dict(drop_arm_ack=True), False),
]


def run_case(binary, options):
    item = ExchangeFixture(binary, **options)
    try:
        return item.execute()
    finally:
        item.mcu.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES}
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for p in SOURCES:
            archive.write(ROOT/p, p)
    binary = compile_driver(args.output/'tx-driver')
    rows = []
    for name, options, expected in CASES:
        report = run_case(binary, options)
        report['assumptions']['ppm_true'] = int(report['assumptions']['ppm_true'])
        rows.append(dict(name=name, passed=report['drive_sent_by_fixture'] is expected,
                         expected_drive_by_fixture=expected, report=report))
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    result = dict(scope='bounded_offline_exchanges_not_runtime_or_continuous_throughput',
                  passed=all(r['passed'] for r in rows) and unchanged, cases=rows,
                  source_sha256=hashes, source_unchanged=unchanged,
                  ready_to_drive=False, firmware_verified=False, runtime_connected=False)
    (args.output/'report.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(passed=result['passed'], cases=len(rows), source_unchanged=unchanged)))
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
