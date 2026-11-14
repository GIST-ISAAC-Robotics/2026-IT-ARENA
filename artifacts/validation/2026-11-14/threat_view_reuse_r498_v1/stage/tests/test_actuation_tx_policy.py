"""송신 정책 후보: 바이트·큐·지연의 결정론 검사. 보드/UART/모터 시험이 아니다."""
from pathlib import Path
import shutil
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from audit_actuation_tx_policy import (  # noqa: E402
    BOOT_TEXT, CLIENT_TEXT, LEASE_US, MAX_PAYLOAD, TxDriver, compile_driver, dumps, ns_per_byte,
)
from arena_vehicle_interface.actuation_wire import Decoder, encode  # noqa: E402

COMPACT_KEYS = ['boot', 'generation', 'lease', 'lease_expires_us', 'lease_issued_us', 'output_ready', 'owner',
                'state', 'type', 'v']


@pytest.fixture(scope='module', params=['sanitizers', 'size_optimized'])
def binary(tmp_path_factory, request):
    if not shutil.which('g++'):
        pytest.skip('g++ required for portable transmit policy checks')
    return compile_driver(tmp_path_factory.mktemp('tx') / 'driver', sanitizers=request.param == 'sanitizers')


class Mcu:
    """C++ 정책을 직접 호출하는 시험 도구. 바이트를 frame 크기대로 잘라 원문 payload도 보존한다."""
    def __init__(self, binary, **policy):
        self.driver = TxDriver(binary, **policy)
        self.decoder = Decoder()
        self.pending = bytearray()
        self.sizes = []
        self.records, self.raw, self.payloads = [], [], []
        self.row = None

    def cmd(self, line):
        self.row = self.driver.cmd(line)
        return self.row

    def pump(self, now, capacity=100_000):
        row = self.cmd(f'W {now} {capacity}')
        self.records += row['frames']
        self.sizes += [f['size'] for f in row['frames']]
        self.pending += bytes.fromhex(row['written'])
        while self.sizes and len(self.pending) >= self.sizes[0]:
            size = self.sizes.pop(0)
            frame, self.pending = bytes(self.pending[:size]), self.pending[size:]
            self.raw.append(frame[4:-4])
            self.payloads += self.decoder.feed(frame)
        return row

    def setup(self, until=220_000):
        self.cmd('C 0')
        self.cmd('R 0 1')
        for t in range(0, until + 1, 10_000):
            self.cmd(f'E {t} {t // 10_000 + 1} {t} 0')
            self.cmd(f'T {t}')
            self.pump(t)
        return until

    def last(self, kind):
        return next(p for p in reversed(self.payloads) if p['type'] == kind)

    def send(self, now, command, request=''):
        rows = self.driver.feed(now, encode(dict(type='command', request=request, command=command)))
        self.row = rows[-1]
        return [d for r in rows for d in r['deliveries']]

    def control(self, now, kind, client=CLIENT_TEXT, request='', seq=1):
        offer = self.last('offer')
        return self.send(now, dict(v=1, kind=kind, boot=offer['boot'], generation=offer['generation'],
                                   lease=offer['lease'], client=client, seq=seq), request)

    def drive(self, now, seq, client=CLIENT_TEXT, offer=None, age=0):
        offer = offer or self.last('offer')
        return self.send(now, dict(v=1, kind='DRIVE', boot=offer['boot'], generation=offer['generation'],
                                   lease=offer['lease'], client=client, seq=seq, speed_mps=.5, steering_rad=.2,
                                   source_age_us=age, ttl_us=100_000))

    def close(self):
        code, err = self.driver.close(check=False)
        assert code == 0 and not err, err


def test_worst_payload_fits_frame_limit(binary):
    mcu = Mcu(binary)
    worst = mcu.driver.init['worst_payload']
    assert mcu.driver.init['tx_fault'] == 0
    assert all(0 < worst[k] <= MAX_PAYLOAD for k in ('ack', 'state', 'offer', 'diag')), worst
    # owner/request 64 코드포인트가 모두 astral이면 12 B씩. boot는 escape 없는 ASCII로 제한한다.
    assert worst['offer'] >= 2 + 12 * 64 and worst['ack'] >= 2 + 12 * 64
    mcu.close()


@pytest.mark.parametrize('boot', ['quote"boot', 'back' + chr(92), 'tab' + chr(9), 'h' + chr(0xE9), chr(0x1F600), ''])
def test_boot_that_needs_escaping_is_a_permanent_tx_fault(binary, boot):
    mcu = Mcu(binary, boot=boot if boot else [0])
    assert mcu.driver.init['tx_fault'] == 1
    mcu.cmd('C 0')
    for now in (0, 20_000, 200_000):
        assert mcu.pump(now)['written'] == ''
    mcu.close()


@pytest.mark.parametrize('policy', [dict(offer_period_us=0), dict(max_consecutive_events=0),
                                    dict(max_consecutive_events=9), dict(diag_period_us=-1),
                                    dict(offer_period_us=10_000_001), dict(line_ns_per_byte=-1)])
def test_invalid_policy_never_transmits(binary, policy):
    mcu = Mcu(binary, **policy)
    assert mcu.driver.init['tx_fault'] == 1
    mcu.cmd('C 0')
    assert mcu.pump(0)['written'] == ''
    mcu.close()


def exotic(length=64):
    """astral·짝 없는 surrogate·제어문자·따옴표/역슬래시·DEL·비ASCII를 섞은 정확히 length 코드포인트."""
    parts = [chr(0x1F600), chr(0xD800), chr(0xDFFF), '"', chr(92), chr(10), chr(8), chr(12), chr(13), chr(9),
             chr(0), chr(0x1F), chr(0x7F), chr(0xE9), chr(0xD55C), chr(0x10FFFF), '/', ' ']
    text = ''.join(parts)
    return (text * 4)[:length]


# 수신 명령 하나에 두 최대 문자열(각 770 B)을 함께 넣으면 host->MCU payload 1536 B를 넘으므로 나눠 검사한다.
@pytest.mark.parametrize('client,request_id', [(exotic(), exotic()), (chr(0x1F600) * 64, 'r'),
                                               ('c', chr(0x10FFFF) * 64), ('c', 'r')],
                         ids=['mixed64', 'astral_owner64', 'astral_request64', 'short'])
def test_ack_and_compact_offer_bytes_equal_python_encoder(binary, client, request_id):
    mcu = Mcu(binary)
    now = mcu.setup()
    deliveries = mcu.control(now, 'ARM', client=client, request=request_id)
    assert [d['result'] for d in deliveries] == [[True, 'explicit_arm']]
    mcu.pump(now + 1000)
    mcu.pump(now + 20_000)
    worst = mcu.driver.init['worst_payload']
    kinds = [p['type'] for p in mcu.payloads]
    ack_raw = mcu.raw[kinds.index('ack')]
    assert ack_raw == dumps(dict(type='ack', request=request_id, accepted=True, reason='explicit_arm'))
    assert len(ack_raw) <= worst['ack']
    offer_index = max(i for i, k in enumerate(kinds) if k == 'offer')
    offer, record = mcu.payloads[offer_index], [r for r in mcu.records if r['kind'] == 'offer'][-1]
    expected = dict(type='offer', v=1, boot=BOOT_TEXT, generation=1, state='ARMED', owner=client,
                    lease=record['lease'], lease_issued_us=record['materialize_us'],
                    lease_expires_us=record['materialize_us'] + LEASE_US, output_ready=True)
    assert mcu.raw[offer_index] == dumps(expected)
    assert sorted(offer) == COMPACT_KEYS and len(mcu.raw[offer_index]) <= worst['offer']
    mcu.close()


def test_zero_and_short_writes_never_replace_a_started_frame(binary):
    mcu = Mcu(binary, diag_period_us=0)
    now = mcu.setup()
    mcu.control(now, 'ARM')
    mcu.pump(now)                       # ARM 결과/상태/urgent offer를 모두 비운다
    now += 20_000
    mcu.cmd(f'T {now}')
    row = mcu.pump(now, 0)              # 주기 offer를 고정했지만 sink가 0 B만 받는다
    started = row['frames'][0]
    assert started['kind'] == 'offer' and row['written'] == '' and row['backpressure']
    assert mcu.pump(now, 7)['written'] != ''
    # 전송 중에 STOP이 와도 이미 고정한 offer 바이트는 바뀌지 않는다.
    assert [d['result'] for d in mcu.send(now, dict(v=1, kind='STOP'))] == [[True, 'software_stop']]
    assert mcu.row['generation'] == 2 and mcu.row['leases'] == []
    before = len(mcu.payloads)
    steps = 0
    while len(mcu.payloads) == before:     # 1 B씩만 받는 sink로 나머지 바이트를 보낸다
        mcu.pump(now, 1)
        steps += 1
    assert steps == started['size'] - 7
    mcu.pump(now + 250)
    mcu.pump(now + 500)
    old = mcu.payloads[before]
    assert old.get('lease') == started['lease']
    assert old['generation'] == 1 and old['state'] == 'ARMED'
    after = mcu.payloads[mcu.payloads.index(old) + 1:]
    # 세대가 바뀌면 새 임대 offer가 state 알림보다 먼저 나간다.
    assert [p['type'] for p in after[:2]] == ['offer', 'state']
    assert after[0]['generation'] == 2 and after[0]['state'] == 'ESTOP_LATCHED'
    assert after[1]['state'] == 'ESTOP_LATCHED' and after[1]['reason'] == 'software_stop'
    stats = mcu.row['stats']
    assert stats['sink_zero_writes'] >= 1 and stats['sink_partial_writes'] >= 1 and stats['offers_urgent'] >= 1
    # 옛 offer 임대는 STOP에서 무효가 되었고 재전송/병합으로 다시 살아나지 않는다.
    result = mcu.drive(now + 1000, 2, offer=old)
    assert result[0]['result'] == [False, 'session']
    mcu.close()


def test_ack_queue_is_bounded_with_stop_priority_and_explicit_drops(binary):
    mcu = Mcu(binary, diag_period_us=0)
    now = mcu.setup()
    mcu.pump(now, 0)  # sink 정지: 큐만 쌓인다
    for i in range(10):
        mcu.control(now, 'DISARM', request=f'd{i}', seq=i + 1)
    stats = mcu.row['stats']
    assert mcu.row['ack_count'] == 8 and stats['ack_dropped_overflow'] == 2
    for i in range(3):
        mcu.send(now, dict(v=1, kind='STOP'), request=f's{i}')
    stats = mcu.row['stats']
    assert mcu.row['ack_count'] == 8 and stats['ack_evicted_for_stop'] == 3
    for i in range(3, 12):
        mcu.send(now, dict(v=1, kind='STOP'), request=f's{i}')
    stats = mcu.row['stats']
    # 비STOP 결과 8개를 모두 축출한 뒤 큐가 STOP 결과로만 차면 새 STOP 결과는 명시 폐기한다(STOP 자체는 적용됨).
    assert stats['ack_evicted_for_stop'] == 8 and stats['ack_dropped_overflow'] == 2 + 4
    assert mcu.row['state'] == 'ESTOP_LATCHED'
    for t in range(now + 1000, now + 200_000, 1000):
        mcu.pump(t)
    acks = [p['request'] for p in mcu.payloads if p['type'] == 'ack']
    assert acks == [f's{i}' for i in range(8)]
    assert stats['ack_queue_max'] == 8
    mcu.close()


def test_stop_ack_overtakes_queued_non_stop_acks(binary):
    # v1 변이체 검사에서 큐가 STOP 결과로만 찬 사례만으로는 STOP 우선 순서를 구별하지 못했다(보강).
    mcu = Mcu(binary, diag_period_us=0)
    now = mcu.setup()
    mcu.pump(now, 0)
    for i in range(3):
        mcu.control(now, 'DISARM', request=f'd{i}', seq=i + 1)
    mcu.send(now, dict(v=1, kind='STOP'), request='s0')
    assert mcu.row['ack_count'] == 4 and mcu.row['stats']['ack_evicted_for_stop'] == 0
    for t in range(now + 1000, now + 100_000, 1000):
        mcu.pump(t)
    acks = [p['request'] for p in mcu.payloads if p['type'] == 'ack']
    assert acks == ['s0', 'd0', 'd1', 'd2']
    records = [r for r in mcu.records if r['kind'] == 'ack']
    assert [r['stop_ack'] for r in records] == [True, False, False, False]
    mcu.close()


def test_starvation_guard_bounds_offer_wait_under_request_flood(binary):
    baud = 230400
    mcu = Mcu(binary, diag_period_us=0, line_ns_per_byte=ns_per_byte(baud))
    now = mcu.setup()
    per_ms = baud // 10 // 1000   # 1 ms마다 sink가 받는 바이트(8N1 산술)
    seq = 0
    for t in range(now + 1000, now + 400_000, 1000):
        if t % 5000 == 0:
            mcu.cmd(f'E {t} {t // 5000 + 100} {t} 0')
            mcu.cmd(f'T {t}')
        for _ in range(2):
            seq += 1
            mcu.control(t, 'DISARM', request=f'flood-{seq:06d}', seq=seq)
        mcu.pump(t, per_ms)
    stats = mcu.row['stats']
    worst = mcu.driver.init['worst_payload']
    offers = [r for r in mcu.records if r['kind'] == 'offer' and r['materialize_us'] > now + 10_000]
    gaps = [b['materialize_us'] - a['materialize_us'] for a, b in zip(offers, offers[1:])]
    # offer 대기 상한(산술): 전송 중 프레임 하나 + 연속 사건 2개, 각 실제 최대 크기 + pump 1 ms 해상도.
    event_us = max(r['size'] for r in mcu.records if r['kind'] in ('ack', 'state')) * 10e6 / baud
    inflight_us = max(r['size'] for r in mcu.records) * 10e6 / baud
    bound = inflight_us + 2 * event_us + 2000
    assert stats['starvation_guard_used'] > 0 and stats['ack_dropped_overflow'] > 0
    assert stats['max_dwell_us'][3] <= bound, (stats['max_dwell_us'], bound)
    assert max(gaps) <= 20_000 + bound
    assert max(r['size'] for r in mcu.records if r['kind'] == 'ack') <= worst['ack'] + 8
    mcu.close()


def test_session_break_discards_tail_purges_acks_and_never_rearms(binary):
    mcu = Mcu(binary, diag_period_us=0)
    now = mcu.setup()
    mcu.control(now, 'ARM')
    mcu.pump(now)
    mcu.drive(now + 1000, 2)
    assert mcu.row['state'] == 'ACTIVE'
    mcu.pump(now + 1000, 0)
    old_offer = mcu.last('offer')
    for name in ('a-before-break', 'b-before-break'):   # 둘 다 sequence 거절. 하나는 전송 중, 하나는 큐 대기
        mcu.control(now + 1000, 'DISARM', request=name, seq=1)
    now += 20_000
    mcu.cmd(f'T {now}')
    mcu.pump(now, 5)
    assert mcu.row['in_flight_remaining'] > 0 and mcu.row['ack_count'] == 1
    row = mcu.cmd(f'D {now}')
    assert row['state'] == 'ESTOP_LATCHED' and row['reason'] == 'software_stop'
    assert row['in_flight_remaining'] == 0 and row['ack_count'] == 0
    assert row['stats']['aborted_frames'] == 1 and row['stats']['ack_purged_session'] == 1
    for t in range(now, now + 50_000, 5000):
        mcu.cmd(f'E {t} {t // 10_000 + 50} {t} 0')
        mcu.cmd(f'T {t}')
        assert mcu.pump(t)['written'] == ''          # 단절 중에는 materialize/쓰기 없음
    now += 50_000
    mcu.cmd(f'C {now}')
    mcu.pending.clear()
    mcu.sizes.clear()
    mcu.decoder = Decoder()
    first = len(mcu.payloads)
    mcu.pump(now)
    fresh = mcu.payloads[first:]
    assert [p['type'] for p in fresh[:2]] == ['offer', 'state']
    assert fresh[1]['state'] == 'ESTOP_LATCHED' and fresh[0]['generation'] == mcu.row['generation']
    assert not {'a-before-break', 'b-before-break'} & {p.get('request') for p in mcu.payloads}
    # 재연결·offer 수신만으로 ARM되지 않는다. 옛 세대 DRIVE는 session 거절.
    assert mcu.drive(now, 3, offer=old_offer)[0]['result'] == [False, 'session']
    for t in range(now, now + 300_000, 10_000):
        mcu.cmd(f'E {t} {t // 10_000 + 200} {t} 0')
        mcu.cmd(f'T {t}')
        mcu.pump(t)
        assert mcu.row['state'] == 'ESTOP_LATCHED'
    now += 300_000
    assert mcu.control(now, 'CLEAR', seq=10)[0]['result'] == [True, 'explicit_clear']
    for t in range(now, now + 200_000, 10_000):
        mcu.cmd(f'E {t} {t // 10_000 + 400} {t} 0')
        mcu.cmd(f'T {t}')
        mcu.pump(t)
        assert mcu.row['state'] == 'DISARMED'
    assert mcu.control(now + 200_000, 'ARM', seq=11)[0]['result'] == [True, 'explicit_arm']
    mcu.close()


def test_clock_regression_and_sink_contract_violation_latch_silence(binary):
    mcu = Mcu(binary)
    mcu.cmd('C 0')
    mcu.pump(1000)
    assert mcu.pump(999)['tx_fault'] == 2
    assert mcu.pump(50_000)['written'] == ''
    mcu.close()
    mcu = Mcu(binary)
    mcu.cmd('C 0')
    row = mcu.cmd('Z 0 10')
    assert row['tx_fault'] == 3 and row['stats']['aborted_frames'] == 1 and row['in_flight_remaining'] == 0
    assert mcu.pump(20_000)['written'] == ''
    mcu.close()


def test_backpressure_never_extends_lease_or_command_life(binary):
    mcu = Mcu(binary, diag_period_us=0)
    now = mcu.setup()
    mcu.control(now, 'ARM')
    for t in range(now, now + 5000, 250):
        mcu.pump(t)
    offer = mcu.last('offer')
    issued = offer['lease_issued_us']
    mcu.drive(now + 5000, 2, offer=offer)
    assert mcu.row['state'] == 'ACTIVE' and mcu.row['deadline_us'] == issued + LEASE_US
    blocked_from = now + 5000
    issued_during_block = set()
    for t in range(blocked_from, blocked_from + 200_000, 5000):
        mcu.cmd(f'E {t} {t // 5000 + 1000} {t} {(t - blocked_from) // 1000}')  # 약 0.49 m/s 진행
        mcu.cmd(f'T {t}')
        row = mcu.pump(t, 0)
        issued_during_block |= {lease[1] for lease in row['leases'] if lease[1] >= blocked_from}
        if t >= issued + LEASE_US:
            assert row['state'] == 'FAULT_LATCHED' and row['reason'] == 'command_expired'
        else:
            assert row['deadline_us'] == issued + LEASE_US, row['reason']   # 수신·대기로 수명이 늘지 않는다
    # 막힌 동안에는 전송 중 프레임 하나만 고정되므로 대기열에서 임대가 추가 발행되지 않는다.
    assert len(issued_during_block) <= 1
    unblock = blocked_from + 200_000
    for t in range(unblock, unblock + 40_000, 250):
        mcu.pump(t)
    latest = [r for r in mcu.records if r['kind'] == 'offer'][-1]
    payload = mcu.last('offer')
    assert payload['lease_issued_us'] == latest['materialize_us'] >= unblock
    assert payload['state'] == 'FAULT_LATCHED'
    mcu.close()


def test_diagnostic_snapshot_and_disable(binary):
    mcu = Mcu(binary, diag_period_us=50_000)
    now = mcu.setup()
    mcu.control(now, 'ARM')
    mcu.pump(now)
    assert mcu.drive(now + 1000, 2)[0]['result'] == [True, 'drive']
    for t in range(now + 1000, now + 60_000, 1000):
        mcu.pump(t)
    diag = mcu.last('diag')
    assert diag['state'] == 'ACTIVE' and diag['target_speed_umps'] == 500_000 and diag['steering_urad'] == 200_000
    assert diag['feedback_mode'] == 'drive_motor_shaft' and diag['left_ticks'] is None and diag['output_ready'] is True
    assert set(diag['tx']) == {'ack_dropped', 'ack_evicted', 'ack_purged', 'state_merged', 'diag_skipped',
                               'aborted_frames', 'encode_failures'}
    mcu.close()
    mcu = Mcu(binary, diag_period_us=0)
    now = mcu.setup()
    assert not [p for p in mcu.payloads if p['type'] == 'diag']
    mcu.close()


def test_diag_waits_for_room_before_next_offer(binary):
    baud = 230400
    mcu = Mcu(binary, diag_period_us=100_000, line_ns_per_byte=ns_per_byte(baud))
    now = mcu.setup(0)
    per_250us = baud // 10 // 4000
    for t in range(250, 500_000, 250):
        if t % 5000 == 0:
            mcu.cmd(f'E {t} {t // 5000 + 1} {t} 0')
            mcu.cmd(f'T {t}')
        mcu.pump(t, per_250us)
    offers = [r['materialize_us'] for r in mcu.records if r['kind'] == 'offer']
    assert max(b - a for a, b in zip(offers, offers[1:])) <= 20_000 + 250
    assert mcu.row['stats']['diag_deferred_for_offer'] > 0
    mcu.close()


def test_output_ready_is_reported_not_granted(binary):
    mcu = Mcu(binary, diag_period_us=0)
    mcu.cmd('C 0')
    mcu.cmd('R 0 0')
    mcu.pump(0)
    assert mcu.last('offer')['output_ready'] is False
    mcu.cmd('R 1 1')
    mcu.pump(20_000)
    assert mcu.last('offer')['output_ready'] is True and mcu.row['state'] == 'DISARMED'
    mcu.close()


def test_output_gate_rejection_is_acknowledged_without_receiver_or_lease_mutation(binary):
    mcu = Mcu(binary, diag_period_us=0)
    now = mcu.setup()
    mcu.cmd(f'R {now} 0')
    before = (mcu.row['generation'], mcu.row['leases'], mcu.row['accepted_seq'])
    rejected = mcu.control(now, 'ARM', request='ARM-ready?')
    assert rejected[0]['result'] == [False, 'output_not_ready']
    assert rejected[0]['output_gate_rejected'] and not rejected[0]['called']
    assert (mcu.row['generation'], mcu.row['leases'], mcu.row['accepted_seq']) == before
    mcu.pump(now)
    assert mcu.last('ack') == dict(type='ack', request='ARM-ready?', accepted=False, reason='output_not_ready')
    mcu.cmd(f'R {now} 1')
    assert mcu.row['state'] == 'DISARMED'
    assert mcu.control(now, 'ARM', request='ARM-now')[0]['result'] == [True, 'explicit_arm']
    mcu.pump(now)
    assert mcu.last('ack')['accepted'] is True
    mcu.close()


# ---- 실제 ActuationHost 콜백 폐루프(ROS 메시지 패키지 필요) ----

@pytest.fixture(scope='module')
def sanitized(tmp_path_factory):
    if not shutil.which('g++'):
        pytest.skip('g++ required')
    return compile_driver(tmp_path_factory.mktemp('tx_loop') / 'driver', sanitizers=True)


@pytest.fixture(scope='module')
def baseline():
    pytest.importorskip('rclpy')
    from audit_actuation_callback_budget import build_report
    return {r['scenario']: r for r in build_report()['scenarios']}


@pytest.mark.parametrize('scenario,state,reason,acks,rejected', [
    ('idle', 'DISARMED', 'boot', 0, 0), ('drive50', 'ACTIVE', 'drive', 0, 0),
    ('duplicate50', 'FAULT_LATCHED', 'command_expired', 0, 49), ('drive50_ack', 'ACTIVE', 'drive', 50, 0)])
def test_closed_loop_matches_baseline_semantics_with_less_traffic(sanitized, baseline, scenario, state, reason,
                                                                  acks, rejected):
    from audit_actuation_tx_policy import run_scenario
    run = run_scenario(sanitized, scenario)
    base = baseline[scenario]
    summary = run['summary']
    assert not run['violations'] and not run['sanitizer_stderr'] and run['driver_exit'] == 0
    assert (run['final_state'], run['final_reason']) == (state, reason)
    assert (base['final_state'], base['final_reason']) == (state, reason)
    assert summary['frames']['offer'] == 50 and summary['frames']['ack'] == acks
    assert summary['rejected_deliveries'] == rejected == base['rejected_command_callbacks']
    base_offers = sum(s['frames'] for s in base['summary'] if s['kind'] == 'offer')
    assert base_offers == (50 if scenario == 'idle' else 100)
    assert summary['mcu_to_host_bytes_per_s'] < base['byte_counts']['mcu_to_host']
    assert summary['load']['460800']['mcu_to_host'] < .5
    if scenario != 'idle':
        assert run['host_offer_keys'] == COMPACT_KEYS and run['arm_ok'] is True
        assert abs(summary['host_to_mcu_bytes_per_s'] - base['byte_counts']['host_to_mcu']) <= 500 + 1000 * (acks > 0)


def test_closed_loop_stop_reconnect_requires_explicit_clear_then_arm(sanitized):
    pytest.importorskip('rclpy')
    from audit_actuation_tx_policy import run_stop_reconnect
    run = run_stop_reconnect(sanitized)
    phases = run['phases']
    assert not run['violations'] and not run['sanitizer_stderr']
    assert phases['break']['state'] == 'ESTOP_LATCHED' and phases['break']['aborted_frames'] == 1
    assert phases['frames_while_down'] == 0
    after = phases['after_reconnect']
    assert after['first_types'][:2] == ['offer', 'state'] and after['first_state']['state'] == 'ESTOP_LATCHED'
    assert after['drives_sent'] == 0 and after['receiver']['state'] == 'ESTOP_LATCHED'
    assert phases['clear']['success'] and phases['after_clear'] == dict(state='DISARMED', drives_sent=0)
    assert phases['rearm']['success'] and phases['after_rearm']['state'] == 'ACTIVE'
    assert phases['stop']['success'] and phases['after_stop']['state'] == 'ESTOP_LATCHED'
    assert all(result[0] is False for result in phases['after_stop']['delivered_results'])


def test_saturation_stays_bounded_and_fails_closed(sanitized):
    pytest.importorskip('rclpy')
    from audit_actuation_tx_policy import run_scenario
    slow = run_scenario(sanitized, 'drive50', baud=115200)
    assert not slow['violations'] and slow['tx_stats']['ack_queue_max'] <= 8
    # 115200: ARM 임대(+100 ms) 안에 ack->offer->DRIVE 직렬화가 끝나지 않아 첫 DRIVE가 도착 전에 만료된다.
    # 늦은 DRIVE는 수명을 얻지 못하고(session 거절) 고장 고정으로 끝난다(fail-closed).
    assert (slow['final_state'], slow['final_reason']) == ('FAULT_LATCHED', 'command_expired')
    assert slow['first_drive_results'] and all(not r[0] for r in slow['first_drive_results'])
    gated = run_scenario(sanitized, 'drive50', baud=230400)
    ungated = run_scenario(sanitized, 'drive50', baud=230400, gate_diag=False)
    assert not gated['violations'] and not ungated['violations']
    assert gated['final_state'] == 'ACTIVE' and gated['summary']['max_offer_gap_us'] == 20_000
    assert gated['summary']['frames']['diag'] == 0 and gated['tx_stats']['diag_deferred_for_offer'] > 0
    assert ungated['summary']['max_offer_gap_us'] > 20_000
