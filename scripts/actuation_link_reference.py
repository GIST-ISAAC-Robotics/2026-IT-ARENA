#!/usr/bin/env python3
"""부분 프레임 절대 조립 기한·연결/수신 오류 정책의 독립 PC 참조. 런타임/펌웨어 아님.

- LinkOracle: C++ FrameLink 상태를 읽지 않고, 바이트마다 도착 호출/시각을 보존해 같은 바이트 경계를 계산한다.
- LinkActuationPolicy: 링크 출력과 기존 ActuationReceiver의 경계를 정의하는 참조 정책 후보다.
  프레임 유효성(CRC)·명령 유효성(엄격 JSON/스키마/세션)·구동 허가(CLEAR→ARM)를 분리한다.
  ROS 기본 경로·운영 Python 런타임에는 연결하지 않는다.
"""
from copy import deepcopy
import struct
import zlib

from actuation_conformance import Builder, apply_event, compare_values
from arena_vehicle_interface.actuation_contract import ActuationReceiver, ContractConfig, State
from arena_vehicle_interface.actuation_wire import Decoder, MAGIC, MAX_PAYLOAD, encode

U64_MAX = 2**64 - 1
CHUNK = 256
RESULTS = ('ok', 'not_connected', 'fault_latched', 'fault_raised', 'not_faulted')
STATES = ('disconnected', 'connected', 'faulted')
FAULTS = ('none', 'invalid_config', 'clock_regressed', 'input_budget')
STOP = {'v': 1, 'kind': 'STOP'}
# 시험 전용 조립 기한. 실제 baud·최대 프레임·USB/스케줄 지연 실측 전 운영값으로 쓰지 않는다.
TEST_ASSEMBLY_TIMEOUT_US = 30_000


def frame(payload):
    body = struct.pack('<H', len(payload)) + payload
    return MAGIC + body + struct.pack('<I', zlib.crc32(body))


class LinkOracle:
    """바이트별 (값, 도착 호출 번호, 도착 시각)을 보존하는 느린 독립 계산."""

    def __init__(self, timeout):
        if type(timeout) is not int or not 0 <= timeout <= U64_MAX:
            raise ValueError('timeout outside uint64')
        self.timeout = timeout
        self.pending = []
        self.anchor = None
        self.last_now = None
        self.call = 0
        self.counters = dict(errors=0, timeouts=0, rejected=0, dropped=0, breaks=0, sessions=0)
        self.state, self.fault, self.last_fault = 'disconnected', 'none', 'none'
        if timeout == 0:
            self.state, self.fault, self.last_fault = 'faulted', 'invalid_config', 'invalid_config'

    def _data(self):
        return bytes(item[0] for item in self.pending)

    def _scan(self, frames, origin):
        while self.pending:
            data = self._data()
            start = data.find(MAGIC)
            if start < 0:
                self.pending = self.pending[-1:] if data.endswith(b'I') else []
                break
            if start:
                self.pending = self.pending[start:]
                data = data[start:]
                self.counters['errors'] += 1
            if len(data) < 4:
                break
            size = int.from_bytes(data[2:4], 'little')
            if not 1 <= size <= MAX_PAYLOAD:
                self.pending = self.pending[2:]
                self.counters['errors'] += 1
                continue
            if len(data) < size + 8:
                break
            if zlib.crc32(data[2:4+size]) != int.from_bytes(data[4+size:8+size], 'little'):
                self.pending = self.pending[2:]
                self.counters['errors'] += 1
                continue
            frames.append((origin, data[4:4+size]))
            self.pending = self.pending[size+8:]

    def _clock(self, now):
        if self.last_now is not None and now < self.last_now:
            self._latch('clock_regressed')
            return False
        self.last_now = now
        return True

    def _drop(self):
        self.counters['dropped'] += len(self.pending)
        self.pending, self.anchor = [], None

    def _latch(self, fault):
        self._drop()
        self.counters['breaks'] += 1
        self.state, self.fault, self.last_fault = 'faulted', fault, fault

    def _expire(self, now, frames):
        if self.anchor is None or now - self.anchor < self.timeout:
            return
        self.counters['timeouts'] += 1
        while self.pending:
            self.pending = self.pending[min(2, len(self.pending)):]
            self.counters['errors'] += 1
            self._scan(frames, 'X')
        self.anchor = None

    def _feed(self, now, data, null, frames):
        if self.state == 'faulted':
            self.counters['rejected'] += 1
            return 'fault_latched'
        if not self._clock(now):
            return 'fault_raised'
        if self.state != 'connected':
            self.counters['rejected'] += 1
            return 'not_connected'
        if len(data) > CHUNK or (null and len(data)):
            self._latch('input_budget')
            return 'fault_raised'
        self._expire(now, frames)
        self.pending += [(byte, self.call, now) for byte in data]
        self._scan(frames, 'L')
        if not self.pending:
            self.anchor = None
        elif all(item[1] == self.call for item in self.pending):
            self.anchor = now
        return 'ok'

    def apply(self, op):
        kind, now = op[0], op[1]
        if type(now) is not int or not 0 <= now <= U64_MAX:
            raise ValueError('fixture time outside uint64')
        self.call += 1
        frames = []
        if kind == 'F':
            result = self._feed(now, op[2], False, frames)
        elif kind == 'Z':
            # 길이만 있는 null 포인터 입력. 내용은 의미가 없다.
            result = self._feed(now, b'\0' * op[2] if op[2] <= CHUNK else b'\0' * (CHUNK+1), True, frames)
        elif kind == 'X':
            if self.state != 'faulted':
                result = 'not_faulted'
            elif self.fault == 'invalid_config':
                self.counters['rejected'] += 1
                result = 'fault_latched'
            else:
                self.state, self.fault, self.last_now = 'disconnected', 'none', now
                result = 'ok'
        elif self.state == 'faulted':
            self.counters['rejected'] += 1
            result = 'fault_latched'
        elif not self._clock(now):
            result = 'fault_raised'
        elif kind == 'P':
            result = 'not_connected' if self.state != 'connected' else 'ok'
            if result == 'ok':
                self._expire(now, frames)
        elif kind == 'C':
            if self.state == 'connected':
                self.counters['breaks'] += 1
            self._drop()
            self.state = 'connected'
            self.counters['sessions'] += 1
            result = 'ok'
        elif kind == 'D':
            if self.state != 'connected':
                result = 'not_connected'
            else:
                self.counters['breaks'] += 1
                self._drop()
                self.state = 'disconnected'
                result = 'ok'
        else:
            raise ValueError(f'unknown link op {kind}')
        self._invariants(kind, now, result)
        return self.snapshot(result, frames)

    def _invariants(self, kind, now, result):
        if (self.anchor is None) != (not self.pending):
            raise AssertionError('anchor/pending mismatch')
        if self.state != 'connected' and self.pending:
            raise AssertionError('bytes kept outside a connected session')
        if self.pending:
            oldest = self.pending[0][2]
            if self.anchor > oldest:
                raise AssertionError('anchor later than oldest buffered byte')
            if kind in ('F', 'P') and result == 'ok' and now - oldest >= self.timeout:
                raise AssertionError('buffered byte survived its assembly deadline')

    def snapshot(self, result, frames):
        return dict(result=result, state=self.state, fault=self.fault, last_fault=self.last_fault,
                    **self.counters, anchor=self.anchor, buffer=self._data().hex(),
                    frames=[[origin, payload.hex()] for origin, payload in frames])


def snapshot_line(snapshot):
    s = snapshot
    fields = [RESULTS.index(s['result']), STATES.index(s['state']), FAULTS.index(s['fault']),
              FAULTS.index(s['last_fault']), s['errors'], s['timeouts'], s['rejected'], s['dropped'],
              s['breaks'], s['sessions'], '-' if s['anchor'] is None else s['anchor'], s['buffer'],
              ','.join(f'{origin}:{payload}' for origin, payload in s['frames'])]
    return 'L;' + ';'.join(str(field) for field in fields)


def parse_line(line):
    parts = line.split(';')
    if len(parts) != 14 or parts[0] != 'L':
        raise ValueError(f'unexpected link line: {line[:80]!r}')
    numbers = [int(value) for value in parts[1:11]]
    frames = []
    for item in filter(None, parts[13].split(',')):
        origin, payload = item.split(':')
        frames.append([origin, payload])
    return dict(result=RESULTS[numbers[0]], state=STATES[numbers[1]], fault=FAULTS[numbers[2]],
                last_fault=FAULTS[numbers[3]], errors=numbers[4], timeouts=numbers[5],
                rejected=numbers[6], dropped=numbers[7], breaks=numbers[8], sessions=numbers[9],
                anchor=None if parts[11] == '-' else int(parts[11]), buffer=parts[12], frames=frames)


def op_command(op):
    kind, now = op[0], op[1]
    if kind == 'F':
        return f'F {now}' + (f' {op[2].hex()}' if op[2] else '')
    if kind == 'Z':
        return f'Z {now} {op[2]}'
    return f'{kind} {now}'


# ---------------------------------------------------------------- 구동 정책 참조 후보

def apply_policy_link(receiver, policy, now, snapshot):
    """링크 출력 하나를 수신 상태기계에 반영한다. 이 함수는 tick/ARM/CLEAR를 호출하지 않는다.

    - 완성 프레임: 기존 Python 엄격 JSON 검증 뒤 같은 시각으로 receive. 늦게 drain된 프레임도
      원래 임대/원천 시각으로 만료를 판정하므로 수명이 늘지 않는다.
    - 세션 경계(단절, 놓친 단절, 링크 고장): 수신기가 이미 고정 상태가 아니면 로컬 STOP을 넣어
      세대를 바꾼다. 이후 재구동은 새 offer 기반 CLEAR→별도 ARM만 허용한다.
    - 조립 기한 초과·CRC/길이 오류: 구동 상태를 바꾸지 않는다(명령 만료가 별도로 판정).
    """
    deliveries = []
    for origin, payload in snapshot['frames']:
        messages = Decoder().feed(frame(bytes.fromhex(payload)))
        if not messages:
            deliveries.append(dict(origin=origin, result=[False, 'json_rejected']))
            continue
        deliveries.append(dict(origin=origin, kind=messages[0].get('kind'),
                               result=list(receiver.receive(messages[0], now))))
    injected = None
    if snapshot['breaks'] > policy['breaks_seen']:
        policy['breaks_seen'] = snapshot['breaks']
        reason = 'link_fault_' + snapshot['fault'] if snapshot['state'] == 'faulted' else 'link_session_break'
        policy['latch_reason'] = reason
        if receiver.state not in (State.ESTOP, State.FAULT):
            injected = list(receiver.receive(dict(STOP), now))
    if snapshot['breaks'] < policy['breaks_seen']:
        raise AssertionError('link break counter regressed')
    return dict(deliveries=deliveries, injected_stop=injected)


def apply_link_event(receiver, policy, config, event, snapshot):
    op, now = event['op'], event['now_us']
    if op == 'link':
        result = apply_policy_link(receiver, policy, now, snapshot)
        link = snapshot
    else:
        receiver, response = apply_event(receiver, config, event)
        result, link = response['result'], None
    return receiver, dict(result=result, link=link, status=receiver.status(),
                          policy=deepcopy(policy))


def link_op_from_event(event):
    args = event['args']
    if args['kind'] in ('F',):
        return ('F', event['now_us'], bytes.fromhex(args['hex']))
    if args['kind'] == 'Z':
        return ('Z', event['now_us'], args['size'])
    return (args['kind'], event['now_us'])


class LinkBuilder(Builder):
    """명령은 링크를 통해서만 전달한다. 링크 연산은 제어 tick을 대신하지 않는다."""

    def __init__(self, name, timeout=TEST_ASSEMBLY_TIMEOUT_US):
        super().__init__(name)
        self.timeout = timeout
        self.link = LinkOracle(timeout)
        self.policy = dict(breaks_seen=0, latch_reason=None)
        self.link_checkpoints = []

    def event(self, op, now=None, **args):
        if now is not None:
            self.now = now
        event = dict(op=op, now_us=self.now, args=args)
        snapshot = self.link.apply(link_op_from_event(event)) if op == 'link' else None
        self.receiver, response = apply_link_event(self.receiver, self.policy, self.config, event, snapshot)
        self.events.append(dict(input=deepcopy(event), expected=deepcopy(response)))
        return response['result']

    def link_op(self, kind, now=None, **args):
        return self.event('link', now, kind=kind, **args)

    def feed(self, data):
        if len(data) > CHUNK:
            raise ValueError('fixture must chunk explicitly')
        return self.link_op('F', hex=data.hex())

    def send(self, message):
        data, deliveries = encode(message), []
        for start in range(0, len(data), CHUNK):
            deliveries += self.feed(data[start:start+CHUNK])['deliveries']
        live = [d for d in deliveries if d['origin'] == 'L']
        if len(live) != 1:
            raise AssertionError((self.name, 'fixture expected one live frame', deliveries))
        return live[0]['result']

    def advance(self, duration, *, feedback=True, increment=0, refresh=False, tick=True, poll=True):
        if duration <= 0 or duration % 10_000:
            raise ValueError('fixture duration must be positive 10 ms steps')
        for _ in range(duration // 10_000):
            self.now += 10_000
            if feedback:
                self.feedback(self.count + increment)
            if poll:
                self.link_op('P')
            if tick:
                self.event('tick')
            if refresh and self.receiver.armed:
                self.drive(1.)

    def check_link(self, **expected):
        # 가장 최근 링크 연산의 결과를 수동 기대값과 비교한다(advance 뒤면 마지막 poll).
        index = max(i for i, row in enumerate(self.events) if row['input']['op'] == 'link')
        response = self.events[index]['expected']
        observed = dict(response['link'], injected_stop=response['result']['injected_stop'],
                        deliveries=[[d['origin'], d.get('kind'), d['result']]
                                    for d in response['result']['deliveries']],
                        latch_reason=response['policy']['latch_reason'])
        for key, value in expected.items():
            if observed[key] != value:
                raise AssertionError(f'{self.name}: link {key} expected {value!r}, got {observed[key]!r}')
        self.link_checkpoints.append(dict(event_index=index, expected=expected))

    def finish(self):
        case = super().finish()
        case.update(link_timeout=self.timeout, link_checkpoints=self.link_checkpoints)
        return case


def active(name, timeout=TEST_ASSEMBLY_TIMEOUT_US):
    b = LinkBuilder(name, timeout)
    b.link_op('C')
    b.prepare()
    packet = b.drive(1.)
    b.check(state='ACTIVE', target_speed_mps=1., deadline_us=320_000)
    return b, packet


def hidden_prefix(inner):
    """허용 범위의 길이 1536을 가진 손상 머리 뒤에 완성 프레임이 숨는다."""
    return MAGIC + struct.pack('<H', MAX_PAYLOAD) + inner


def make_policy_cases():
    cases = []

    b, _ = active('hidden_stop_surfaces_at_assembly_deadline')
    b.feed(hidden_prefix(encode(STOP)))
    b.check_link(deliveries=[], anchor=220_000)
    b.advance(20_000)
    b.check(state='ACTIVE', target_speed_mps=1.)
    b.now += 9_999; b.link_op('P')
    b.check_link(deliveries=[], timeouts=0)
    b.now += 1; b.link_op('P')
    b.check_link(deliveries=[['X', 'STOP', [True, 'software_stop']]], timeouts=1, buffer='', anchor=None)
    b.check(state='ESTOP_LATCHED', reason='software_stop', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('trickle_cannot_extend_and_timeout_alone_keeps_actuation')
    b.feed(hidden_prefix(b''))
    for _ in range(2):
        b.advance(10_000, poll=False); b.feed(b'x')
        b.check_link(anchor=220_000)
    b.advance(10_000, poll=False)
    b.feed(b'IA')
    b.check_link(timeouts=1, buffer='4941', anchor=250_000, deliveries=[])
    b.check(state='ACTIVE', reason='drive', target_speed_mps=1., deadline_us=320_000)
    b.advance(60_000)
    b.check(state='ACTIVE', target_speed_mps=1.)
    b.advance(10_000)
    b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('drained_drive_keeps_original_lease_limit')
    late = b.packet(.6)
    b.feed(hidden_prefix(encode(late)))
    b.advance(30_000)
    b.check_link(deliveries=[['X', 'DRIVE', [True, 'drive']]], timeouts=1)
    b.check(state='ACTIVE', target_speed_mps=.6, deadline_us=320_000)
    b.advance(60_000); b.check(state='ACTIVE')
    b.advance(10_000); b.check(state='FAULT_LATCHED', reason='command_expired', target_speed_mps=0.)
    cases.append(b.finish())

    b, original = active('disconnect_latches_and_reconnect_needs_clear_then_arm')
    partial = encode(b.packet(.8))
    b.feed(partial[:17])
    b.link_op('D')
    b.check_link(state='disconnected', dropped=17, breaks=1, latch_reason='link_session_break',
                 injected_stop=[True, 'software_stop'])
    b.check(state='ESTOP_LATCHED', target_speed_mps=0.)
    b.feed(partial[17:]); b.check_link(result='not_connected', buffer='')
    b.link_op('C'); b.check_link(state='connected', sessions=2, injected_stop=None)
    b.check(state='ESTOP_LATCHED', target_speed_mps=0.)
    b.feed(partial[17:]); b.check_link(deliveries=[], buffer='')
    if b.send(dict(original, seq=b.producer.sequence + 50)) != [False, 'session']:
        raise AssertionError('old session command accepted after reconnect')
    b.advance(220_000)
    b.check(state='ESTOP_LATCHED', target_speed_mps=0.)
    b.control('CLEAR'); b.check(state='DISARMED', reason='explicit_clear', target_speed_mps=0.)
    if b.send(b.packet(.5)) != [False, 'not_owner_or_armed']:
        raise AssertionError('drive accepted before explicit ARM')
    b.control('ARM'); b.check(state='ARMED', reason='explicit_arm')
    b.drive(.5); b.check(state='ACTIVE', target_speed_mps=.5)
    cases.append(b.finish())

    b, _ = active('implicit_reconnect_is_a_session_break')
    b.feed(encode(STOP)[:5])
    b.link_op('C')
    b.check_link(state='connected', breaks=1, sessions=2, dropped=5, injected_stop=[True, 'software_stop'])
    b.feed(encode(STOP)[5:]); b.check_link(deliveries=[], buffer='')
    b.check(state='ESTOP_LATCHED', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('clock_regression_latches_link_and_rejects_same_feed')
    b.feed(encode(STOP)[:3])
    b.link_op('F', now=b.now - 1, hex=encode(STOP).hex())
    b.check_link(result='fault_raised', state='faulted', fault='clock_regressed', deliveries=[],
                 dropped=3, latch_reason='link_fault_clock_regressed')
    b.check(state='FAULT_LATCHED', reason='clock_regressed', target_speed_mps=0.)
    b.now += 1
    b.link_op('P'); b.check_link(result='fault_latched')
    b.link_op('C'); b.check_link(result='fault_latched', state='faulted')
    b.link_op('X'); b.check_link(result='ok', state='disconnected', fault='none', last_fault='clock_regressed')
    b.link_op('C'); b.check_link(state='connected', injected_stop=None)
    b.check(state='FAULT_LATCHED', target_speed_mps=0.)
    cases.append(b.finish())

    b, _ = active('input_budget_fault_discards_whole_call')
    b.feed(encode(STOP)[:4])
    oversized = encode(STOP) + b'\0' * (CHUNK + 1 - len(encode(STOP)))
    b.link_op('F', hex=oversized.hex())
    b.check_link(result='fault_raised', fault='input_budget', deliveries=[], dropped=4,
                 injected_stop=[True, 'software_stop'], latch_reason='link_fault_input_budget')
    b.check(state='ESTOP_LATCHED', target_speed_mps=0.)
    b.link_op('Z', size=1); b.check_link(result='fault_latched', rejected=1)
    cases.append(b.finish())

    b, _ = active('physical_stop_independent_of_link_state')
    b.feed(hidden_prefix(encode(STOP)))
    b.event('physical_stop', pressed=True)
    b.check(state='ESTOP_LATCHED', reason='physical_stop', target_speed_mps=0., estop_held=True)
    b.link_op('D'); b.check_link(injected_stop=None, latch_reason='link_session_break')
    b.event('physical_stop', pressed=False)
    b.link_op('C')
    b.advance(220_000)
    b.check(state='ESTOP_LATCHED', reason='physical_stop', target_speed_mps=0., estop_held=False)
    cases.append(b.finish())

    b, original = active('link_traffic_and_polls_do_not_feed_control_watchdog')
    for _ in range(6):
        b.now += 10_000
        b.feedback()  # 피드백·링크 poll·중복 완성 프레임만 계속되고 제어 tick은 멎은 조건.
        b.link_op('P')
        b.feed(encode(original))
    b.check_link(deliveries=[['L', 'DRIVE', [False, 'session']]])
    b.check(state='FAULT_LATCHED', reason='control_deadline_missed', target_speed_mps=0.)
    cases.append(b.finish())

    b = LinkBuilder('boot_link_cycle_never_clears_or_arms')
    b.link_op('C'); b.prepare(False)
    b.check(state='DISARMED', reason='boot')
    b.link_op('D'); b.check(state='ESTOP_LATCHED', target_speed_mps=0.)
    b.link_op('C'); b.link_op('D'); b.link_op('C')
    b.advance(220_000)
    b.check(state='ESTOP_LATCHED', target_speed_mps=0., owner=None)
    if b.send(b.producer.control(b.event('offer'), 'ARM')) != [False, 'not_disarmed']:
        raise AssertionError('ARM accepted without CLEAR after link break')
    b.control('CLEAR'); b.check(state='DISARMED', reason='explicit_clear')
    cases.append(b.finish())
    return cases


def replay_policy(case, snapshots):
    """링크 출력(예: C++)을 순서대로 받아 같은 수신기/정책 결과를 다시 계산한다."""
    config = ContractConfig(**case['config'])
    receiver = ActuationReceiver(config, boot_id=case['initial_boot'])
    policy = dict(breaks_seen=0, latch_reason=None)
    responses, index = [], 0
    for row in case['events']:
        event = row['input']
        snapshot = None
        if event['op'] == 'link':
            snapshot = snapshots[index]
            index += 1
        receiver, response = apply_link_event(receiver, policy, config, event, snapshot)
        responses.append(response)
    if index != len(snapshots):
        raise AssertionError('unused link snapshots')
    return dict(name=case['name'], responses=responses)


def link_ops(case):
    return [link_op_from_event(row['input']) for row in case['events'] if row['input']['op'] == 'link']


def compare_policy_case(case, observed):
    expected = dict(name=case['name'], responses=[row['expected'] for row in case['events']])
    return compare_values(expected, observed)
