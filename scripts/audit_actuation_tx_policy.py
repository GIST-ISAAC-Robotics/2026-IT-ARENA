#!/usr/bin/env python3
"""구동 출력 없는 송신 정책 후보의 결정론 선로·실제 호스트 콜백 폐루프 감사.

C++ `ActuationTxScheduler`(MCU 쪽 후보)를 lock-step driver로 호출하고, 8N1 직렬 선로의 바이트 시각을
Fraction(µs)으로 계산한다. 호스트는 실제 `ActuationHost.on_rx/on_drive/control/watch`를 노드 없이 호출한다.
두 쪽 시각을 같은 합성 시계로 둔 것은 이 fixture의 가정이며 실물 MCU/호스트 시계 대응이 아니다.
보드·UART·USB·DDS를 측정하지 않는다. firmware_verified=false, ready_to_drive=false.
"""
import argparse
from collections import Counter, deque
from contextlib import contextmanager
from fractions import Fraction
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
from types import MethodType, SimpleNamespace
import zipfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/arena_vehicle_interface'))
sys.path.insert(0, str(ROOT / 'scripts'))
from arena_vehicle_interface.actuation_wire import Decoder, MAX_PAYLOAD, encode  # noqa: E402

# 변이체 검사(check_actuation_tx_mutants.py)만 사본 헤더 폴더를 지정한다. 감사 증거 실행에서는 설정하지 않는다.
PORTABLE = Path(os.environ.get('ARENA_TX_PORTABLE_OVERRIDE', ROOT / 'src/arena_vehicle_interface/portable'))
DRIVER = ROOT / 'scripts/actuation_tx_driver.cpp'
BAUDS = (115200, 230400, 460800, 921600)
KINDS = ('ack', 'state', 'offer', 'diag')
SCENARIOS = ('idle', 'drive50', 'duplicate50', 'drive50_ack')
BOOT_TEXT = 'b' * 32       # 이전 감사(callback_budget_v1)와 같은 32자 식별자
CLIENT_TEXT = 'c' * 32
LEASE_US = 100_000
FILES = [
    'scripts/audit_actuation_tx_policy.py', 'scripts/actuation_tx_driver.cpp', 'tests/test_actuation_tx_policy.py',
    'src/arena_vehicle_interface/portable/actuation_tx.hpp',
    'src/arena_vehicle_interface/portable/actuation_ingress.hpp',
    'src/arena_vehicle_interface/portable/actuation_receiver.hpp',
    'src/arena_vehicle_interface/portable/actuation_command.hpp',
    'src/arena_vehicle_interface/portable/actuation_link.hpp',
    'src/arena_vehicle_interface/portable/actuation_wire.hpp',
    'scripts/audit_actuation_callback_budget.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_ros.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_wire.py',
]
DEFAULT_POLICY = dict(offer_period_us=20_000, diag_period_us=100_000, max_consecutive_events=2, link_timeout_us=30_000)


def ns_per_byte(baud):
    """8N1 바이트당 시간(ns, 올림). 정책의 진단 끼워 넣기 판단에만 쓰며 실측값이 아니다."""
    return -(-10_000_000_000 // baud)


def compile_driver(output, sanitizers=True):
    compiler = shutil.which('g++')
    if not compiler:
        raise RuntimeError('g++ required')
    flags = ['-O1', '-fsanitize=address,undefined', '-fno-omit-frame-pointer'] if sanitizers else ['-Os']
    run = subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror', '-pedantic', '-fno-exceptions',
                          '-fno-rtti', *flags, '-I' + str(PORTABLE), str(DRIVER), '-o', str(output)],
                         capture_output=True, text=True, timeout=180)
    if run.returncode != 0:
        raise RuntimeError(run.stderr)
    return Path(output)


def code_points(text):
    return ','.join(str(ord(c)) for c in text)


def text_of(points):
    return None if points is None else ''.join(map(chr, points))


def dumps(obj):
    """Python 운영 encode와 같은 직렬화. 기대 바이트 비교에 쓴다."""
    return json.dumps(obj, allow_nan=False, separators=(',', ':'), ensure_ascii=True).encode('ascii')


class TxDriver:
    """C++ 송신 정책 lock-step 프로세스. 한 줄 입력마다 한 줄 JSON."""
    def __init__(self, binary, *, boot=BOOT_TEXT, layout=1, line_ns_per_byte=0, **policy):
        self.policy = {**DEFAULT_POLICY, **policy, 'line_ns_per_byte': line_ns_per_byte}
        self.stderr = tempfile.TemporaryFile(mode='w+')
        self.proc = subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=self.stderr, text=True, bufsize=1)
        p = self.policy
        boot_field = code_points(boot) if isinstance(boot, str) else ','.join(map(str, boot))
        self.init = self.cmd(f"I {p['offer_period_us']} {p['diag_period_us']} {p['max_consecutive_events']} "
                             f"{p['line_ns_per_byte']} {p['link_timeout_us']} {layout} {boot_field}")

    def cmd(self, line):
        self.proc.stdin.write(line + '\n')
        self.proc.stdin.flush()
        out = self.proc.stdout.readline()
        if not out:
            self.close(check=False)
            raise RuntimeError(f'driver exited on {line[:80]!r}: {self.error_text()}')
        return json.loads(out)

    def feed(self, now, data):
        rows = []
        for i in range(0, len(data), 256):
            rows.append(self.cmd(f'F {now} {bytes(data[i:i+256]).hex()}'))
        return rows

    def error_text(self):
        self.stderr.seek(0)
        return self.stderr.read()

    def close(self, check=True):
        if self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=30)
        err = self.error_text()
        self.stderr.close()
        if check and (self.proc.returncode != 0 or err):
            raise RuntimeError(f'driver exit {self.proc.returncode}: {err}')
        return self.proc.returncode, err


class Line:
    """8N1 한 방향. 바이트당 10 bit. fifo가 있으면 송신 HW FIFO(바이트가 시프트를 시작하면 FIFO를 떠남)."""
    def __init__(self, baud, fifo=None):
        self.baud, self.fifo = baud, fifo
        self.byte_us = Fraction(10_000_000, baud)
        self.free_at = Fraction(0)
        self.starts = deque()
        self.in_flight = deque()   # (end_time, byte)
        self.total = 0

    def space(self, now):
        while self.starts and self.starts[0] <= now:
            self.starts.popleft()
        return None if self.fifo is None else self.fifo - len(self.starts)

    def push(self, now, data):
        times = []
        for byte in data:
            start = max(Fraction(now), self.free_at)
            end = start + self.byte_us
            self.free_at = end
            self.starts.append(start)
            self.in_flight.append((end, byte))
            times.append((start, end))
        self.total += len(data)
        return times

    def deliver(self, now):
        out = bytearray()
        while self.in_flight and self.in_flight[0][0] <= now:
            out.append(self.in_flight.popleft()[1])
        return bytes(out)


@contextmanager
def deterministic_host_runtime(clock):
    """ActuationHost가 쓰는 time.monotonic/uuid를 합성 시계·순번으로 바꾼다(종료 시 복원)."""
    import arena_vehicle_interface.actuation_ros as ros
    saved = ros.time, ros.uuid
    counter = iter(range(1, 1 << 62))
    ros.time = SimpleNamespace(monotonic=lambda: clock.now / 1e6)
    ros.uuid = SimpleNamespace(uuid4=lambda: SimpleNamespace(hex=f'{next(counter):032x}'))
    try:
        yield ros
    finally:
        ros.time, ros.uuid = saved


class Coroutine:
    """rclpy Future를 기다리는 ActuationHost.control 코루틴을 executor 없이 진행한다.

    rclpy Future는 완료 전 두 번 재개하면 오류이므로, 호스트가 등록한 미결 Future가 끝났을 때만 재개한다.
    """
    def __init__(self, host, coro):
        self.coro, self.done, self.response, self.future = coro, False, None, None
        before = set(host.pending)
        self._send()
        added = [key for key in host.pending if key not in before]
        if not self.done:
            self.future = host.pending[added[0]][0]

    def _send(self):
        try:
            self.coro.send(None)
        except StopIteration as stop:
            self.done, self.response = True, stop.value

    def step(self):
        if not self.done and self.future.done():
            self._send()


class ClosedLoop:
    """MCU(C++ driver) ↔ 선로 ↔ 실제 ActuationHost 콜백. 같은 합성 시계(µs)를 쓴다."""
    def __init__(self, binary, *, baud=460800, fifo=128, pump_interval_us=250, scenario='drive50',
                 output_ready=True, gate_diag=True, **policy):
        self.scenario = scenario
        self.baud, self.fifo, self.pump_interval_us = baud, fifo, pump_interval_us
        self.now = 0
        self.mcu = TxDriver(binary, line_ns_per_byte=ns_per_byte(baud) if gate_diag else 0, **policy)
        self.up = Line(baud)              # host -> MCU, 호스트 OS 버퍼는 무제한으로 둔다
        self.down = Line(baud, fifo)      # MCU -> host, MCU HW FIFO 가정
        self.frames = []                  # MCU -> host materialized frames (+ byte times)
        self.byte_owner = deque()         # 아직 시각을 배정하지 않은 frame 바이트의 소유 frame
        self.pending_bytes = 0
        self.monitor = Decoder()
        self.decoded = []                 # (host 수신 시각, payload)
        self.host_frames = []             # host -> MCU (시각, 종류, 크기)
        self.deliveries = []
        self.snapshots = []
        self.violations = []
        self.offers_seen_leases = Counter()
        self.previous_wire = None
        self.count = 0
        self.seq = 0
        self.output_ready = output_ready
        self.last_row = None
        self.coroutines = []

    # ---- host side ----
    def bind_host(self, ros):
        from arena_vehicle_interface.actuation_contract import CommandProducer
        host = SimpleNamespace(producer=CommandProducer(client_id=CLIENT_TEXT), decoder=Decoder(), offer=None,
                               offer_wall=0., pending={}, arm_barrier=None, last_now=None, tx=None,
                               now_us=lambda: self.now, wall_timeout=3.)
        for name in ('on_rx', 'fresh_offer', 'on_drive', 'control', 'watch'):
            setattr(host, name, MethodType(getattr(ros.ActuationHost, name), host))
        host.send = lambda _publisher, payload: self.host_send(payload)
        self.host = host
        return host

    def host_send(self, payload):
        command = payload.get('command', {})
        if self.scenario == 'drive50_ack' and command.get('kind') == 'DRIVE':
            payload = {**payload, 'request': 'r' * 32}
        wire = encode(payload)
        if command.get('kind') == 'DRIVE':
            self.previous_wire = wire
        self.up.push(self.now, wire)
        self.host_frames.append(dict(time_us=self.now, kind=command.get('kind'), bytes=len(wire),
                                     request=payload.get('request', '')))

    def control(self, kind):
        coroutine = Coroutine(self.host, self.host.control(kind, SimpleNamespace(success=None, message=None)))
        self.coroutines.append((kind, coroutine))
        return coroutine

    def drive_message(self, speed=.5, steering=.2):
        sec, micro = divmod(self.now, 1_000_000)
        return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=micro * 1000)),
                               drive=SimpleNamespace(speed=speed, steering_angle=steering))

    # ---- MCU side ----
    def mcu_cmd(self, line):
        row = self.mcu.cmd(line)
        self.deliveries.extend(dict(time_us=self.now, **d) for d in row['deliveries'])
        self.last_row = row
        return row

    def pump(self):
        space = self.down.space(self.now)
        row = self.mcu_cmd(f'W {self.now} {space}')
        for frame in row['frames']:
            frame = dict(frame, first_start=None, last_end=None, written=0, decoded=None)
            frame['snapshot'] = {k: row[k] for k in ('generation', 'state', 'reason', 'owner', 'leases', 'accepted_seq',
                                                     'deadline_us', 'feedback_seq', 'feedback_capture_us',
                                                     'motor_counts', 'target_speed_mps', 'steering_rad',
                                                     'mean_wheel_mps', 'estop_held', 'deadline_miss_count')}
            frame['output_ready'] = self.output_ready
            self.frames.append(frame)
            self.byte_owner.append([frame, frame['size']])
        data = bytes.fromhex(row['written'])
        times = self.down.push(self.now, data)
        for start, end in times:
            owner = self.byte_owner[0]
            frame = owner[0]
            if frame['first_start'] is None:
                frame['first_start'] = start
            frame['last_end'] = end
            frame['written'] += 1
            owner[1] -= 1
            if owner[1] == 0:
                self.byte_owner.popleft()
        return row

    def link(self, op):
        """C 연결 / D 단절. 단절 때 C++가 버린 in-flight 꼬리를 소유 목록에서도 제거한다(materialize는 1개씩)."""
        row = self.mcu_cmd(f'{op} {self.now}')
        if op == 'D' and self.byte_owner:
            if row['in_flight_remaining'] != 0 or len(self.byte_owner) != 1:
                self.violations.append('in-flight frame not discarded at session break')
            self.byte_owner.popleft()[0]['aborted'] = True
        return row

    def step(self, *, encoder=True, drive=False, drive_speed=.5):
        t = self.now
        arrived = self.up.deliver(t)
        if arrived:
            for row in self.mcu.feed(t, arrived):
                self.deliveries.extend(dict(time_us=t, **d) for d in row['deliveries'])
                self.last_row = row
        if t % 10_000 == 0 and encoder:
            self.seq += 1
            self.mcu_cmd(f'E {t} {self.seq} {t} {self.count}')
        if t % 5_000 == 0:
            self.mcu_cmd(f'T {t}')
        if t % self.pump_interval_us == 0:
            self.pump()
        received = self.down.deliver(t)
        self.last_step_t = t
        if received:
            for payload in self.monitor.feed(received):
                self.decoded.append((t, payload))
            self.host.on_rx(SimpleNamespace(data=received))
        if drive and t % 20_000 == 0:
            if self.scenario == 'duplicate50' and self.previous_wire is not None:
                # 이전 감사와 같은 고장 fixture: 호스트 판단과 무관하게 이전 DRIVE wire를 50 Hz로 반복 배송한다.
                self.up.push(t, self.previous_wire)
                self.host_frames.append(dict(time_us=t, kind='DRIVE', bytes=len(self.previous_wire), request=''))
            else:
                self.host.on_drive(self.drive_message(drive_speed))
        if t % 10_000 == 0:
            self.host.watch()
        for _, coroutine in self.coroutines:
            coroutine.step()

    def run_until(self, end, **kwargs):
        while self.now < end:
            self.step(**kwargs)
            self.now += 250

    def wait_control(self, kind, limit_us=500_000, **kwargs):
        coroutine = self.control(kind)
        deadline = self.now + limit_us
        while not coroutine.done and self.now < deadline:
            self.step(**kwargs)
            self.now += 250
        return coroutine

    def close(self):
        return self.mcu.close()


def frame_timing(frame):
    if frame.get('first_start') is None or frame.get('last_end') is None or frame['written'] != frame['size']:
        return None
    return dict(dwell_us=frame['materialize_us'] - frame['trigger_us'],
                sink_wait_us=float(frame['first_start'] - frame['materialize_us']),
                serialize_us=float(frame['last_end'] - frame['first_start']),
                latency_us=float(frame['last_end'] - frame['trigger_us']))


def verify_frames(loop):
    """디코드한 실제 바이트와 C++ 기록/수신기 스냅숏 대조. 위반은 목록으로 돌려준다."""
    violations = []
    decoded_payloads = [p for _, p in loop.decoded]
    complete = [f for f in loop.frames if f['written'] == f['size'] and not f.get('aborted')
                and f['last_end'] <= loop.last_step_t]
    if len(decoded_payloads) != len(complete):
        violations.append(f'decoded {len(decoded_payloads)} != complete frames {len(complete)}')
    expected_acks = Counter()
    for d in loop.deliveries:
        if d['called'] and d['request']:
            expected_acks[(text_of(d['request']), d['result'][0], d['result'][1])] += 1
    seen_acks = Counter()
    for frame, payload in zip(complete, decoded_payloads):
        frame['decoded'] = payload
        snap = frame['snapshot']
        kind = payload.get('type')
        if kind != frame['kind']:
            violations.append(f"frame {frame['id']} kind {kind} != {frame['kind']}")
            continue
        if len(encode(payload)) != frame['size']:
            violations.append(f"frame {frame['id']} re-encoded size differs")
        if kind == 'offer':
            lease = next((l for l in snap['leases'] if l[0] == payload['lease']), None)
            expected = dict(type='offer', v=1, boot=BOOT_TEXT if loop.mcu_boot is None else loop.mcu_boot,
                            generation=snap['generation'], state=snap['state'], owner=text_of(snap['owner']),
                            lease=frame['lease'], lease_issued_us=frame['materialize_us'],
                            lease_expires_us=frame['materialize_us'] + LEASE_US, output_ready=frame['output_ready'])
            if encode(expected) != encode(payload):
                violations.append(f"offer {frame['id']} {payload} != {expected}")
            if lease is None or lease[1] != frame['materialize_us'] or lease[2] - lease[1] != LEASE_US:
                violations.append(f"offer {frame['id']} lease not issued at materialization: {lease}")
            loop.offers_seen_leases[payload['lease']] += 1
        elif kind == 'ack':
            seen_acks[(payload['request'], payload['accepted'], payload['reason'])] += 1
        elif kind == 'state':
            if payload['generation'] > snap['generation'] or (
                    payload['generation'] == snap['generation'] and payload['state'] != snap['state']):
                violations.append(f"state {frame['id']} not current: {payload} vs {snap['generation']}/{snap['state']}")
        elif kind == 'diag':
            for key in ('generation', 'state', 'reason', 'accepted_seq', 'deadline_us', 'feedback_seq',
                        'feedback_capture_us', 'motor_counts', 'estop_held', 'deadline_miss_count'):
                if payload[key] != snap[key]:
                    violations.append(f"diag {frame['id']} {key} {payload[key]} != {snap[key]}")
            for key, source in (('target_speed_umps', 'target_speed_mps'), ('steering_urad', 'steering_rad'),
                                ('mean_wheel_umps', 'mean_wheel_mps')):
                if abs(payload[key] - snap[source] * 1e6) > 1:
                    violations.append(f"diag {frame['id']} {key} {payload[key]} vs {snap[source]}")
            if payload['now_us'] != frame['materialize_us']:
                violations.append(f"diag {frame['id']} snapshot time differs")
    duplicates = [lease for lease, n in loop.offers_seen_leases.items() if n > 1]
    if duplicates:
        violations.append(f'lease repackaged in multiple offers: {duplicates[:5]}')
    unexpected = seen_acks - expected_acks
    if unexpected:
        violations.append(f'ack without matching receiver decision: {list(unexpected.items())[:3]}')
    return violations, seen_acks, expected_acks


def summarize(loop, window):
    start, end = window
    rows = [f for f in loop.frames if start <= f['materialize_us'] < end]
    by_kind = {k: [f for f in rows if f['kind'] == k] for k in KINDS}
    mcu_bytes = sum(f['size'] for f in rows)
    host_bytes = sum(f['bytes'] for f in loop.host_frames if start <= f['time_us'] < end)
    duration_s = (end - start) / 1e6
    timing = {}
    for kind, frames in by_kind.items():
        values = [frame_timing(f) for f in frames]
        values = [v for v in values if v]
        timing[kind] = {key: (max(v[key] for v in values) if values else None)
                        for key in ('dwell_us', 'sink_wait_us', 'serialize_us', 'latency_us')}
    offer_gaps = [b['materialize_us'] - a['materialize_us'] for a, b in zip(by_kind['offer'], by_kind['offer'][1:])]
    rejected = [d for d in loop.deliveries if start <= d['time_us'] < end and d['called'] and not d['result'][0]]
    return dict(window_us=[start, end],
                frames={k: len(v) for k, v in by_kind.items()},
                bytes_by_kind={k: sum(f['size'] for f in v) for k, v in by_kind.items()},
                mcu_to_host_bytes_per_s=mcu_bytes / duration_s, host_to_mcu_bytes_per_s=host_bytes / duration_s,
                host_frames=Counter(f['kind'] for f in loop.host_frames if start <= f['time_us'] < end),
                load={str(b): dict(mcu_to_host=mcu_bytes / duration_s * 10 / b, host_to_mcu=host_bytes / duration_s * 10 / b)
                      for b in BAUDS},
                max_timing_us=timing, max_offer_gap_us=max(offer_gaps) if offer_gaps else None,
                rejected_deliveries=len(rejected),
                rejected_reasons=dict(Counter(d['result'][1] for d in rejected)))


def run_scenario(binary, scenario, *, baud=460800, fifo=128, pump_interval_us=250, duration_us=1_000_000, **policy):
    if scenario not in SCENARIOS:
        raise ValueError(scenario)
    clock = SimpleNamespace(now=0)
    loop = ClosedLoop(binary, baud=baud, fifo=fifo, pump_interval_us=pump_interval_us, scenario=scenario, **policy)
    loop.mcu_boot = None
    loop.now_ref = clock
    with deterministic_host_runtime(loop) as ros:
        loop.bind_host(ros)
        loop.mcu_cmd('C 0')
        loop.mcu_cmd(f'R 0 {int(loop.output_ready)}')
        loop.run_until(250_000)
        arm = None
        if scenario != 'idle':
            arm = loop.wait_control('ARM')
            if not (arm.done and arm.response.success):
                loop.violations.append(f'ARM failed: {arm.response and arm.response.message}')
        start = (loop.now // 20_000 + 2) * 20_000
        drive = scenario != 'idle'
        while loop.now < start + duration_us + 100_000:
            if drive and loop.now % 10_000 == 0:
                loop.count += 10  # 0.5 m/s 명령 아래의 합성 모터축 진행. 진행 감시 통과용 fixture
            loop.step(drive=drive)
            loop.now += 250
    violations, seen_acks, expected_acks = verify_frames(loop)
    final = loop.last_row
    code, err = loop.close()
    summary = summarize(loop, (start, start + duration_us))
    return dict(scenario=scenario, baud=baud, fifo_bytes=fifo, pump_interval_us=pump_interval_us,
                policy={**DEFAULT_POLICY, **policy}, arm_ok=None if arm is None else bool(arm.response and arm.response.success),
                summary=summary, final_state=final['state'], final_reason=final['reason'],
                final_generation=final['generation'], tx_stats=final['stats'], tx_fault=final['tx_fault'],
                host_offer_state=None if loop.host.offer is None else loop.host.offer['state'],
                first_drive_results=[d['result'] for d in loop.deliveries if d['class'] == 8][:5],
                host_offer_keys=None if loop.host.offer is None else sorted(loop.host.offer),
                acks_decoded=sum(seen_acks.values()), acks_expected=sum(expected_acks.values()),
                worst_payload=loop.mcu.init['worst_payload'], sizeof_tx=loop.mcu.init['sizeof_tx'],
                violations=loop.violations + violations, driver_exit=code, sanitizer_stderr=err)


def run_stop_reconnect(binary, *, baud=460800, fifo=128, **policy):
    """ACTIVE 중 단절(전송 중 프레임) → 재연결 → 명시 CLEAR → 별도 ARM → 호스트 STOP. 자동 재허가가 없는지 본다."""
    loop = ClosedLoop(binary, baud=baud, fifo=fifo, scenario='drive50', **policy)
    loop.mcu_boot = None
    phases = {}
    # 단절 동안 호스트가 보낸 바이트는 선로에서 사라진 것으로 본다(UART에는 버퍼 보존 보장이 없다).

    def drives_since(t):
        return sum(1 for f in loop.host_frames if f['kind'] == 'DRIVE' and f['time_us'] >= t)

    def advance(duration, drive, moving):
        end = loop.now + duration
        while loop.now < end:
            if moving and loop.now % 10_000 == 0:
                loop.count += 10
            loop.step(drive=drive)
            loop.now += 250

    with deterministic_host_runtime(loop) as ros:
        loop.bind_host(ros)
        loop.mcu_cmd('C 0')
        loop.mcu_cmd('R 0 1')
        loop.run_until(250_000)
        phases['arm'] = loop.wait_control('ARM').response.success
        advance(200_000, True, True)
        phases['active_before_break'] = loop.last_row['state']
        # 전송 중인 프레임이 있을 때 단절한다.
        while loop.last_row['in_flight_remaining'] == 0:
            loop.step(drive=True)
            loop.now += 250
        row = loop.link('D')
        phases['break'] = dict(time_us=loop.now, state=row['state'], reason=row['reason'],
                               aborted_frames=row['stats']['aborted_frames'], aborted_bytes=row['stats']['aborted_bytes'],
                               ack_purged=row['stats']['ack_purged_session'])
        down_start = len(loop.frames)
        advance(50_000, True, False)
        phases['frames_while_down'] = len(loop.frames) - down_start
        row = loop.link('C')
        reconnect_t = loop.now
        first_after = len(loop.decoded)
        advance(250_000, True, False)
        after = [p for _, p in loop.decoded[first_after:]]
        phases['after_reconnect'] = dict(
            first_types=[p['type'] for p in after[:3]],
            first_state=next((dict(state=p['state'], reason=p['reason'], generation=p['generation'])
                              for p in after if p['type'] == 'state'), None),
            host_offer_state=loop.host.offer['state'], drives_sent=drives_since(reconnect_t),
            receiver=dict(state=loop.last_row['state'], reason=loop.last_row['reason']))
        clear = loop.wait_control('CLEAR', drive=True)
        phases['clear'] = dict(success=clear.response.success, message=clear.response.message,
                               state=loop.last_row['state'])
        clear_t = loop.now
        advance(250_000, True, False)
        phases['after_clear'] = dict(state=loop.last_row['state'], drives_sent=drives_since(clear_t))
        arm = loop.wait_control('ARM', drive=True)
        phases['rearm'] = dict(success=arm.response.success, state=loop.last_row['state'])
        advance(200_000, True, True)
        phases['after_rearm'] = dict(state=loop.last_row['state'], reason=loop.last_row['reason'])
        stop = loop.wait_control('STOP', drive=True)
        stop_t = loop.now
        phases['stop'] = dict(success=stop.response.success, message=stop.response.message, state=loop.last_row['state'])
        advance(250_000, True, False)
        late = [d['result'] for d in loop.deliveries if d['time_us'] >= stop_t and d['called']]
        phases['after_stop'] = dict(state=loop.last_row['state'], reason=loop.last_row['reason'],
                                    drives_sent=drives_since(stop_t), delivered_results=late,
                                    host_offer_state=loop.host.offer['state'])
    violations, seen_acks, expected_acks = verify_frames(loop)
    final = loop.last_row
    code, err = loop.close()
    return dict(phases=phases, tx_stats=final['stats'], tx_fault=final['tx_fault'],
                acks_decoded=sum(seen_acks.values()), acks_expected=sum(expected_acks.values()),
                violations=loop.violations + violations, driver_exit=code, sanitizer_stderr=err)


def baseline_queue(trace, baud):
    """기존(no-policy) 콜백 trace의 MCU->host 바이트를 같은 선로에 무제한 큐로 넣은 산술 대조."""
    line = Line(baud)
    worst_delay, backlog_end = 0., 0
    for row in trace:
        if row['direction'] != 'mcu_to_host':
            continue
        times = line.push(row['now_us'], b'\0' * row['bytes'])
        worst_delay = max(worst_delay, float(times[-1][1] - row['now_us']))
    last = max(r['now_us'] for r in trace)
    backlog_end = max(0., float(line.free_at - last))
    return dict(baud=baud, max_frame_completion_delay_us=worst_delay, backlog_after_last_event_us=backlog_end)


def arithmetic(worst_payload, policy):
    rows = []
    for baud in BAUDS:
        byte_us = 10e6 / baud
        worst = {k: (v + 8) * byte_us for k, v in worst_payload.items()}
        event = max(worst['ack'], worst['state'])
        rows.append(dict(baud=baud, byte_us=byte_us, worst_frame_serialize_us=worst,
                         max_legal_frame_serialize_us=(MAX_PAYLOAD + 8) * byte_us,
                         offer_dwell_bound_us=max(worst.values()) + policy['max_consecutive_events'] * event,
                         fifo128_drain_us=128 * byte_us))
    return rows


def build_report(binary):
    from audit_actuation_callback_budget import build_report as baseline_report
    baseline = baseline_report()
    policy_runs = [run_scenario(binary, s) for s in SCENARIOS]
    sensitivity = [run_scenario(binary, 'drive50', baud=b) for b in (115200, 230400, 921600)]
    # 진단 끼워 넣기 판단을 끈 비교: 긴 진단 프레임이 offer를 가리는 포화 사례를 보존한다.
    sensitivity += [run_scenario(binary, 'drive50', baud=b, gate_diag=False) for b in (115200, 230400)]
    sensitivity.append(run_scenario(binary, 'drive50_ack', baud=115200))
    sensitivity.append(run_scenario(binary, 'drive50', pump_interval_us=1000))
    sensitivity.append(run_scenario(binary, 'drive50', diag_period_us=0))
    stop_reconnect = run_stop_reconnect(binary)
    comparison = []
    for run in policy_runs:
        base = next(r for r in baseline['scenarios'] if r['scenario'] == run['scenario'])
        offers = sum(s['frames'] for s in base['summary'] if s['kind'] == 'offer')
        acks = sum(s['frames'] for s in base['summary'] if s['kind'] == 'ack')
        comparison.append(dict(scenario=run['scenario'],
                               baseline=dict(offers=offers, acks=acks, mcu_to_host_bytes_per_s=base['byte_counts']['mcu_to_host'],
                                             host_to_mcu_bytes_per_s=base['byte_counts']['host_to_mcu'],
                                             final_state=base['final_state'], final_reason=base['final_reason'],
                                             queue_460800=baseline_queue(base['trace'], 460800)),
                               policy=dict(frames=run['summary']['frames'],
                                           mcu_to_host_bytes_per_s=run['summary']['mcu_to_host_bytes_per_s'],
                                           host_to_mcu_bytes_per_s=run['summary']['host_to_mcu_bytes_per_s'],
                                           final_state=run['final_state'], final_reason=run['final_reason'])))
    worst = policy_runs[0]['worst_payload']
    return dict(scope='C++ transmit policy candidate under a deterministic 8N1 line model with real ActuationHost '
                      'callbacks; no ROS nodes, UART, USB, board or motor output',
                assumptions=dict(shared_synthetic_clock_host_and_mcu=True, real_clock_mapping_connected=False,
                                 mcu_tx_fifo_bytes=128, mcu_tx_fifo_is_assumption=True,
                                 pump_interval_us=250, host_tx_buffer_unbounded=True, uart_8n1_bits_per_byte=10,
                                 output_ready_stubbed=True, boot_and_client_32_ascii=True,
                                 duplicate_scenario_replays_old_wire_not_nominal_host=True),
                policy=DEFAULT_POLICY, worst_payload=worst, arithmetic=arithmetic(worst, DEFAULT_POLICY),
                comparison=comparison, policy_runs=policy_runs, sensitivity=sensitivity,
                stop_reconnect=stop_reconnect,
                runtime_changed=False, real_transport_measured=False, deadline_proven=False,
                firmware_verified=False, ready_to_drive=False)


def digest(path):
    import hashlib
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def strip(report):
    """report.json에는 프레임 원시 목록 대신 요약만 남긴다(Fraction 직렬화 제외)."""
    return json.loads(json.dumps(report, default=float))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    before = {path: digest(ROOT / path) for path in FILES}
    with tempfile.TemporaryDirectory() as tmp:
        binary = compile_driver(Path(tmp) / 'tx_driver', sanitizers=True)
        report = build_report(binary)
    report.update(source_sha256=before, source_unchanged=all(digest(ROOT / p) == v for p, v in before.items()),
                  portable_override=os.environ.get('ARENA_TX_PORTABLE_OVERRIDE'))
    with (args.output / 'report.json').open('x', encoding='utf-8') as stream:
        json.dump(strip(report), stream, ensure_ascii=False, indent=2, allow_nan=False)
    with zipfile.ZipFile(args.output / 'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            archive.write(ROOT / path, path)
    print(json.dumps([dict(scenario=c['scenario'], baseline=c['baseline']['mcu_to_host_bytes_per_s'],
                           policy=round(c['policy']['mcu_to_host_bytes_per_s']), frames=c['policy']['frames'],
                           final=c['policy']['final_state']) for c in report['comparison']]))
    runs = report['policy_runs'] + report['sensitivity'] + [dict(report['stop_reconnect'], scenario='stop_reconnect')]
    bad = [r['scenario'] for r in runs if r['violations'] or r['sanitizer_stderr'] or r['driver_exit'] != 0]
    print(json.dumps(dict(violations=bad)))
    return 0 if report['source_unchanged'] and not bad else 2


if __name__ == '__main__':
    raise SystemExit(main())
