"""서보 없는 BNV1 단발 통합: 실제 포트/모터 접근 없음."""
from collections import deque
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_vehicle_interface'), str(ROOT/'tests')]
from test_uno_q_host import make_owner, ticks, FakeMcu
from uno_q_external_servo import encode_bench, decode_bench, UnoMailbox
from uno_q_bench import OneShotBench
from uno_q_servo import NoServo
from uno_q_host import parser, validate


class BenchMcu(FakeMcu):
    def __init__(self, clock):
        self.bench_state = 'disarmed';self.bench_seq = -1;self.bench_reason = 'not_started'
        self.stop_at = None;self.mode = 'bench';self.pulses = 0;self.keepalives = 0
        super().__init__(clock)
        self.observation()
    def observation(self):
        self.row(dict(type='uno_q', v=1, boot=self.boot, generation=self.generation, accepted_seq=self.seq,
            state=self.state, reason='fixture', bench_state=self.bench_state, bench_seq=self.bench_seq,
            bench_reason=self.bench_reason, mode=self.mode, motor_counts=0, raw_count=0,
            profile_sha256='0'*64,
            pwm_duty_ppm=50000 if self.bench_state == 'running' else 0, hal_fault=False))
    def send(self, wire):
        if wire[4:8] != b'BNV1':
            super().send(wire)
            self.bench_state = 'disarmed';self.pwm_permitted = False
            self.observation();return
        self.sent.append(wire);self.rows.append(self.receipt('tx', wire))
        c = decode_bench(wire)
        assert self.state == 'DISARMED'
        assert (c['boot'], c['generation'], c['lease']) == (self.boot, self.generation, self.lease)
        assert c['sequence'] > self.bench_seq
        self.bench_seq = c['sequence']
        if c['action'] == 1:
            assert self.bench_state == 'disarmed'
            self.bench_state = 'armed';self.bench_reason = 'explicit_arm'
        elif c['action'] == 2:
            assert self.bench_state == 'armed'
            self.bench_state = 'running';self.bench_reason = 'pulse';self.pulses += 1
            self.stop_at = self.clock.now+c['duration_us']*1000;self.pwm_permitted = True
        elif c['action'] == 4:
            self.keepalives += 1
        else:self.bench_state = 'disarmed';self.pwm_permitted = False
        self.observation()
    def cycle(self):
        if self.stop_at is not None and self.clock.now >= self.stop_at:
            self.bench_state = 'disarmed';self.bench_reason = 'duration_complete';self.pwm_permitted = False
        self.offer();self.observation()


def bench_owner():
    owner, t, _, _, events = make_owner(False)
    owner.servo = NoServo()
    owner.bench = OneShotBench(duty_permille=50, direction=0, duration_us=150_000)
    mcu = BenchMcu(t);owner.io = mcu
    owner.request_control('STOP')
    return owner, t, mcu, events


def test_bench_codec_bounded_and_no_runtime_command_relaxation():
    row = dict(boot='1234abcd', generation=2, lease=3, sequence=4, action=2,
               direction=1, duty_permille=150, duration_us=5_000_000)
    wire = encode_bench(**row)
    assert len(wire) == 52 and decode_bench(wire) == row
    for update in ({'duty_permille':151}, {'duration_us':5_000_001}, {'action':4}, {'direction':2}, {'duty_permille':0}):
        with pytest.raises(ValueError):encode_bench(**(row|update))


def test_motor_only_one_pulse_clear_arm_pulse_keepalive_complete_no_servo():
    owner, t, m, events = bench_owner()
    for _ in range(80):
        m.cycle();ticks(owner, t)
        if owner.bench.done:break
    assert not owner.terminating and owner.bench.done and owner.bench.result['success']
    assert m.pulses == 1 and m.keepalives > 0 and not m.pwm_permitted
    packets = [decode_bench(w) for w in m.sent if w[4:8] == b'BNV1']
    assert [p['action'] for p in packets[:2]] == [1, 2]
    assert all(p['action'] == 4 for p in packets[2:])
    assert not any(w[4:8] == b'ESV1' for w in m.sent)
    assert all(a['sequence'] < b['sequence'] for a, b in zip(packets, packets[1:]))
    original_stop = m.stop_at
    for _ in range(10):m.cycle();ticks(owner, t)
    assert m.pulses == 1 and m.stop_at == original_stop


def test_wrong_firmware_and_host_observation_gap_fail_without_pulse():
    owner, t, m, _ = bench_owner();m.mode = 'installed';m.observation();ticks(owner, t)
    assert owner.terminating and m.pulses == 0
    owner, t, m, _ = bench_owner()
    for _ in range(4):m.cycle();ticks(owner, t)
    m.rows.clear();t.step(101_000_000);owner.bench.step(owner, t.now)
    assert owner.terminating and not owner.bench.result['success']


def test_cli_bench_requires_all_explicit_parameters_and_allows_no_servo():
    args = parser().parse_args(['--dry-run', '--bench', '--bench-duty-permille', '50',
                              '--bench-direction', '0', '--bench-duration-ms', '1000'])
    assert validate(args)[2] is None and args.servo_port is None and not args.enable_drive_wire
    for added in (['--release-servo'], ['--ros'], ['--diagnostics']):
        with pytest.raises(ValueError):validate(parser().parse_args(['--dry-run', '--bench',
            '--bench-duty-permille', '50', '--bench-direction', '0', '--bench-duration-ms', '1000']+added))
    with pytest.raises(ValueError):validate(parser().parse_args(['--dry-run', '--bench']))
