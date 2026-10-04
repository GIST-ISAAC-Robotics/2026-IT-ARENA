#!/usr/bin/env python3
"""모터축 카운트의 양자화·통신/계수 고장·단일 관측 한계 검사. ROS/실물 시험이 아니다."""
import argparse
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/arena_vehicle_interface'))
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig
from arena_vehicle_interface.drive_feedback import MODE_MOTOR, MotorEncoderSpec
from arena_vehicle_interface.mcu_speed_loop import SpeedPI

SPEC = MotorEncoderSpec(28, 15., .0325)
CONFIG = ContractConfig(feedback_mode=MODE_MOTOR, wheel_radius_m=.0325, ticks_per_rev=28, gear_ratio=15.)


def quantization(speed, period_us):
    receiver = ActuationReceiver(CONFIG)
    observed, stationary = [], []
    for i in range(4_000_000 // period_us + 1):
        t = i * period_us
        counts = round(speed * t / 1e6 / SPEC.mean_wheel_travel_m_per_count)
        receiver.update_motor_encoder(t, sequence=i, capture_us=t, motor_counts=counts)
        receiver.tick(t)
        if i:
            observed.append(receiver.mean_wheel_mps)
        if t >= 300_000:
            stationary.append(receiver._stationary(t))
    error = [value - speed for value in observed]
    return dict(speed_mps=speed, sample_period_ms=period_us / 1000,
                mean_error_mps=statistics.mean(error), max_abs_error_mps=max(map(abs, error)),
                zero_sample_fraction=observed.count(0.) / len(observed),
                stationary_ready_fraction=sum(stationary) / len(stationary),
                passed=max(map(abs, error)) <= SPEC.speed_quantum_mps(period_us / 1e6) + 1e-9)


def fault_case(kind):
    """실제 이동은 평가용이다. 수신기에는 시각과 정수 카운트만 전달한다."""
    receiver, host = ActuationReceiver(CONFIG), CommandProducer()
    queue, observed = [], []
    first_fault, last_counts, frozen_counts = None, None, None
    max_age, largest_delta = 0, 0
    offset = 2**31 - 50 if kind == 'signed32_wrap' else 100_000
    for t in range(0, 1_800_001, 1000):
        if t % 10_000 == 0:
            travel = max(0, t - 300_000) / 1e6  # 평가용 일정 1 m/s 이동
            raw = round(travel / SPEC.mean_wheel_travel_m_per_count)
            counts = offset + (math.floor(raw * .9) if kind == 'pulse_loss_10pct' else raw)
            if kind == 'signed32_wrap':
                counts = (counts + 2**31) % 2**32 - 2**31
            if t >= 800_000:
                if kind == 'counter_reset':
                    counts -= offset + round(.5 / SPEC.mean_wheel_travel_m_per_count)
                if kind == 'stuck_counts':
                    frozen_counts = counts if frozen_counts is None else frozen_counts
                    counts = frozen_counts
                if kind == 'one_count_noise':
                    counts += (t // 10_000) % 2
            drop = ((kind == 'one_packet_lost' and t == 800_000) or
                    (kind == 'burst_60ms' and 800_000 <= t < 860_000))
            latency = {'delay_2ms': 2000, 'delay_20ms': 20_000, 'delay_50ms': 50_000}.get(kind, 0)
            if kind == 'jitter_reorder':
                latency = [2000, 18_000, 3000, 4000][t // 10_000 % 4]
            if not drop:
                queue.append((t + latency, t, counts))
        due = sorted(item for item in queue if item[0] <= t)
        queue = [item for item in queue if item[0] > t]
        for _, capture, counts in due:
            accepted = receiver.update_motor_encoder(t, sequence=capture // 10_000,
                                                     capture_us=capture, motor_counts=counts)
            if accepted and receiver.feedback_valid:
                max_age = max(max_age, t - capture)
                if last_counts is not None:
                    largest_delta = max(largest_delta, abs(counts - last_counts))
                last_counts = counts
                if t > 500_000:
                    observed.append(receiver.mean_wheel_mps)
        receiver.tick(t)
        if t == 300_000 and receiver._stationary(t):
            receiver.receive(host.control(receiver.offer(t), 'ARM'), t)
        if t >= 300_000 and t % 20_000 == 0 and receiver.armed:
            receiver.receive(host.drive(receiver.offer(t), speed_mps=1., steering_rad=0.,
                                        source_us=t, host_now_us=t), t)
        if receiver.state.value == 'FAULT_LATCHED' and first_fault is None:
            first_fault = dict(time_us=t, reason=receiver.reason)
    expected_fault = {
        'burst_60ms': 'encoder_timeout', 'counter_reset': 'encoder_implausible',
        'signed32_wrap': 'encoder_implausible',
        'stuck_counts': 'motor_feedback_no_progress',
    }.get(kind)
    if kind == 'delay_50ms':
        passed = not receiver.armed and not receiver.feedback_valid
    elif expected_fault:
        passed = first_fault is not None and first_fault['reason'] == expected_fault and receiver.target_speed == 0
    else:
        passed = first_fault is None and receiver.armed
    return dict(case=kind, passed=passed, first_fault=first_fault, final_state=receiver.state.value,
                final_reason=receiver.reason, final_target_mps=receiver.target_speed,
                max_accepted_age_ms=max_age / 1000, largest_count_delta=largest_delta,
                reported_mean_mps=statistics.mean(observed) if observed else None,
                final_stationary_ready=receiver._stationary(1_800_000))


def toy_pi(target, kind):
    """임시 1차원 모형. PWM·전류·실측 모터 특성이나 Gazebo 폐루프 시험이 아니다."""
    pi = SpeedPI()
    dt = .005
    velocity = distance = measured = 0.
    previous = (0., 0)
    pending = deque()
    steady, tail = [], []
    effort_history = []
    for i in range(1, 2001):
        t = i * dt
        # 전진만 가능한 임시 모형. 음의 effort/정지 명령은 2 m/s^2의 모의 제동이다.
        command = target if t < 8 else 0.
        if i % 2 == 0:
            counts = round(distance / SPEC.mean_wheel_travel_m_per_count)
            if kind == 'pulse_loss_10pct':
                counts = math.floor(counts * .9)
            speed = (counts - previous[1]) * SPEC.mean_wheel_travel_m_per_count / (t - previous[0])
            previous = (t, counts)
            pending.append((t + (.02 if kind == 'delay_20ms' else .002), speed))
        while pending and pending[0][0] <= t + 1e-12:
            _, measured = pending.popleft()
        feedback = velocity if kind == 'ideal_reference' else measured
        intent = pi.update_mean(command, feedback, dt, enabled=True)
        effort_history.append(intent.effort)
        if intent.brake_requested:
            velocity = max(0., velocity - 2 * dt)
        else:
            velocity = max(0., velocity + (3 * intent.effort - .3 * velocity) * dt)
        distance += velocity * dt
        if 5 <= t < 8:
            steady.append(velocity)
        if t >= 8:
            tail.append(velocity)
    return dict(target_mps=target, condition=kind, steady_mean_mps=statistics.mean(steady),
                steady_peak_to_peak_mps=max(steady) - min(steady),
                steady_mean_error_mps=statistics.mean(steady) - target, final_speed_mps=velocity,
                output_bounded=all(abs(v) <= 1 for v in effort_history),
                stopped=velocity == 0, passed=velocity == 0 and all(abs(v) <= 1 for v in effort_history))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--sensor-only', action='store_true', help='고장 재현용. PI 검사 이전에도 사용한다.')
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT / 'artifacts'):
        raise ValueError('output must be under artifacts')
    output.mkdir(parents=True, exist_ok=False)
    paths = [Path(__file__), *(ROOT / 'src/arena_vehicle_interface/arena_vehicle_interface' / (n + '.py')
                               for n in ('actuation_contract', 'drive_feedback', 'mcu_speed_loop'))]
    report = dict(scope='deterministic count/contract and optional toy PI; not Gazebo or hardware',
                  spec=asdict(SPEC), contract=asdict(CONFIG),
                  source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths})
    report['quantization'] = [quantization(v, dt) for dt in (10_000, 20_000)
                              for v in (0., .001, .005, .01, .02, .03, .05, .1, .3, .6, 1.4, 2.5, 4.08)]
    report['faults'] = [fault_case(k) for k in ('baseline', 'delay_2ms', 'delay_20ms', 'delay_50ms',
                        'jitter_reorder', 'one_packet_lost', 'burst_60ms', 'counter_reset',
                        'signed32_wrap', 'stuck_counts', 'pulse_loss_10pct', 'one_count_noise')]
    report['toy_pi'] = [] if args.sensor_only else [toy_pi(v, k) for v in (.05, .1, .6, 1.4, 2.5)
                      for k in ('ideal_reference', 'quantized', 'delay_20ms', 'pulse_loss_10pct')]
    report['limitations'] = [
        'Plausible pulse loss, slow counter reset, noisy stuck feedback and opposite wheel rotation remain ambiguous.',
        'Stationary-ready means low mean motor-shaft speed only, not actual vehicle standstill.',
        'Toy PI passes mean bounded effort and eventual model stop, not hardware tuning or tracking certification.',
    ]
    report['passed'] = all(row['passed'] for key in ('quantization', 'faults', 'toy_pi') for row in report[key])
    (output / 'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding='utf-8')
    print(json.dumps({'passed': report['passed'], 'faults': report['faults'], 'output': str(output)}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
