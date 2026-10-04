"""숫자 카운트 시험기의 독립 기대값. ROS/Gazebo를 실행하지 않는다."""
import importlib.util
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location('motor_encoder_validation',
        Path(__file__).resolve().parents[1] / 'scripts/validate_motor_encoder.py')
bench = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bench)


@pytest.mark.parametrize('speed', [0., .001, .01, .1, .6, 2.5])
def test_fixed_interval_quantization_has_one_count_error_bound(speed):
    row = bench.quantization(speed, 10_000)
    assert row['passed']
    assert abs(row['mean_error_mps']) < .00013


@pytest.mark.parametrize('kind,reason,when', [
    ('burst_60ms', 'encoder_timeout', 840_000),
    ('counter_reset', 'encoder_implausible', 800_000),
    ('signed32_wrap', 'encoder_implausible', 330_000),
    ('stuck_counts', 'motor_feedback_no_progress', 1_300_000),
])
def test_fault_time_and_latched_zero(kind, reason, when):
    row = bench.fault_case(kind)
    assert row['passed']
    assert row['first_fault'] == {'time_us': when, 'reason': reason}
    assert row['final_target_mps'] == 0.


def test_packet_loss_and_pulse_loss_are_not_the_same():
    packet = bench.fault_case('one_packet_lost')
    pulse = bench.fault_case('pulse_loss_10pct')
    assert packet['reported_mean_mps'] == pytest.approx(1., abs=.001)
    assert pulse['reported_mean_mps'] == pytest.approx(.9, abs=.001)
    assert packet['first_fault'] is None and pulse['first_fault'] is None


def test_toy_pi_pulse_loss_exposes_speed_bias_not_a_successful_accuracy_verdict():
    clean = bench.toy_pi(2.5, 'quantized')
    loss = bench.toy_pi(2.5, 'pulse_loss_10pct')
    assert clean['stopped'] and loss['stopped']
    assert clean['steady_mean_mps'] == pytest.approx(2.5, abs=.01)
    assert loss['steady_mean_mps'] > 2.75
