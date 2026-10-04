from collections import deque
import math
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import JointState

from arena_vehicle_interface.drive_feedback import (
    MODE_LEGACY, MODE_MOTOR, MOTOR_FRAME, MOTOR_JOINT, MotorEncoderSpec,
)
from arena_vehicle_interface.sim_wheel_encoder import SimWheelEncoder, radians_per_tick


def encoder_double(mode=MODE_LEGACY, dropout=0.):
    encoder = SimpleNamespace(
        _mode=mode, _spec=MotorEncoderSpec(28, 15., .0325) if mode == MODE_MOTOR else None,
        _joint_names=("rear_left_wheel_joint", "rear_right_wheel_joint"),
        _pending=deque(), _last_clock_time_ns=None, _last_capture_time_ns=None, _last_published=None,
        _sample_period_ns=10_000_000, _latency_ns=2_000_000, _radians_per_tick=radians_per_tick(2048),
        _random=SimpleNamespace(random=lambda: .5), _dropout_probability=dropout,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_030_000_000)))
    encoder._check_clock_reset = lambda now: SimWheelEncoder._check_clock_reset(encoder, now)
    encoder._find_joint_index = lambda message, target: SimWheelEncoder._find_joint_index(encoder, message, target)
    encoder._motor_state = lambda sample: SimWheelEncoder._motor_state(encoder, sample)
    encoder.published = []
    encoder._motor_publisher = SimpleNamespace(publish=encoder.published.append)
    return encoder


def source_message(left=1., right=2., sec=10, nanosec=5_000_000):
    message = JointState()
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec
    message.name = ["rear_left_wheel_joint", "rear_right_wheel_joint"]
    message.position = [left, right]
    return message


def test_encoder_uses_source_capture_time_not_ros_arrival_time():
    encoder = encoder_double()
    SimWheelEncoder._capture(encoder, source_message())
    sample = encoder._pending[0]
    assert sample.capture_time_ns == 10_005_000_000
    assert sample.due_time_ns == 10_007_000_000


def test_encoder_clock_rewind_discards_old_history_and_delayed_samples():
    encoder = encoder_double()
    SimWheelEncoder._capture(encoder, source_message())
    encoder._last_published = encoder._pending[0]
    encoder._check_clock_reset(1_000_000)
    assert not encoder._pending
    assert encoder._last_capture_time_ns is None and encoder._last_published is None


def test_encoder_rejects_nonfinite_joint_positions():
    for mode in (MODE_LEGACY, MODE_MOTOR):
        encoder = encoder_double(mode)
        message = source_message()
        message.position[0] = math.nan
        SimWheelEncoder._capture(encoder, message)
        assert not encoder._pending


@pytest.mark.parametrize("left,right", [(100., 100.), (0., 200.)])
def test_motor_mode_keeps_only_quantized_average_motor_angle(left, right):
    encoder = encoder_double(MODE_MOTOR)
    SimWheelEncoder._capture(encoder, source_message(left, right))
    sample = encoder._pending[0]
    assert sample.left_ticks is None and sample.right_ticks is None
    assert sample.motor_counts == round(15 * 100 / (math.tau / 28))
    assert sample.capture_time_ns == 10_005_000_000 and sample.due_time_ns == 10_007_000_000


def test_motor_mode_publishes_one_named_channel_with_capture_interval_velocity():
    encoder = encoder_double(MODE_MOTOR)
    SimWheelEncoder._capture(encoder, source_message(0., 0., nanosec=0))
    SimWheelEncoder._capture(encoder, source_message(1., 1., nanosec=20_000_000))
    first, second = encoder._pending
    SimWheelEncoder._publish(encoder, first)
    SimWheelEncoder._publish(encoder, second)
    a, b = encoder.published
    for state in (a, b):
        assert list(state.name) == [MOTOR_JOINT] and state.header.frame_id == MOTOR_FRAME
        assert len(state.position) == len(state.velocity) == 1
    # 첫 표본은 속도 기준이 없으므로 0으로 꾸미지 않는다.
    assert math.isnan(a.velocity[0])
    counts = second.motor_counts - first.motor_counts
    assert b.velocity[0] == pytest.approx(counts * (math.tau / 28) / .020)
    assert b.velocity[0] / 15 == pytest.approx(1. / .020, rel=.01)
    assert b.header.stamp.nanosec == 20_000_000


def test_motor_mode_sampling_period_and_dropout_keep_source_stamps():
    encoder = encoder_double(MODE_MOTOR)
    SimWheelEncoder._capture(encoder, source_message(nanosec=0))
    SimWheelEncoder._capture(encoder, source_message(nanosec=5_000_000))  # 10 ms 주기 이전 표본은 생략
    assert len(encoder._pending) == 1
    dropped = encoder_double(MODE_MOTOR, dropout=1.)
    SimWheelEncoder._capture(dropped, source_message(nanosec=0))
    assert not dropped._pending and dropped._last_capture_time_ns == 10_000_000_000


def test_motor_mode_velocity_after_clock_reset_restarts_without_reference():
    encoder = encoder_double(MODE_MOTOR)
    SimWheelEncoder._capture(encoder, source_message(nanosec=0))
    SimWheelEncoder._publish(encoder, encoder._pending.popleft())
    encoder._check_clock_reset(1_000_000)
    encoder._last_clock_time_ns = None
    encoder.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=1_030_000_000))
    SimWheelEncoder._capture(encoder, source_message(5., 5., sec=1, nanosec=0))
    SimWheelEncoder._publish(encoder, encoder._pending.popleft())
    assert math.isnan(encoder.published[-1].velocity[0])
