from collections import deque
import math
from types import SimpleNamespace

from sensor_msgs.msg import JointState

from arena_vehicle_interface.sim_wheel_encoder import SimWheelEncoder, radians_per_tick


def encoder_double():
    encoder = SimpleNamespace(
        _joint_names=("rear_left_wheel_joint", "rear_right_wheel_joint"),
        _pending=deque(), _last_clock_time_ns=None, _last_capture_time_ns=None, _last_published=None,
        _sample_period_ns=10_000_000, _latency_ns=2_000_000, _radians_per_tick=radians_per_tick(2048),
        _random=SimpleNamespace(random=lambda: .5), _dropout_probability=0.,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10_030_000_000)))
    encoder._check_clock_reset = lambda now: SimWheelEncoder._check_clock_reset(encoder, now)
    encoder._find_joint_index = lambda message, target: SimWheelEncoder._find_joint_index(encoder, message, target)
    return encoder


def source_message():
    message = JointState()
    message.header.stamp.sec = 10
    message.header.stamp.nanosec = 5_000_000
    message.name = ["rear_left_wheel_joint", "rear_right_wheel_joint"]
    message.position = [1., 2.]
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
    encoder = encoder_double()
    message = source_message()
    message.position[0] = math.nan
    SimWheelEncoder._capture(encoder, message)
    assert not encoder._pending
