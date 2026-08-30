import math

import pytest

from arena_vehicle_interface.ackermann_to_twist import AckermannToTwist
from arena_vehicle_interface.sim_wheel_encoder import (
    position_to_ticks,
    radians_per_tick,
    tick_delta_to_velocity,
)


def test_command_clamp_respects_both_limits():
    assert AckermannToTwist._clamp(3.0, -2.0, 2.0) == 2.0
    assert AckermannToTwist._clamp(-3.0, -2.0, 2.0) == -2.0
    assert AckermannToTwist._clamp(0.5, -2.0, 2.0) == 0.5


def test_quarter_turn_quantizes_to_expected_count():
    tick_angle = radians_per_tick(2048)
    assert position_to_ticks(math.pi / 2.0, tick_angle) == 512


def test_tick_velocity_uses_quantized_delta():
    tick_angle = radians_per_tick(2048)
    velocity = tick_delta_to_velocity(120, 100, tick_angle, 0.01)
    assert velocity == pytest.approx(20 * tick_angle / 0.01)


def test_invalid_encoder_resolution_is_rejected():
    with pytest.raises(ValueError, match="must be positive"):
        radians_per_tick(0)
