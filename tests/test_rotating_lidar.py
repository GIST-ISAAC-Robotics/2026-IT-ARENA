import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/arena_vehicle_interface"))
from arena_vehicle_interface.rotating_lidar import RevolutionAssembler


def test_moving_wall_uses_different_physics_instants_and_waits_full_turn():
    sensor = RevolutionAssembler()
    output = []
    for i in range(101):
        current = sensor.feed(i / 1000, [10 - 5 * i / 1000] * 500)
        if i < 100:
            assert not current
        output.extend(current)
    first, ranges, error = output[0]
    assert first == 0
    assert ranges[0] == 10
    assert ranges[250] == pytest.approx(9.75)
    assert ranges[-1] == pytest.approx(9.505)
    assert error <= .00100001
    assert len(set(ranges)) >= 99


def test_stationary_world_matches_snapshot_and_has_ten_scans_per_second():
    sensor = RevolutionAssembler()
    fixed = [1 + i / 500 for i in range(500)]
    outputs = []
    for i in range(1001):
        outputs.extend(sensor.feed(i / 1000, fixed))
    assert len(outputs) == 10
    assert all(ranges == fixed for _, ranges, _ in outputs)
    assert outputs[-1][0] == pytest.approx(.9)


def test_source_dropout_discards_turn_instead_of_inventing_measurements():
    sensor = RevolutionAssembler()
    sensor.feed(0, [1.] * 500)
    sensor.feed(.001, [1.] * 500)
    assert sensor.feed(.010, [2.] * 500) == []
    assert sensor.discarded == 1
    result = []
    for i in range(11, 111):
        result.extend(sensor.feed(i / 1000, [2.] * 500))
    assert result[0][0] == .010
    assert result[0][1] == [2.] * 500


def test_clock_reset_duplicate_and_invalid_input():
    sensor = RevolutionAssembler()
    sensor.feed(1., [math.inf] * 500)
    assert sensor.feed(1., [2.] * 500) == []
    assert sensor.feed(0., [3.] * 500) == []
    assert sensor.start == 0
    with pytest.raises(ValueError):
        sensor.feed(.001, [1.])


def test_two_millisecond_physics_frames_keep_time_error_bounded():
    sensor = RevolutionAssembler()
    outputs = []
    for i in range(501):
        outputs.extend(sensor.feed(i * .002, [10 - i * .002] * 500))
    assert len(outputs) == 10
    assert max(row[2] for row in outputs) <= .00200001
    assert sensor.discarded == 0
