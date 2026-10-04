"""모터축 PGE-213 단일 피드백 계약. ROS·시뮬레이션 없이 실행한다."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/arena_vehicle_interface"))
from arena_vehicle_interface.drive_feedback import (  # noqa: E402
    LEGACY_JOINTS, MODE_LEGACY, MODE_MOTOR, MOTOR_JOINT, MOTOR_TOPIC, MotorEncoderSpec,
    motor_angle_from_wheels, parse_legacy_wheels, parse_motor_state, validate_mode,
)

SPEC = MotorEncoderSpec(counts_per_motor_revolution=28, gear_ratio=15.0, wheel_radius_m=.0325)


def counts_for_wheels(left_rad, right_rad, spec=SPEC):
    return spec.counts_from_motor_angle(motor_angle_from_wheels(left_rad, right_rad, spec.gear_ratio))


def test_gear_ratio_is_applied_to_wheel_mean_resolution():
    assert SPEC.counts_per_wheel_revolution == pytest.approx(420)
    assert SPEC.mean_wheel_travel_m_per_count * 1000 == pytest.approx(.48620, abs=1e-5)
    # 평균 바퀴 1회전 = 모터 15회전 = 420카운트.
    assert counts_for_wheels(math.tau, math.tau) == 420
    assert SPEC.mean_wheel_rad_s(15 * math.tau) == pytest.approx(math.tau)


def test_documented_quantization_steps_are_reproduced():
    step_kmh = SPEC.speed_quantum_mps(.020) * 3.6
    assert step_kmh == pytest.approx(.0875, abs=1e-4)
    for speed_kmh, expected in ((10, 114), (1, 11), (9, 103)):
        travel = speed_kmh / 3.6 * .020
        assert round(travel / SPEC.mean_wheel_travel_m_per_count) == expected
    # 시뮬레이션 임시 100 Hz 표본에서의 한 카운트 속도 단계(모델 분해능과 별개인 표본율 영향).
    assert SPEC.speed_quantum_mps(.010) == pytest.approx(.04862, abs=1e-5)


def test_bidirectional_and_straight_motion_are_symmetric():
    forward = counts_for_wheels(10., 10.)
    reverse = counts_for_wheels(-10., -10.)
    assert forward == -reverse == round(15 * 10 / SPEC.motor_rad_per_count)
    delta = forward - counts_for_wheels(0., 0.)
    assert SPEC.motor_rad_s_from_counts(delta, 1.) / SPEC.gear_ratio == pytest.approx(10., abs=.02)


@pytest.mark.parametrize("left,right", [(100., 100.), (0., 200.), (200., 0.), (-50., 250.)])
def test_differential_cases_with_same_mean_are_one_motor_observation(left, right):
    # 좌우 100/100과 0/200은 모터축에서 같은 값이다. 차이는 관측할 수 없다.
    assert counts_for_wheels(left, right) == counts_for_wheels(100., 100.)


def test_opposite_wheel_rotation_is_invisible_to_motor_shaft():
    assert counts_for_wheels(5., -5.) == counts_for_wheels(0., 0.) == 0


def test_low_speed_quantization_reports_zero_between_counts():
    # 0.01 m/s, 10 ms 표본: 0.1 mm 이동은 한 카운트(0.486 mm)보다 작다.
    speed, dt = .01, .010
    wheel_rate = speed / SPEC.wheel_radius_m
    counts = [counts_for_wheels(wheel_rate * k * dt, wheel_rate * k * dt) for k in range(0, 60)]
    velocities = [SPEC.mean_wheel_mps(SPEC.motor_rad_s_from_counts(b - a, dt)) for a, b in zip(counts, counts[1:])]
    assert set(round(v, 6) for v in velocities) <= {0.0, round(SPEC.speed_quantum_mps(dt), 6)}
    assert velocities.count(0.0) > len(velocities) // 2
    assert sum(velocities) / len(velocities) == pytest.approx(speed, rel=.15)


def test_position_round_trip_and_off_grid_rejection():
    assert SPEC.counts_from_position(SPEC.motor_angle_from_counts(-1234)) == -1234
    with pytest.raises(ValueError):
        SPEC.counts_from_position(SPEC.motor_rad_per_count * 10.4)
    with pytest.raises(ValueError):
        SPEC.counts_from_position(float("nan"))


@pytest.mark.parametrize('value', [1e308, -1e308, (2**63) * SPEC.motor_rad_per_count])
def test_extreme_finite_motor_position_is_a_validation_error_not_overflow(value):
    with pytest.raises(ValueError):
        SPEC.counts_from_position(value)


@pytest.mark.parametrize("kwargs", [
    dict(counts_per_motor_revolution=0, gear_ratio=15., wheel_radius_m=.0325),
    dict(counts_per_motor_revolution=28., gear_ratio=15., wheel_radius_m=.0325),
    dict(counts_per_motor_revolution=28, gear_ratio=0., wheel_radius_m=.0325),
    dict(counts_per_motor_revolution=28, gear_ratio=float("nan"), wheel_radius_m=.0325),
    dict(counts_per_motor_revolution=28, gear_ratio=15., wheel_radius_m=float("inf")),
])
def test_invalid_spec_rejected(kwargs):
    with pytest.raises(ValueError):
        MotorEncoderSpec(**kwargs)


def test_motor_message_contract_rejects_names_counts_and_nonfinite():
    assert parse_motor_state([MOTOR_JOINT], [1.], [2.]) == (1., 2.)
    for names in ([], ["car::" + MOTOR_JOINT], [MOTOR_JOINT, LEGACY_JOINTS[0]], list(LEGACY_JOINTS)):
        with pytest.raises(ValueError):
            parse_motor_state(names, [0.] * len(names), [0.] * len(names))
    for position, velocity in ((float("nan"), 0.), (0., float("inf"))):
        with pytest.raises(ValueError):
            parse_motor_state([MOTOR_JOINT], [position], [velocity])
    with pytest.raises(ValueError):
        parse_motor_state([MOTOR_JOINT], [], [1.])
    # 호출자가 비유한 값을 직접 거절·기록하는 경로만 값 검사를 미룬다.
    assert math.isnan(parse_motor_state([MOTOR_JOINT], [0.], [float("nan")], require_finite=False)[1])


def test_legacy_pair_requires_both_names_and_never_reads_motor_channel():
    assert parse_legacy_wheels(["car::rear_right_wheel_joint", "rear_left_wheel_joint"], [6., 2.]) == (2., 6.)
    with pytest.raises(ValueError):
        parse_legacy_wheels([MOTOR_JOINT], [1.])
    with pytest.raises(ValueError):
        parse_legacy_wheels([LEGACY_JOINTS[0], MOTOR_JOINT], [1., 2.])
    with pytest.raises(ValueError):
        parse_legacy_wheels([LEGACY_JOINTS[0]], [1.])
    with pytest.raises(ValueError):
        parse_legacy_wheels(list(LEGACY_JOINTS) + [LEGACY_JOINTS[0]], [1., 2., 3.])


def test_mode_names_are_explicit():
    assert validate_mode(MODE_MOTOR) == MODE_MOTOR and validate_mode(MODE_LEGACY) == MODE_LEGACY
    assert MOTOR_TOPIC == "/drive_motor/encoder"
    for bad in ("", "configured", "wheel_pair", None):
        with pytest.raises(ValueError):
            validate_mode(bad)
