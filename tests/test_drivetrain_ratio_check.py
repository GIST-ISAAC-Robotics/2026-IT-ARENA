"""감속비 검토 계산의 모델 일관성 회귀."""
import math
from pathlib import Path
import sys

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
from drivetrain_ratio_check import (  # noqa: E402
    MotorPoints, Vehicle, evaluate, main, motor_model, recommended_ratio, rpm_to_rad_s,
)


def test_model_reproduces_both_datasheet_points():
    p = MotorPoints()
    m = motor_model(p)
    for rpm, amps in ((p.no_load_rpm, p.no_load_a), (p.rated_rpm, p.rated_a)):
        predicted = (p.voltage_v - amps * m["resistance_ohm"]) / m["ke_v_s_per_rad"]
        assert predicted == pytest.approx(rpm_to_rad_s(rpm), rel=1e-9)
    # 독립적으로 구한 Kt와 Ke가 SI 단위에서 근사 일치해야 선형 모델이 자기모순이 없다.
    assert m["kt_nm_per_a"] == pytest.approx(m["ke_v_s_per_rad"], rel=0.05)
    # 판매 요약의 기동 토크 12 kgf·cm와 같은 범위
    assert 11 < m["stall_torque_kgfcm"] < 14
    assert 90 < m["stall_a_at_rated_voltage"] < 110


def test_recommended_ratio_puts_design_speed_at_headroom_and_scales_with_wheel():
    v = Vehicle()
    for d in (0.05, 0.065, 0.08):
        n = recommended_ratio(d, 15.0, vehicle=v)
        motor_rpm = 15 / 3.6 / (math.pi * d) * 60 * n
        no_load = MotorPoints().no_load_rpm * v.battery_v / MotorPoints().voltage_v
        assert motor_rpm == pytest.approx(0.7 * no_load)
    assert recommended_ratio(0.08, 15.0) / recommended_ratio(0.04, 15.0) == pytest.approx(2.0)
    with pytest.raises(ValueError):
        recommended_ratio(0.05, 15.0, headroom=0)


def test_higher_ratio_lowers_acceleration_current_and_speed_sensitivity():
    low, high = evaluate(2.5, 0.05), evaluate(7.5, 0.05)
    assert high["traction_limit_motor_a"] < low["traction_limit_motor_a"]
    assert (high["speeds"]["9"]["fixed_pwm_drop_fraction_at_traction_load"]
            < low["speeds"]["9"]["fixed_pwm_drop_fraction_at_traction_load"])
    assert high["encoder_mm_per_count"] == pytest.approx(low["encoder_mm_per_count"] / 3)
    # 바퀴가 막힌 전류는 PWM과 권선 저항으로만 정해진다.
    assert high["blocked_current_a_by_duty"] == low["blocked_current_a_by_duty"]


def test_invalid_geometry_is_rejected_and_cli_runs(capsys):
    with pytest.raises(ValueError):
        evaluate(0, 0.05)
    assert main(["--wheel-mm", "65", "--ratio", "2.5", "9.5", "--design-top-kmh", "15"]) == 0
    out = capsys.readouterr().out
    assert "권장 전체 감속비" in out and "| 9.5 | 65 |" in out
