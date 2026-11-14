"""PI의 유한 입력 산술 범위. 극단값은 실차 측정값이 아니라 API 고장 반례다."""
import math

import pytest

from arena_vehicle_interface.mcu_speed_loop import SpeedPI


@pytest.mark.parametrize('gains,target,measured,dt', [
    ((.6, 1.2, .05), 1e308, -1e308, .005),  # 차분 overflow
    ((1e308, 1.2, .05), 2., 0., .005),      # 비례항 overflow
    ((.6, 1e308, .05), 2., 0., .005),       # 적분 증가량 overflow
    ((.6, 1.2, .05), 10**308, -10**308, .005),  # int 차분 → float 환산
    ((.6, 10**308, .05), 2, 0, .005),           # int 곱 → float 환산
])
def test_finite_inputs_with_nonfinite_intermediate_reset_and_inhibit(gains, target, measured, dt):
    assert all(math.isfinite(x) for x in (*gains, target, measured, dt))
    pi = SpeedPI(*gains)
    result = pi.update_mean(target, measured, dt, enabled=True)
    assert result.effort == 0. and result.brake_requested is True
    assert pi.integral == 0.


def test_normal_inputs_preserve_existing_pi_equation_and_reset():
    pi = SpeedPI()
    first = pi.update_mean(1., .5, .005, enabled=True)
    assert first.effort == pytest.approx(.303) and not first.brake_requested
    assert pi.integral == pytest.approx(.003)
    assert pi.update_mean(0., .5, .005, enabled=True).brake_requested
    assert pi.integral == 0.
    assert pi.update_mean(1., .5, .005, enabled=True).effort == first.effort
