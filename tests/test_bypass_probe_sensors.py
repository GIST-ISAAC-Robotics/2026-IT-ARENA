"""측정 생산자의 기어비/정수 카운트/현재 취득 시각과 선언한 오차만 검사한다."""
import math
import numpy as np
import pytest
from bypass_probe_sensors import ProbeSensors,SPEC


def test_single_motor_quantization_uses_gearing_and_past_interval():
    sensor=ProbeSensors('motion')
    sensor.measure_motion(0.,0.,0.,0.)
    rates=[];counts=[]
    for k in range(1,101):
        mean,gyro,count=sensor.measure_motion(k*.02,.35*k*.02,.35,0.)
        rates.append(mean*.0325);counts.append(count)
    assert SPEC.counts_per_wheel_revolution==420
    assert counts[-1]==round(.7/SPEC.mean_wheel_travel_m_per_count)
    assert max(abs(v-.35) for v in rates)<SPEC.speed_quantum_mps(.02)+1e-9
    # 출력 상한보다 큰 측정값이 양자화만으로 가능함을 보존한다.
    assert max(rates)>.35 and min(rates)<.35
    assert abs(np.mean(rates)-.35)<SPEC.speed_quantum_mps(2.)


def test_range_noise_is_seeded_bounded_and_does_not_fill_invalid_returns():
    a,b=ProbeSensors('range',23),ProbeSensors('range',23)
    values=[a.range_value(1.) for _ in range(100)]
    assert values==[b.range_value(1.) for _ in range(100)]
    assert min(values)>=.97 and max(values)<=1.03
    assert math.isinf(a.range_value(math.inf))
    assert math.isnan(a.range_value(math.nan))


def test_profiles_do_not_silently_add_other_sensor_errors():
    plain=ProbeSensors();rng=ProbeSensors('range');motion=ProbeSensors('motion')
    assert plain.range_value(1.)==1. and plain.actual_lidar_yaw==0.
    assert not rng.motion_enabled and motion.range_value(1.)==1.
    assert motion.actual_lidar_yaw==pytest.approx(math.radians(-.5))
    assert not motion.description()['measured_distribution']


def test_repeated_encoder_capture_time_is_not_rewritten():
    sensor=ProbeSensors('motion');sensor.measure_motion(.5,0.,0.,0.)
    with pytest.raises(ValueError):sensor.measure_motion(.5,.1,.3,0.)
