from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_zero_speed_modes import model_without_mode, phase_stats, zero_phase


def test_model_normalization_preserves_every_other_setting(tmp_path):
    a, b = tmp_path/'a.sdf', tmp_path/'b.sdf'
    text = '<sdf><model><plugin name="arena::SingleMotorDrive"><gear_ratio>15</gear_ratio>{}</plugin></model></sdf>'
    a.write_text(text.format(''))
    b.write_text(text.format('<evaluation_zero_speed_mode>no_lag</evaluation_zero_speed_mode>'))
    assert model_without_mode(a)[0] == 'legacy'
    assert model_without_mode(b)[0] == 'no_lag'
    assert model_without_mode(a)[1] == model_without_mode(b)[1]
    b.write_text(b.read_text().replace('>15<', '>16<'))
    assert model_without_mode(a)[1] != model_without_mode(b)[1]


def sample(i, target):
    return dict(sim_time_s=i*.02, applied_target_mps=target, truth_longitudinal_mps=.01,
        truth_lateral_mps=0., wheel_surface_speed_mps=.01, truth_x_m=i*.001, truth_y_m=0.,
        zero_positive_work_j=0. if i<10 else .1, zero_positive_power_steps=0 if i<10 else 3,
        pre_motor_power_w=-.1, pi_integral_after_nm=0.)


def test_zero_transition_and_chord_require_stable_tail():
    data = [sample(i, 1. if i<10 else 0.) for i in range(60)]
    data[10]['truth_longitudinal_mps'] = .5
    result = zero_phase(data, .05)
    assert result['first_zero_sim_s'] == .2
    assert result['last_positive_sim_s'] == .18
    assert result['stable_sim_s'] == .22
    assert result['zero_to_stable_chord_m'] == pytest.approx(.001)
    assert result['zero_positive_work_j'] == .1
    assert result['zero_positive_power_steps'] == 3
    data[-1]['truth_longitudinal_mps'] = .5
    assert zero_phase(data, 0.)['stable_sim_s'] is None


def test_insufficient_zero_data_rejected():
    with pytest.raises(ValueError):
        zero_phase([sample(i, 0.) for i in range(40)], 0.)
    with pytest.raises(ValueError):
        zero_phase([sample(i, 1. if i<10 else 0.) for i in range(20)], 0.)


def test_phase_uses_pre_power_not_post_power():
    data = [sample(0, 0.), sample(1, 0.)]
    data[1].update(wheel_surface_speed_mps=-.3, pre_motor_power_w=.2)
    result = phase_stats(data)
    assert result['peak_difference_mps'] == pytest.approx(.31)
    assert result['positive_pre_power_samples'] == 1
    assert phase_stats([]) is None
