import copy
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from summarize_motion_robustness import dynamic_audit


def trace():
    result = []
    for i in range(75, 125):
        t = i/10
        stopped = 8.5 <= t < 10.2
        speed = 0. if stopped else 1.
        result.append({'time': t, 'x': t if t < 8.5 else 8.5 if t < 10.2 else t-1.7, 'y': 0.,
                       'progress_m': t if t < 8.5 else 8.5 if t < 10.2 else t-1.7,
                       'dynamics': {'truth_longitudinal_mps': speed, 'truth_lateral_mps': 0.},
                       'safe_drive_speed_mps': 0. if 8.1 <= t < 10.2 else 1.,
                       'safety': {'reason': 'motion_stale' if 8.1 <= t < 10.2 else 'clear'}})
    return result


CONFIG = {'imu': {'drop': True, 'start_s': 8., 'end_s': 10.}}


def test_moving_fault_requires_actual_stop_and_recovery():
    result = dynamic_audit(trace(), CONFIG)
    assert result['passed'] and result['stable_stop_during_fault'] and result['resumed_more_than_1m']
    assert .09 < result['first_zero_safety_after_fault_s'] < .11


def test_zero_command_without_actual_stop_is_not_success():
    data = trace()
    for row in data:
        row['dynamics']['truth_longitudinal_mps'] = 1.
    assert not dynamic_audit(data, CONFIG)['passed']


def test_startup_stillness_is_not_moving_vehicle_braking():
    data = trace()
    for row in data:
        if row['time'] < 8:
            row['dynamics']['truth_longitudinal_mps'] = 0.
    assert not dynamic_audit(data, CONFIG)['passed']


def test_missing_recovery_and_late_protection_rejected():
    assert not dynamic_audit(trace()[:30], CONFIG)['passed']
    data = copy.deepcopy(trace())
    for row in data:
        if row['time'] < 8.4:
            row['safety']['reason'] = 'clear'
    assert not dynamic_audit(data, CONFIG)['passed']


def test_static_gain_is_not_dynamic_fault_case():
    assert dynamic_audit(trace(), {'wheels': {'scale': .95}}) is None
