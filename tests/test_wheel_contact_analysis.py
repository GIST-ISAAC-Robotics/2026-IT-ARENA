"""접촉 순서/힘 누락을 하중 0으로 혼동하지 않는 평가기 검사."""
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_wheel_contacts import contact_rows, nearest


def sample():
    return {'header': {'stamp': {'sec': 5, 'nsec': 2000000}}, 'contact': [{
        'collision1': {'name': 'ground'}, 'collision2': {'name': 'arena_car::rear_left_wheel::collision'},
        'position': [{'z': .003}], 'wrench': [{'body1Wrench': {'force': {'z': -4.}},
                                            'body2Wrench': {'force': {'z': 4.}}}]}]}


def test_force_uses_wheel_side_not_always_body_one():
    row, = contact_rows(sample())
    assert row['force_world_xyz_n'] == [0., 0., 4.]
    assert row['force_available']
    assert row['t'] == pytest.approx(5.002)


def test_empty_contact_is_not_a_zero_load_observation():
    message = sample()
    message['contact'] = []
    assert contact_rows(message) == []


def test_missing_wrench_not_reported_as_measured_zero():
    message = sample()
    message['contact'][0]['wrench'] = []
    row, = contact_rows(message)
    assert not row['force_available']


def test_missing_force_not_reported_as_measured_zero():
    message = sample()
    message['contact'][0]['wrench'][0]['body2Wrench'] = {}
    assert not contact_rows(message)[0]['force_available']


def test_explicit_zero_force_is_a_valid_measurement():
    message = sample()
    message['contact'][0]['wrench'][0]['body2Wrench']['force'] = {}
    row, = contact_rows(message)
    assert row['force_available'] and row['force_world_xyz_n'] == [0., 0., 0.]


def test_multiple_contact_patches_are_summed():
    message = sample()
    message['contact'].append(message['contact'][0])
    row, = contact_rows(message)
    assert row['force_world_xyz_n'][2] == 8.
    assert row['points'] == 2


def test_nonfinite_force_rejected():
    message = sample()
    message['contact'][0]['wrench'][0]['body2Wrench']['force']['z'] = float('nan')
    with pytest.raises(ValueError):
        contact_rows(message)


def test_stale_nearest_measurement_is_missing():
    assert nearest([{'t': 1.}], 1.02) is None
    assert nearest([], 1.) is None
    assert nearest([{'t': 1.}, {'t': 1.01}], 1.012) == {'t': 1.01}
