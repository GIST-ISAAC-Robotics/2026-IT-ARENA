import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_contact_model import rotate, friction_components, read_contacts, impulse, planar_com_velocity, exact_sample, validate_frames


def message(t=1, side=1, force=None):
    c = {'collision1': {'name': 'arena_car::rear_left_wheel::collision'},
         'collision2': {'name': 'floor'}, 'position': [{'z': .003}],
         'normal': [{'z': 1}], 'wrench': [{'body1Wrench': {'force': force or {'x': -.9, 'z': 1}}}]}
    if side == 2:
        c['collision1'], c['collision2'] = c['collision2'], c['collision1']
        c['normal'] = [{'z': -1}]
        c['wrench'] = [{'body2Wrench': {'force': force or {'x': -.9, 'z': 1}}}]
    return {'header': {'stamp': {'nsec': t}}, 'contact': [c]}


def test_axis_and_speed_sign():
    assert rotate([0, 0, 1], [-math.pi/2, 0, 0]) == pytest.approx([0, 1, 0])
    assert rotate([1, 0, 0], [0, 0, math.pi/2]) == pytest.approx([0, 1, 0])


def test_pyramid_not_circular_cone():
    x = friction_components([.9, .9, 1], [0, 0, 1], [0, 1, 0], .9)
    assert x['pyramid_bound_ratio'] == pytest.approx(1)
    x = friction_components([1.1, 0, 1], [0, 0, 1], [0, 1, 0], .9)
    assert x['pyramid_bound_ratio'] > 1.2


def test_rotated_friction_has_same_components():
    values = ([1, .2, 3], [0, 0, 1], [0, 1, 0])
    a = friction_components(*values, .9)
    b = friction_components(*(rotate(v, [.1, -.2, .7]) for v in values), .9)
    assert a == pytest.approx(b)


def test_side_two_uses_opposite_normal_but_own_force(tmp_path):
    f = tmp_path/'raw.jsonl'
    f.write_text(json.dumps(message(side=2)))
    rows, duplicate = read_contacts(f)
    p = rows['rear_left_wheel'][1][0]
    assert p['normal'] == [0, 0, 1] and p['force'] == [-.9, 0, 1]
    assert duplicate == 0


def test_identical_dedup_conflict_and_missing_force(tmp_path):
    f = tmp_path/'raw.jsonl'
    a = message()
    f.write_text(json.dumps(a)+'\n'+json.dumps(a))
    _, n = read_contacts(f)
    assert n == 1
    f.write_text(json.dumps(a)+'\n'+json.dumps(message(force={'z': 2})))
    with pytest.raises(ValueError, match='conflicting'):
        read_contacts(f)
    a['contact'][0]['wrench'][0]['body1Wrench'] = {}
    f.write_text(json.dumps(a))
    with pytest.raises(ValueError, match='missing force'):
        read_contacts(f)


def test_impulse_right_endpoint_and_missing_not_zero():
    records = {t: [{'force': [1, 2, 3]}] for t in (0, 1000000, 2000000)}
    assert impulse(records, 0, 2000000) == pytest.approx([.002, .004, .006])
    del records[1000000]
    with pytest.raises(ValueError, match='coverage gap'):
        impulse(records, 0, 2000000)


def test_com_velocity_includes_rotating_reference_offset():
    r = {'truth_yaw_rad': 0, 'truth_longitudinal_mps': 1, 'truth_lateral_mps': 0, 'truth_yaw_rate_rad_s': 2}
    assert planar_com_velocity(r, [.1, .2, 0]) == pytest.approx([.6, .2, 0])


def test_bad_vector_rejected(tmp_path):
    f = tmp_path/'raw.jsonl'
    f.write_text(json.dumps(message(force={'z': float('nan')})))
    with pytest.raises(ValueError, match='nonfinite'):
        read_contacts(f)


def test_exact_sample_refuses_nearest_contact_and_mixed_time_moment():
    row = dict(sim_time_s=1., truth_roll_rad=0., truth_pitch_rad=0., truth_yaw_rad=0.,
               truth_longitudinal_mps=.5, wheel_surface_speed_mps=.1, motor_torque_nm=-.01,
               left_speed_rad_s=3., right_speed_rad_s=3., left_torque_nm=-.075, right_torque_nm=-.075)
    params = {'drive': {'wheel_radius': .0325}, 'wheels': {}}
    contacts = {}
    for wheel in ('rear_left_wheel', 'rear_right_wheel'):
        params['wheels'][wheel] = dict(axis_model=[0, 1, 0], mu=.9, slip_compliance=.02, normal_reference_n=4.905)
        contacts[wheel] = {10**9: [{'force': [-1, 0, 2], 'normal': [0, 0, 1], 'position': [999, 999, 999]}]}
    result = exact_sample(row, contacts, params)
    for record in result['rear'].values():
        assert 'contact_torque_nm' not in record and 'approx_net_spin_torque_nm' not in record
    contacts['rear_right_wheel'][10**9-1000000] = contacts['rear_right_wheel'].pop(10**9)
    with pytest.raises(ValueError, match='exact same-step'):
        exact_sample(row, contacts, params)


def test_physical_frame_guard_does_not_reject_unrelated_named_frame():
    m = ET.fromstring('<model><frame name="base_link"><pose relative_to="__model__">0 0 0 0 0 0</pose></frame>'
                      '<link name="chassis"><pose relative_to="__model__">0 0 1 0 0 0</pose></link></model>')
    validate_frames(m)
    m.find('link/pose').set('relative_to', 'other')
    with pytest.raises(ValueError, match='link frame'):
        validate_frames(m)
    m.find('link/pose').set('relative_to', '__model__')
    inertial = ET.SubElement(m.find('link'), 'inertial')
    ET.SubElement(inertial, 'pose', relative_to='__model__').text = '0 0 0 0 0 0'
    with pytest.raises(ValueError, match='inertia frame'):
        validate_frames(m)
