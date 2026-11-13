import copy
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from analyze_scheduled_surface import (FLAT_MODEL, surface_audit, zero_application,
    collect_contacts, contact_summary, comparison_checks, frozen_sources_match)


def test_surface_exclusion_does_not_hide_other_changes(tmp_path):
    official, flat = tmp_path/'official.sdf', tmp_path/'flat.sdf'
    track = '<model name="track"><link name="road">{}</link></model>'
    floor = '<collision name="surface_1"/><collision name="grass_1"/>'
    wall = '<collision name="wall_1"><geometry><box><size>1 1 1</size></box></geometry></collision>'
    def world(path, collision, support=''):
        path.write_text('<sdf><world>'+track.format(collision)+support+'</world></sdf>')
    world(official, floor+wall)
    world(flat, wall, FLAT_MODEL)
    assert surface_audit(official, flat)['passed']
    world(flat, wall.replace('1 1 1', '2 1 1'), FLAT_MODEL)
    assert not surface_audit(official, flat)['passed']
    world(flat, wall, FLAT_MODEL.replace('-0.047', '-0.05'))
    assert not surface_audit(official, flat)['passed']
    world(flat, wall, FLAT_MODEL.replace('</collision>', '<surface><friction/></surface></collision>'))
    assert not surface_audit(official, flat)['passed']
    world(flat, wall, FLAT_MODEL+FLAT_MODEL)
    assert not surface_audit(official, flat)['passed']
    world(flat, floor+wall, FLAT_MODEL)
    assert not surface_audit(official, flat)['passed']


def event(t, speed):
    return dict(type='applied', seq=t, stamp_us=t-20, apply_us=t, sim_us=t, speed=speed, steering=0.)


def test_final_zero_is_actual_ack_not_first_initial_zero_or_stop():
    events = [event(0, 0), event(100, 1), event(200, 0), event(250, .3), event(300, 0),
              {'type': 'stopped', 'sim_us': 400}]
    assert zero_application(events, 0, 400)['sim_us'] == 300
    with pytest.raises(ValueError):
        zero_application(events, 0, 100)
    with pytest.raises(ValueError):
        zero_application([event(0, 0)], 0, 400)


def test_missing_contact_is_not_zero_force_and_timestamp_conflicts_ambiguous():
    record = {'t': 1., 'wheel': 'rear_left_wheel', 'force_world_xyz_n': [0, 0, 2.],
              'force_available': True, 'points': 1, 'other_collisions': ['floor']}
    empty = contact_summary([], 1., 2., 1.5)
    assert empty['at_peak'] is None and empty['fz_min_p05_median_p95_max_n'] is None
    old = contact_summary([record], 1., 2., 1.02)
    assert old['at_peak'] is None  # 6 ms보다 오래된 표본
    close = contact_summary([record], 1., 2., 1.005)
    assert close['at_peak']['force_world_xyz_n'][2] == 2.
    assert close['peak_time_offset_s'] == pytest.approx(-.005)
    missing = contact_summary([{**record, 'force_available': False}], 1., 2., 1.)
    assert missing['fz_min_p05_median_p95_max_n'] is None
    conflict = contact_summary([record, {**record, 'points': 2}], 1., 2., 1.)
    assert conflict['conflicting_timestamp_records'] == 1 and conflict['at_peak_ambiguous']
    assert conflict['at_peak'] is None
    exclusive = contact_summary([record], 0., 1., 1., end_exclusive=True)
    assert exclusive['observed_records'] == 0


def test_contact_dedup_only_removes_identical_rows():
    message = {'header': {'stamp': {'sec': 1}}, 'contact': [{
        'collision1': {'name': 'arena_car::rear_left_wheel::collision'},
        'collision2': {'name': 'floor'}, 'position': [{}],
        'wrench': [{'body1Wrench': {'force': {'z': 2.}}}]}]}
    changed = copy.deepcopy(message)
    changed['contact'][0]['wrench'][0]['body1Wrench']['force']['z'] = 3.
    records = collect_contacts([message, message, changed])['rear_left_wheel']
    assert len(records) == 2


def test_comparison_requires_valid_exact_application_and_same_sources():
    summary = {'passed': True, 'scheduled': True, 'start_ns': 1, 'duration_s': 3.5,
               'trace': {'trace': {'spawn': {'iteration': 1}}, 'step_s': .001},
               'evidence': {k: 'same' for k in ('source_sha256', 'runtime_vehicle_sha256',
                    'meshes_sha256', 'profile', 'spawn_pose')}}
    ack = {'same_all_normal_applied_envelopes': True}
    shape = {'passed': True}
    assert all(comparison_checks(summary, summary, ack, shape).values())
    for key in ('passed', 'scheduled'):
        other = {**summary, key: False}
        assert not all(comparison_checks(summary, other, ack, shape).values())
    assert not all(comparison_checks(summary, summary, {**ack, 'same_all_normal_applied_envelopes': False}, shape).values())
    other = copy.deepcopy(summary)
    other['evidence']['source_sha256'] = 'changed'
    assert not all(comparison_checks(summary, other, ack, shape).values())
    other = copy.deepcopy(summary)
    other['trace']['step_s'] = .00025
    assert not all(comparison_checks(summary, other, ack, shape).values())


def test_source_registration_is_subset_but_full_reference_must_match():
    full = {'runtime': 'abc', 'evaluator': 'def', 'world': 'ghi'}
    registered = {'runtime': 'abc', 'registration_script': 'jkl'}
    assert frozen_sources_match(full, registered, full)
    assert not frozen_sources_match({'runtime': 'abc'}, registered, full)
    assert not frozen_sources_match({**full, 'world': 'changed'}, registered, full)
    assert not frozen_sources_match({**full, 'unknown': 'x'}, registered, full)
    assert not frozen_sources_match(full, {**registered, 'world': 'conflict'}, full)
    assert not frozen_sources_match({}, {}, {})
