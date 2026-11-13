import copy
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from braking_command_profile import load_profile, audit_requests


def data():
    return dict(schema_version=1, interpolation='previous_sample', duration_s=3.,
                brake_window_s=[1., 2.], samples_t_speed_steering=[[0, 0, 0], [1, 1, .1], [2, 0, 0], [3, 0, 0]])


def load(tmp_path, value):
    path = tmp_path/'profile.json'
    path.write_text(json.dumps(value))
    return load_profile(path)


def test_fixed_profile_zoh_and_stop(tmp_path):
    profile = load(tmp_path, data())
    assert profile.at(-1) == (0., 0.)
    assert profile.at(.999) == (0., 0.)
    assert profile.at(1) == (1., .1)
    assert profile.at(1.999) == (1., .1)
    assert profile.at(2) == (0., 0.)
    assert profile.at(3) == profile.at(10) == (0., 0.)
    with pytest.raises(ValueError):
        profile.at(float('nan'))


@pytest.mark.parametrize('change', [
    {'schema_version': 2}, {'interpolation': 'linear'}, {'duration_s': 20.},
    {'brake_window_s': [2, 1]}, {'brake_window_s': [1, 4]},
    {'samples_t_speed_steering': [[0, 0, 0], [1, float('nan'), 0], [3, 0, 0]]},
    {'samples_t_speed_steering': [[0, 0, 0], [1, True, 0], [3, 0, 0]]},
    {'samples_t_speed_steering': [[0, 0, 0], [1, -.1, 0], [3, 0, 0]]},
    {'samples_t_speed_steering': [[0, 0, 0], [1, 1.1, 0], [3, 0, 0]]},
    {'samples_t_speed_steering': [[0, 0, 0], [1, 1, .4], [3, 0, 0]]},
    {'samples_t_speed_steering': [[0, 1, 0], [1, 1, 0], [3, 0, 0]]},
    {'samples_t_speed_steering': [[0, 0, 0], [1, 1, 0], [3, 1, 0]]},
    {'samples_t_speed_steering': [[0, 0, 0], [1, 1, 0], [1, 0, 0], [3, 0, 0]]},
])
def test_bad_profiles_rejected(tmp_path, change):
    with pytest.raises(ValueError):
        load(tmp_path, {**data(), **change})


def test_publish_audit_detects_loss_and_changed_request(tmp_path):
    profile = load(tmp_path, data())
    rows = [dict(sim_s=10.+i*.02, speed_mps=profile.at(i*.02)[0], steering_rad=profile.at(i*.02)[1]) for i in range(151)]
    assert audit_requests(rows, profile, 10., 13.)['passed']
    assert not audit_requests(rows[10:], profile, 10., 13.)['passed']
    assert not audit_requests(rows[:-10], profile, 10., 13.)['passed']
    assert not audit_requests(rows[:50]+rows[60:], profile, 10., 13.)['passed']
    changed = copy.deepcopy(rows)
    changed[60]['speed_mps'] = .3
    assert not audit_requests(changed, profile, 10., 13.)['passed']


def test_registered_profile_bounded_and_has_braking():
    profile = load_profile(ROOT/'artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json')
    assert max(row[1] for row in profile.samples) == 1.
    assert max(abs(row[2]) for row in profile.samples) > .01
    assert profile.at(2.5) == (0., 0.)
