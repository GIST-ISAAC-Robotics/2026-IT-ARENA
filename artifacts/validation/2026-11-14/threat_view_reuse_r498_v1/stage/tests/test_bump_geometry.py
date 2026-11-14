import math
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from analyze_bump_geometry import Geometry, WheelEnvelope, axle_pose, clearance_at_pose, forward_floor_ray, profile, sweep


def test_profile_edges_peak_and_outside():
    g = Geometry(.05)
    assert profile(np.array([-.01, 0, .025, .05, .06]), g) == pytest.approx([0, 0, .01, 0, 0])


@pytest.mark.parametrize('length', [.05, .1, .15, .2])
def test_support_brackets_independent_dense_samples_and_symmetry(length):
    g = Geometry(length)
    env = WheelEnvelope(g, .001)
    x = np.linspace(-.02, length+.02, 39)
    q = np.linspace(0, length, 20001)
    dx = q[None, :]-x[:, None]
    cap = profile(q, g)+np.sqrt(np.maximum(0, g.radius_m**2-dx**2))
    dense = np.maximum(g.radius_m, np.where(abs(dx) <= g.radius_m, cap, -np.inf).max(axis=1))
    lower, upper = env.heights(x), env.heights(x, upper=True)
    assert (lower <= dense+1e-9).all()
    assert (dense <= upper+1e-12).all()
    assert lower == pytest.approx(env.heights(length-x), abs=1e-12)
    assert env.heights([length/2])[0] == pytest.approx(g.radius_m+g.height_m)


def test_flat_exact_pose_and_zero_required_clearance():
    g = Geometry(.1, height_m=0.)
    env = WheelEnvelope(g, .001)
    x = np.linspace(-.2, .3, 51)
    rz, fx, fz, pitch = axle_pose(env, x)
    assert rz == pytest.approx([g.radius_m]*len(x))
    assert fz == pytest.approx(rz)
    assert fx-x == pytest.approx([g.wheelbase_m]*len(x))
    assert pitch == pytest.approx(np.zeros(len(x)))
    assert clearance_at_pose(env, 0., g.radius_m, 0.) == pytest.approx((0., 0.))


def test_bump_between_grounded_axles_needs_height_not_zero():
    g = Geometry(.05)
    env = WheelEnvelope(g, .0001)
    rear = (g.length_m-g.wheelbase_m)/2
    rz, fx, fz, pitch = axle_pose(env, [rear])
    assert rz[0] == pytest.approx(g.radius_m)
    assert fz[0] == pytest.approx(g.radius_m)
    required, bound = clearance_at_pose(env, rear, rz[0], pitch[0])
    assert required == pytest.approx(g.height_m)
    assert 0 < bound < .00004


@pytest.mark.parametrize('length', [.05, .1, .15, .2])
def test_fixed_wheelbase_not_horizontal_separation(length):
    g = Geometry(length)
    env = WheelEnvelope(g, .0004)
    x = np.linspace(-g.wheelbase_m, g.length_m, 70)
    rz, fx, fz, pitch = axle_pose(env, x)
    assert np.hypot(fx-x, fz-rz) == pytest.approx(np.full(len(x), g.wheelbase_m), abs=1e-10)
    assert np.max(abs(pitch)) > 0
    assert np.min(fx-x) < g.wheelbase_m-.0001
    assert np.max(abs(pitch)) <= math.asin(g.height_m/g.wheelbase_m)+1e-12


def test_clearance_body_projection_matches_independent_dense_world_line():
    g = Geometry(.15)
    env = WheelEnvelope(g, .0005)
    for rear in [-.13, -.06, 0., .07, .15]:
        rz, _, _, pitch = axle_pose(env, [rear])
        theta, z = pitch[0], rz[0]
        estimate, bound = clearance_at_pose(env, rear, z, theta)
        co, si = math.cos(theta), math.sin(theta)
        q = np.linspace(rear-.1, rear+.25, 50001)
        h = profile(q, g)
        u = co*(q-rear)+si*(h-z)
        v = -si*(q-rear)+co*(h-z)
        overhang = (g.body_length_m-g.wheelbase_m)/2
        valid = (u >= -overhang) & (u <= g.wheelbase_m+overhang)
        dense = max(0., g.radius_m+float(v[valid].max()))
        assert estimate <= dense+1e-5  # independent grid can miss the exact segment boundary
        assert dense <= estimate+bound+1e-8


@pytest.mark.parametrize('change', [dict(radius_m=0.), dict(height_m=.033), dict(length_m=float('nan')),
                                   dict(wheelbase_m=.01), dict(body_length_m=.1), dict(length_m=True)])
def test_invalid_geometry_rejected(change):
    fields = dict(length_m=.05)
    fields.update(change)
    with pytest.raises(ValueError):
        Geometry(**fields).validate()


def test_study_contract_no_dynamics_or_safe_speed():
    result = sweep(Geometry(.05), .001, .005)
    assert result['sampled_required_clearance_m'] == pytest.approx(.01, abs=.0002)
    assert result['axle_distance_residual_max_m'] < 1e-10
    assert not result['continuous_travel_certified']
    assert not result['dynamic_contact_verified']
    assert not result['vehicle_pass_verified']
    assert result['safe_speed_mps'] is None
    assert json.loads(json.dumps(result, allow_nan=False)) == result


def test_selected_dimensions_and_mount_match_current_source():
    import yaml
    root = Path(__file__).resolve().parents[1]
    vehicle = yaml.safe_load((root/'src/arena_description/config/vehicle.yaml').read_text())['vehicle']
    mount = yaml.safe_load((root/'src/arena_description/config/b0183_c1.yaml').read_text())['lidar']
    g = Geometry(.05)
    assert g.radius_m == vehicle['drivetrain']['wheel_radius_m']
    assert g.wheelbase_m == vehicle['drivetrain']['wheelbase_m']
    assert g.body_length_m == vehicle['body']['length_m']
    assert mount['xyz_m'] == [.060, 0., .095]


@pytest.mark.parametrize('pitch', [0., .05])
def test_forward_ray_level_or_nose_up_does_not_hit_flat_floor(pitch):
    witness = forward_floor_ray(Geometry(.05), -.1, .0325, pitch)
    assert witness['forward_flat_floor_slant_distance_m'] is None
    assert not witness['floor_hit_downstream_of_bump']
    assert not witness['lidar_return_verified']


def test_forward_down_ray_intersection_and_local_rotation():
    g = Geometry(.05)
    w = forward_floor_ray(g, .1, .0425, -.07)
    distance = w['forward_flat_floor_slant_distance_m']
    assert distance > 0
    assert w['sensor_z_m']+distance*math.sin(-.07) == pytest.approx(0.)
    assert w['floor_x_m']-w['sensor_x_m'] == pytest.approx(distance*math.cos(.07))
    assert w['floor_hit_downstream_of_bump']
    assert not w['beam_visibility_verified']


def test_serialization_failure_does_not_create_partial_report(tmp_path, monkeypatch):
    import analyze_bump_geometry as module
    target = tmp_path/'report.json'
    monkeypatch.setattr(sys, 'argv', ['study', '--output', str(target)])
    monkeypatch.setattr(module, 'study', lambda: {'unsupported': object()})
    with pytest.raises(TypeError):
        module.main()
    assert not target.exists()


def test_cli_output_roundtrip_and_no_overwrite(tmp_path, monkeypatch):
    import analyze_bump_geometry as module
    result = sweep(Geometry(.05), .001, .005)
    monkeypatch.setattr(module, 'study', lambda: result)
    target = tmp_path/'report.json'
    monkeypatch.setattr(sys, 'argv', ['study', '--output', str(target)])
    module.main()
    saved = target.read_bytes()
    assert json.loads(saved)['maximum_nose_down_floor_witness']['floor_hit_downstream_of_bump'] is True
    with pytest.raises(SystemExit):
        module.main()
    assert target.read_bytes() == saved
