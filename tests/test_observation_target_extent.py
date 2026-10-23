import math
from types import SimpleNamespace

import numpy as np
import pytest
from arena_autonomy.local_scene import LocalScene, SensorRay
from probe_observation_target_extent import finite_return_clearance, rectangle_distance, ego_clearance


def beam(origin=(0.,0.), direction=(1.,0.), stamp=.5, status='return', distance=2.):
    hit = tuple(np.array(origin)+np.array(direction)*distance) if status == 'return' else None
    ray = SensorRay(origin, hit, stamp, status, 'laser_frame', 0, 0, direction)
    return LocalScene(.6, stamp, stamp, 'laser_frame', 0, (ray,))


@pytest.mark.parametrize('point,expected', [((1.,.1),.1),((3.,0.),1.),((-.1,0.),.15),((.05,0.),0.)])
def test_segment_caps_and_perpendicular_distance(point, expected):
    row = finite_return_clearance(beam(), point, (0.,0.), range_error=0.)[0]
    assert row['clearance_m'] == pytest.approx(expected)


def test_raw_timestamp_changes_moving_target_position():
    # now .6에서 y=.2. t=.5에서 y=.1이므로 .1m, 시간을 now로 덮으면 .2m로 틀린다.
    assert finite_return_clearance(beam(), (1.,.2), (0.,1.))[0]['clearance_m'] == pytest.approx(.1)


@pytest.mark.parametrize('status', ['unknown','masked'])
def test_missing_rays_do_not_prove_free_or_blocked(status):
    assert finite_return_clearance(beam(status=status), (1.,0.), (0.,0.)) == []


def test_range_error_shortens_only_valid_prefix():
    assert finite_return_clearance(beam(), (2.,0.), (0.,0.), range_error=.03)[0]['clearance_m'] == pytest.approx(.03)
    assert finite_return_clearance(beam(distance=.06), (1.,0.), (0.,0.), range_error=.03) == []


def test_nonzero_origin_and_rotation():
    row = finite_return_clearance(beam((1.,2.),(0.,1.)), (1.2,3.), (0.,0.))[0]
    assert row['clearance_m'] == pytest.approx(.2)


def test_body_distance_is_not_center_distance():
    np.testing.assert_allclose(rectangle_distance([[0.,0.],[.1,.075],[.13,.115]]), [0.,0.,.05], atol=1e-12)


@pytest.mark.parametrize('yaw', [0., .7, -2.4])
def test_ego_reconstruction_uses_raw_beam_angle_and_mount(yaw):
    mount = SimpleNamespace(lidar_x_m=.06, lidar_y_m=.02, lidar_yaw_rad=.3)
    cfg = SimpleNamespace(length_m=.2, width_m=.15)
    raw_angle = -.8
    c, s = math.cos(yaw), math.sin(yaw)
    base = np.array([.4,.2])
    rotation = np.array([[c,-s],[s,c]])
    origin = base+rotation@np.array([.06,.02])
    angle = yaw+mount.lidar_yaw_rad+raw_angle
    scene = beam(tuple(origin), (math.cos(angle),math.sin(angle)))
    center = base+rotation@np.array([.13,.115])
    result = ego_clearance(scene, center, (0.,0.), mount, cfg,
                           dict(count=1,angle_min=raw_angle,angle_increment=.013))
    assert result == pytest.approx(min(.05, float(rectangle_distance(center))))


@pytest.mark.parametrize('center,velocity,error', [((math.nan,0.),(0.,0.),.03), ((0.,0.),(0.,math.inf),.03), ((0.,0.),(0.,0.),-.1)])
def test_invalid_inputs_reject(center, velocity, error):
    with pytest.raises(ValueError):
        finite_return_clearance(beam(), center, velocity, range_error=error)
