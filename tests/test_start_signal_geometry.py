import math
import numpy as np
import pytest
from audit_start_signal_geometry import rotation, project, lamps_from_sdf, diagnostic_transform

CAMERA = dict(xyz_m=[0, 0, 0], rpy_rad=[0, 0, 0], width_px=1000, height_px=500, horizontal_fov_deg=90)
SLOT = dict(index=0, x=0., y=0., yaw_rad=0.)


def lamps():
    return [dict(name=str(i), xyz=[1., .2-i*.1, 0.], radius=.005) for i in range(5)]


def test_ros_forward_left_up_projection_and_diameter():
    result = project(lamps(), SLOT, CAMERA)
    assert result['all_inside_image']
    assert result['red_to_green_left_to_right']
    assert result['lamps'][0]['center_uv'] == pytest.approx([400, 250])
    assert result['lamps'][0]['diameter_approx_px'] == pytest.approx(5)
    assert result['lamp_axis_abs_dot_camera_forward'] == 0.


def test_high_lamps_outside_is_not_detection_failure():
    result = project(diagnostic_transform(lamps(), z=1.), SLOT, CAMERA)
    assert not result['all_inside_image']
    assert all(r['center_uv'][1] < 0 for r in result['lamps'])


def test_projection_invariant_to_world_yaw_and_translation():
    world_r = rotation(0., 0., math.pi/2)
    moved = [dict(l, xyz=(world_r@l['xyz']+[4., 5., 0.]).tolist()) for l in lamps()]
    result = project(moved, dict(SLOT, x=4., y=5., yaw_rad=math.pi/2), CAMERA)
    original = project(lamps(), SLOT, CAMERA)
    np.testing.assert_allclose([r['center_uv'] for r in result['lamps']],
                               [r['center_uv'] for r in original['lamps']])


def test_behind_camera_is_unknown_not_valid_pixel():
    result = project(lamps(), dict(SLOT, yaw_rad=math.pi), CAMERA)
    assert not result['all_inside_image']
    assert all(r['center_uv'] is None for r in result['lamps'])


def test_named_frame_is_not_silently_accepted():
    sdf = '<sdf><world><model name="it_arena_track_static"><pose relative_to="other">0 0 0 0 0 0</pose></model></world></sdf>'
    with pytest.raises(ValueError, match='unsupported'):
        lamps_from_sdf(sdf)


def test_ninety_degree_sign_changes_red_green_order():
    along = [dict(name=str(i), xyz=[1.+i*.03, 0., .15], radius=.0075) for i in range(5)]
    plus = project(diagnostic_transform(along, yaw_delta=math.pi/2), SLOT, CAMERA)
    minus = project(diagnostic_transform(along, yaw_delta=-math.pi/2), SLOT, CAMERA)
    assert not plus['red_to_green_left_to_right']
    assert minus['red_to_green_left_to_right']
