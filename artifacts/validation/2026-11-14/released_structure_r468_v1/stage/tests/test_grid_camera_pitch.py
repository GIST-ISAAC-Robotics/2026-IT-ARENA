import math
import numpy as np
import pytest

from audit_grid_camera_pitch import vertical_interval, intersect, assess, relative_targets
from audit_start_signal_geometry import rotation, project

CAM = dict(xyz_m=[0., 0., 0.], rpy_rad=[0., 0., 0.], width_px=1000, height_px=500, horizontal_fov_deg=90.)
SLOT = dict(index=0, x=0., y=0., yaw_rad=0.)


def test_pitch_interval_matches_vertical_pinhole_bounds():
    points = np.array([[2., 0., 1.], [2., .1, 1.2]])
    result = vertical_interval(points, CAM)
    half = math.degrees(math.atan(.5))
    assert result['min_up_deg'] == pytest.approx(math.degrees(math.atan(.6))-half)
    assert result['max_up_deg'] == pytest.approx(math.degrees(math.atan(.5))+half)
    for up in (result['min_up_deg']+.01, result['max_up_deg']-.01):
        rotated = (rotation(0., -math.radians(up), 0.).T@points.T).T
        v = 250-500*rotated[:, 2]/rotated[:, 0]
        assert np.all(v >= 0) and np.all(v < 500)


def test_crop_can_be_impossible_when_full_image_fits():
    p = [[1., 0., 0.], [1., 0., 1.]]
    assert vertical_interval(p, CAM)['nonempty']
    assert not vertical_interval(p, CAM, .58)['nonempty']


def test_world_transform_invariance():
    lamps = [dict(name=str(i), xyz=[2., .2-i*.1, 1.], radius=.005) for i in range(5)]
    original = relative_targets(lamps, SLOT, CAM)
    r = rotation(0., 0., 1.)
    moved = [dict(l, xyz=(r@l['xyz']+[3., -2., 0.]).tolist()) for l in lamps]
    shifted = relative_targets(moved, dict(SLOT, x=3., y=-2., yaw_rad=1.), CAM)
    for a,b in zip(original, shifted):
        np.testing.assert_allclose(a,b, atol=1e-12)


def test_midpoint_uses_full_projection_and_not_crop_as_visibility():
    lamps = [dict(name=str(i), xyz=[2., .2-i*.1, 1.], radius=.005) for i in range(5)]
    result = assess(lamps, [SLOT], CAM)
    assert result['midpoint_trial']['full_image_count'] == 1
    assert result['vertical_fov_deg'] == pytest.approx(math.degrees(2*math.atan(.5)))


@pytest.mark.parametrize('points', [[], [[0.,0.,1.]], [[-1.,0.,1.]], [[1.,0.,float('nan')]], [[1.,0.]]])
def test_invalid_targets_rejected(points):
    with pytest.raises(ValueError):
        vertical_interval(points, CAM)


def test_intersection_and_nonhorizontal_mount_rejection():
    assert not intersect([dict(min_up_deg=2.,max_up_deg=3.),dict(min_up_deg=4.,max_up_deg=5.)])['nonempty']
    with pytest.raises(ValueError):
        assess([], [], dict(CAM, rpy_rad=[0., -.1, 0.]))
