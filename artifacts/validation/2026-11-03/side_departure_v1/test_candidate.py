import importlib.util
from pathlib import Path

import numpy as np
import pytest
from arena_autonomy.local_path import swept_limit

spec = importlib.util.spec_from_file_location('isolated_side_candidate', Path(__file__).with_name('candidate.py'))
candidate = importlib.util.module_from_spec(spec)
spec.loader.exec_module(candidate)


def compare(points, steering=0.):
    pts = np.array(points, dtype=float)
    return candidate.candidate_limit(swept_limit, pts, steering, .135, 2.5, np.full(len(pts), .11), .20, .15)


def test_straight_motion_can_leave_current_uncertainty_overlap():
    assert swept_limit(np.array([[0., -.16]]), 0., .135, 2.5, .11, .2, .15) == 0.
    assert compare([[0., -.16]]) == pytest.approx(.4)


@pytest.mark.parametrize('points,steer', [([[.15, 0.]], 0.), ([[0., -.12]], 0.),
                                         ([[.18, -.16]], 0.), ([[0., -.16]], -.15)])
def test_front_close_side_corner_and_toward_side_remain_blocked(points, steer):
    assert compare(points, steer) == 0.


def test_another_front_object_is_not_cleared_with_side_points():
    result = compare([[0., -.16], [.30, 0.]])
    assert 0. < result < .2
