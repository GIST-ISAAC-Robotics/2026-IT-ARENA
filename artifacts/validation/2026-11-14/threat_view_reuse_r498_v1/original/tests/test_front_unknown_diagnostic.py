import math
import numpy as np
import pytest
import xml.etree.ElementTree as ET

from diagnose_front_unknown import compose, geometry_summary, obj_triangles, pose, pose_window, raw_kind, triangle_distance


@pytest.mark.parametrize('value,kind', [(math.inf, 'positive_infinity'), (-math.inf, 'negative_infinity'),
    (math.nan, 'nan'), (.01, 'below_minimum'), (13., 'above_maximum'), (.5, 'finite_return')])
def test_unknown_categories_are_not_filled(value, kind):
    assert raw_kind(value, .05, 12.) == kind


def test_triangle_front_back_and_miss_are_separate():
    triangle = np.array([[[1., -1., -1.], [1., -1., 1.], [1., 1., 0.]]])
    assert triangle_distance([0., 0., 0.], [1., 0., 0.], triangle, True) == pytest.approx(1.)
    assert triangle_distance([2., 0., 0.], [-1., 0., 0.], triangle) == pytest.approx(1.)
    assert triangle_distance([2., 0., 0.], [-1., 0., 0.], triangle, True) is None
    assert triangle_distance([0., 2., 0.], [1., 0., 0.], triangle) is None
    assert triangle_distance([0., 0., 0.], [0., 1., 0.], triangle) is None


def test_brackets_cover_capture_interval_without_interpolation():
    rows = np.column_stack((np.arange(0., .02, .001), np.zeros((20, 3))))
    result = pose_window(rows, .01)
    assert result[:, 0] == pytest.approx([.007, .01])
    with pytest.raises(ValueError):
        pose_window(rows, .001)
    with pytest.raises(ValueError):
        pose_window(rows[::10], .01)


def test_model_and_link_pose_composition_and_rejection():
    assert compose([1., 2., 3., 0., 0., math.pi/2], [1., 0., 1., 0., 0., 0.])[:3] == pytest.approx([1., 3., 4.])
    with pytest.raises(ValueError):
        pose(ET.fromstring('<link><pose relative_to="other">0 0 0 0 0 0</pose></link>'))
    with pytest.raises(ValueError):
        pose(ET.fromstring('<link><pose>0 0 0 .1 0 0</pose></link>'))


def test_obj_transform_uses_saved_triangles(tmp_path):
    path = tmp_path/'triangle.obj'
    path.write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1//1 2//1 3//1\n')
    triangles = obj_triangles(path, [1., 2., 3., 0., 0., math.pi/2])
    assert np.allclose(triangles[0], [[1., 2., 3.], [1., 3., 3.], [0., 2., 3.]])


def test_missing_geometry_or_near_unknown_is_not_summarized_as_range_limit():
    values = [dict(raw_kind='positive_infinity', collision_distance_m=d,
                   visual_two_sided_m=d, visual_front_facing_m=d) for d in (12.1, 11., None)]
    result = geometry_summary([dict(range_max_m=12., casts=values, invalid=[{}, {}],
                                   unavailable=[dict(index=3)])])
    assert result['expected_missing_ray_pose_samples'] == 4
    assert result['unavailable_ray_windows'] == 1
    assert result['missing_ray_pose_samples'] == 3
    assert result['beyond_configured_range'] == 1
    assert result['box_visual_agree'] == 2
