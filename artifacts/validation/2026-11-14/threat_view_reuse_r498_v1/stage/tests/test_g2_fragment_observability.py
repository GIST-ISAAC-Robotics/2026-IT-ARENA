from dataclasses import replace
import math

import pytest

from arena_autonomy.local_scene import scene_from_scan
from g2_fragment_observability import apparent_velocities, perturb, summarize, world_point


def scene():
    return scene_from_scan([1.]*8,angle_min=-.04,angle_increment=.01,range_min=.05,
        range_max=12.,acquired_at_s=[1+i*.0002 for i in range(8)],t_ref_s=1.1,sensor_xy_m=(0,0))


def test_world_projection_rotation():
    assert world_point((1,0),(2,3,math.pi/2))==pytest.approx((2,4))


def test_truth_mixed_fragment_reported_not_silently_classified():
    report=summarize(scene(),['target']*4+['wall']*4,(0,0,0))
    assert report['target_fragment_count']==report['contaminated_fragments']==1
    assert report['target_returns']==4
    assert not report['driving_authorized']


def test_missing_ray_splits_without_false_merge():
    modified=perturb(scene(),hide_indices=[3])
    report=summarize(modified,['target']*8,(0,0,0))
    assert report['target_returns']==7 and report['target_fragment_count']==2
    assert modified.rays[3].status=='unknown'
    for old,new in zip(scene().rays,modified.rays):
        assert old.acquired_at_s==new.acquired_at_s and old.source_index==new.source_index


def test_bias_preserves_direction_origin_timestamp():
    original=scene()
    modified=perturb(original,.03)
    for old,new in zip(original.rays,modified.rays):
        assert math.dist(new.ray_origin_xy_m,new.hit_xy_m)==pytest.approx(1.03)
        assert old.direction_xy==new.direction_xy and old.acquired_at_s==new.acquired_at_s
        assert old.ray_origin_xy_m==new.ray_origin_xy_m


def test_invisible_object_is_none_not_stationary_estimate():
    report=summarize(scene(),['wall']*8,(0,0,0))
    assert report['oracle_visible_centroid_world'] is None and report['oracle_center_error_m'] is None
    assert apparent_velocities([report,report])==[]


def test_finite_difference_uses_acquisition_not_callback():
    a=dict(reference_s=2.,oracle_visible_centroid_world=(1.,0.),oracle_mean_acquisition_s=1.)
    b=dict(reference_s=3.,oracle_visible_centroid_world=(1.06,0.),oracle_mean_acquisition_s=1.1)
    assert apparent_velocities([a,b])[0]['speed_mps']==pytest.approx(.6)
    with pytest.raises(ValueError): apparent_velocities([a,a])


def test_bad_label_count_and_bad_bias_rejected():
    with pytest.raises(ValueError): summarize(scene(),[],(0,0,0))
    with pytest.raises(ValueError): perturb(scene(),float('nan'))
