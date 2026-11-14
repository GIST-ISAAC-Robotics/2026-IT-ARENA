from dataclasses import FrozenInstanceError, replace
import math

import pytest

from arena_autonomy.local_scene import LocalScene, SensorRay, scene_from_scan
from arena_autonomy.observed_fragments import FragmentConfig, partition_observed


def build(values,**kwargs):
    options=dict(angle_min=0.,angle_increment=.01,range_min=.05,range_max=12.,
                 acquired_at_s=[1.+i*.0002 for i in range(len(values))],t_ref_s=1.1,
                 sensor_xy_m=(0.,0.))
    options.update(kwargs)
    return scene_from_scan(values,**options)


def indices(partition):
    return [[ray.source_index for ray in f.rays] for f in partition.fragments]


def test_unknown_does_not_bridge_or_drop_singleton():
    data=build([1.,1.,math.inf,1.])
    result=partition_observed(data)
    assert indices(result)==[[0,1],[3]]
    assert result.unknown_indices==(2,)
    assert not result.driving_authorized and not result.free_area_certified
    assert all(not f.complete_object_extent and f.vehicle_classification=='unclassified' for f in result.fragments)


def test_spatial_jump_and_acquisition_gap_are_separate_breaks():
    assert indices(partition_observed(build([1.,2.])))==[[0],[1]]
    assert indices(partition_observed(build([1.,1.],acquired_at_s=[1.,1.01])))==[[0],[1]]


def test_mask_and_scan_wrap_are_not_filled():
    data=build([1.]*500,angle_min=-math.pi,angle_increment=math.tau/500)
    result=partition_observed(data)
    assert result.masked_indices
    assert all(ray.status=='return' for f in result.fragments for ray in f.rays)
    assert 0 not in [i for group in indices(result) for i in group]


def test_stale_points_and_acquisition_times_are_preserved():
    data=build([1.,1.],t_ref_s=1.4)
    result=partition_observed(data)
    assert indices(result)==[[0,1]]
    assert not result.fragments[0].fresh
    assert result.fragments[0].source_first_s==1.
    assert result.fragments[0].source_last_s==1.0002
    assert result.fragments[0].rays==data.rays
    with pytest.raises(FrozenInstanceError): result.fragments[0].fresh=True


def test_noncontiguous_source_indices_do_not_merge():
    data=build([1.,1.])
    rays=(data.rays[0],replace(data.rays[1],source_index=3))
    changed=LocalScene(data.t_ref_s,data.source_first_s,data.source_last_s,'laser_frame',0,rays)
    assert indices(partition_observed(changed))==[[0],[3]]


def test_observed_bounds_are_only_returned_support_not_inferred_full_box():
    result=partition_observed(build([1.,1.,1.]))
    fragment=result.fragments[0]
    assert fragment.observed_bounds_xy_m==(min(r.hit_xy_m[0] for r in fragment.rays),
        min(r.hit_xy_m[1] for r in fragment.rays),max(r.hit_xy_m[0] for r in fragment.rays),
        max(r.hit_xy_m[1] for r in fragment.rays))
    assert not fragment.complete_object_extent


def test_invalid_ray_rejects_instead_of_silently_removing_endpoint():
    ray=SensorRay((0,0),(1,.1),1.,'return','laser_frame',0,0,(1.,0.))
    with pytest.raises(ValueError):
        partition_observed(LocalScene(1.1,1.,1.,'laser_frame',0,(ray,)))


def test_empty_keeps_absent_source_time():
    result=partition_observed(build([]))
    assert result.fragments==() and result.source_first_s is None


@pytest.mark.parametrize('kwargs',[{'max_age_s':.31},{'base_gap_m':float('nan')},
                                  {'range_gap_scale':True},{'max_acquisition_gap_s':0}])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError): FragmentConfig(**kwargs)
