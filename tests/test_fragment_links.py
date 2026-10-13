from dataclasses import replace
import math

import pytest

from arena_autonomy.fragment_links import LinkBounds, candidate_links
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory


def scene(stamp=1.,ranges=(1.,1.,1.),epoch=0):
    return scene_from_scan(ranges,angle_min=0.,angle_increment=.01,range_min=.05,range_max=12.,
        acquired_at_s=[stamp+i*.0002 for i in range(len(ranges))],t_ref_s=stamp+.01,
        sensor_xy_m=(0,0),source_epoch=epoch)


def history(v=0.,omega=0.):
    h=MotionHistory(MotionConfig(wheel_radius_m=1.,rear_axle_x_m=0.,lidar_x_m=0.))
    for i in range(141):
        h.add_mean_wheel(i*.01,v)
        h.add_gyro(i*.01,omega)
    return h


def bounds(**kwargs):
    values=dict(object_diameter_m=.25,speed_bound_mps=1.,range_error_m=.03,ego_transport_error_m=.03)
    values.update(kwargs)
    return LinkBounds(**values)


def test_single_link_is_not_identity_or_stationary_certificate():
    a,b=scene(),scene(1.1)
    result=candidate_links(a,b,history(),bounds())
    assert result.evaluated and len(result.links)==1
    assert not any((result.identity_certified,result.stationary_certified,result.driving_authorized,result.free_area_certified))
    assert result.previous_source_first_s==1. and result.current_source_first_s==1.1
    assert a.rays[0].acquired_at_s==1.  # 이송 기준 시각으로 덮어쓰지 않는다.


def test_split_and_merge_keep_all_edges_no_nearest_identity_choice():
    a=scene(ranges=(1.,1.,math.inf,1.,1.))
    b=scene(1.1,ranges=(1.,1.,1.,1.,1.))
    result=candidate_links(a,b,history(),bounds())
    assert len(result.links)==2 and result.ambiguous_current==(0,)
    merged=scene(ranges=(1.,1.,1.,1.,1.))
    split=scene(1.1,ranges=(1.,1.,math.inf,1.,1.))
    assert candidate_links(merged,split,history(),bounds()).ambiguous_previous==(0,)


def test_ego_translation_is_removed_before_comparing_support():
    a,b=scene(ranges=(2.,)),scene(1.1,ranges=(1.9,))
    strict=bounds(object_diameter_m=.02,speed_bound_mps=0.,range_error_m=0.,ego_transport_error_m=0.)
    result=candidate_links(a,b,history(1.),strict)
    assert result.evaluated and len(result.links)==1
    assert result.links[0].witness_distance_m<1e-10
    assert candidate_links(a,b,history(),strict).unmatched_current==(0,)


@pytest.mark.parametrize('which,reason',[
    ('epoch','source_epoch_or_frame_changed'),('ref','reference_did_not_advance'),
    ('reused','no_new_source_observation'),('age','source_age_exceeded'),
    ('missing','no_return_support')])
def test_invalid_association_is_explicit_not_empty_space(which,reason):
    a,b=scene(),scene(1.1)
    if which=='epoch': b=scene(1.1,epoch=1)
    elif which=='ref': b=replace(b,t_ref_s=a.t_ref_s) if b.source_last_s<=a.t_ref_s else a
    elif which=='reused': b=replace(a,t_ref_s=1.11)
    elif which=='age': b=scene(1.31)
    elif which=='missing': b=scene(1.1,ranges=(math.inf,)*3)
    result=candidate_links(a,b,history(),bounds())
    assert not result.evaluated and result.reason==reason and not result.free_area_certified


def test_motion_history_failure_no_uncompensated_fallback():
    result=candidate_links(scene(),scene(1.1),MotionHistory(),bounds())
    assert not result.evaluated and result.reason.startswith('ego_motion_unavailable:')


def test_resource_limit_no_downsampling_or_silent_removal():
    result=candidate_links(scene(),scene(1.1),history(),bounds(max_returned_points=2))
    assert not result.evaluated and result.reason=='association_capacity_exceeded'
    assert not result.free_area_certified


def test_far_support_unmatched_not_removed_from_source():
    old,new=scene(ranges=(1.,)),scene(1.1,ranges=(3.,))
    result=candidate_links(old,new,history(),bounds())
    assert result.evaluated and not result.links
    assert result.unmatched_previous==result.unmatched_current==(0,)
    assert old.rays[0].hit_xy_m==(1.,0.) and not result.free_area_certified


def test_ego_rotation_is_removed_and_more_uncertainty_never_removes_edges():
    old=scene(ranges=(1.,))
    new=scene_from_scan([1.],angle_min=-.1,angle_increment=.01,range_min=.05,range_max=12.,
        acquired_at_s=[1.1],t_ref_s=1.11,sensor_xy_m=(0,0))
    strict=bounds(object_diameter_m=.001,speed_bound_mps=0.,range_error_m=0.,ego_transport_error_m=0.)
    result=candidate_links(old,new,history(omega=1.),strict)
    assert len(result.links)==1 and result.links[0].witness_distance_m<1e-10
    loose=candidate_links(old,new,history(omega=1.),bounds())
    assert {(e.previous_fragment,e.current_fragment) for e in result.links}.issubset(
        {(e.previous_fragment,e.current_fragment) for e in loose.links})


@pytest.mark.parametrize('kwargs',[{'object_diameter_m':0},{'speed_bound_mps':-1},
    {'range_error_m':True},{'ego_transport_error_m':float('nan')},{'max_source_age_s':.31},
    {'max_returned_points':99999},{'max_fragments':0}])
def test_invalid_bounds(kwargs):
    with pytest.raises(ValueError): bounds(**kwargs)
