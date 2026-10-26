"""미연결 입력 계약·순환 인접성의 원인별 검사."""
from dataclasses import replace
import math
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(HERE)]
import numpy as np
import pytest
from arena_autonomy.observation_support import depth_support
from scan_cycle_candidate import scene_with_cycle,cycle_depth_support,adjacent_sources,cycle_both_returned
from arena_autonomy.m3_execution_trial import _both_sides_returned


def fixture(n=500,*,increment=None,stamps=None,mask=0.,yaw=0.,moving=False,values=None):
    inc=math.tau/n if increment is None else increment
    times=[1.+i*.0002 for i in range(n)] if stamps is None else stamps
    transforms=[(-.02+.02*i/max(1,n-1),.001*math.sin(i),.08*i/max(1,n-1)) for i in range(n)] if moving else None
    return scene_with_cycle([4.]*n if values is None else values,angle_min=-math.pi,
        angle_increment=inc,range_min=.05,range_max=12.,acquired_at_s=times,t_ref_s=max(1.2,times[-1]+.1),
        rear_half_angle_deg=mask,sensor_yaw_rad=yaw,acquisition_to_ref_se2=transforms)


def seam_query(yaw=0.):
    a=math.pi-math.pi/500+yaw
    return np.array([[.06+math.cos(a),math.sin(a)]])


def test_factory_preserves_raw_timing_and_full_turn():
    cycle=fixture()
    assert cycle.full_turn and cycle.rejection is None
    assert cycle.scene.rays[0].acquired_at_s==1.
    assert cycle.scene.rays[-1].acquired_at_s==1.0998
    assert cycle.scene.rays[0].source_index==0 and cycle.scene.rays[-1].source_index==499
    assert adjacent_sources(cycle.scene,np.array([0]),np.array([499]),cycle)[0]


@pytest.mark.parametrize('kind',['partial','missing_last','duplicate_endpoint','simultaneous'])
def test_no_cyclic_permission_without_required_source_contract(kind):
    kwargs={'increment':.01} if kind=='partial' else (
        {'n':499,'increment':math.tau/500} if kind=='missing_last' else (
        {'increment':math.tau/499} if kind=='duplicate_endpoint' else {'stamps':[1.]*500}))
    cycle=fixture(**kwargs)
    assert not cycle.full_turn and cycle.rejection
    assert not adjacent_sources(cycle.scene,np.array([0]),np.array([len(cycle.scene.rays)-1]),cycle)[0]


def test_certificate_cannot_transfer_to_equal_other_scene_or_reordered_indices():
    a,b=fixture(),fixture()
    assert not adjacent_sources(b.scene,np.array([0]),np.array([499]),a)[0]
    rays=list(a.scene.rays);rays[10]=replace(rays[10],source_index=900)
    modified=replace(a.scene,rays=tuple(rays))
    assert not adjacent_sources(modified,np.array([0]),np.array([499]),a)[0]


@pytest.mark.parametrize('yaw',[0.,.3,-.8])
def test_certified_endpoints_use_spatial_seam_but_keep_original_timestamps(yaw):
    c=fixture(yaw=yaw);q=seam_query(yaw);pad=np.zeros(500)
    assert not depth_support(c.scene,q,pad)[0]
    assert cycle_depth_support(c.scene,q,pad,cycle=c)[0]
    assert not cycle_depth_support(c.scene,q,pad)[0] # default unchanged
    assert c.scene.rays[-1].acquired_at_s-c.scene.rays[0].acquired_at_s==pytest.approx(.0998)


@pytest.mark.parametrize('invalid',['unknown_first','unknown_last','masked','near_return','older_ray_margin'])
def test_seam_never_fills_invalid_or_occupied_or_unobserved_space(invalid):
    values=[4.]*500;pad=np.zeros(500)
    if invalid=='unknown_first':values[0]=math.nan
    if invalid=='unknown_last':values[-1]=math.inf
    if invalid=='near_return':values[0]=.1
    if invalid=='older_ray_margin':pad[0]=4.
    c=fixture(mask=30. if invalid=='masked' else 0.,values=values)
    assert c.full_turn
    assert not cycle_depth_support(c.scene,seam_query(),pad,cycle=c)[0]


@pytest.mark.parametrize('moving',[False,True])
@pytest.mark.parametrize('seed',[1,27,91])
def test_reference_keeps_nonseam_results_with_motion_and_noise(moving,seed):
    rng=np.random.default_rng(seed)
    values=rng.uniform(.1,10.,500)
    values[[30,31,233]]=np.nan
    c=fixture(moving=moving,values=values.tolist())
    points=rng.uniform(-2.,2.,(500,2)); pad=rng.uniform(.02,.09,500)
    before=depth_support(c.scene,points,pad)
    after=cycle_depth_support(c.scene,points,pad,cycle=c)
    assert np.all(~before|after) # No previous support lost.
    for point in points[before!=after]:
        rays=c.scene.rays;origins=np.array([r.ray_origin_xy_m for r in rays]);d=np.array([r.direction_xy for r in rays])
        delta=point-origins;angles=np.arctan2(d[:,0]*delta[:,1]-d[:,1]*delta[:,0],np.sum(d*delta,axis=1))
        lo=np.argmin(np.where(angles>=-1e-10,np.maximum(angles,0.),np.inf))
        hi=np.argmin(np.where(angles<=1e-10,np.maximum(-angles,0.),np.inf))
        assert {int(lo),int(hi)}=={0,499}


def test_current_rear_mask_remains_unchanged():
    c=fixture(mask=30.,moving=True)
    rng=np.random.default_rng(33);points=rng.uniform(-1.,1.,(1000,2));pad=np.full(500,.08)
    assert np.array_equal(depth_support(c.scene,points,pad),cycle_depth_support(c.scene,points,pad,cycle=c))


def test_empty_queries_and_origin_are_not_fabricated_free():
    c=fixture();pad=np.zeros(500)
    assert not len(cycle_depth_support(c.scene,[],pad,cycle=c))
    assert not cycle_depth_support(c.scene,[[.06,0.]],pad,cycle=c)[0]


@pytest.mark.parametrize('padding',[[-1.]*500,[math.nan]*500,[0.]*499])
def test_invalid_new_candidate_padding_rejected(padding):
    c=fixture()
    with pytest.raises(ValueError):cycle_depth_support(c.scene,seam_query(),padding,cycle=c)


@pytest.mark.parametrize('mask',[0.,30.])
def test_return_kind_inspection_uses_same_cycle_proof(mask):
    c=fixture(mask=mask)
    assert bool(cycle_both_returned(c.scene,seam_query(),cycle=c)[0])==(mask==0.)
    assert np.array_equal(cycle_both_returned(c.scene,seam_query()),_both_sides_returned(c.scene,seam_query()))


@pytest.mark.parametrize('moving',[False,True])
def test_return_kind_changes_only_first_last_bracket(moving):
    c=fixture(moving=moving)
    rng=np.random.default_rng(99);points=rng.uniform(-1.,1.,(800,2))
    before=_both_sides_returned(c.scene,points)
    after=cycle_both_returned(c.scene,points,cycle=c)
    assert np.all(~before|after)
    assert not len(cycle_both_returned(c.scene,[],cycle=c))


def test_return_kind_unknown_still_rejected():
    values=[4.]*500;values[0]=math.inf;c=fixture(values=values)
    assert not cycle_both_returned(c.scene,seam_query(),cycle=c)[0]


@pytest.mark.parametrize('n',[180,500,513,720,1440,8192])
def test_float32_whole_turn_rounding_is_not_a_missing_ray_license(n):
    inc=float(np.float32(math.tau/n))
    whole=fixture(n=n,increment=inc)
    assert whole.full_turn
    assert adjacent_sources(whole.scene,np.array([0]),np.array([n-1]),whole)[0]
    missing=fixture(n=n-1,increment=inc)
    assert not missing.full_turn
    duplicated=fixture(n=n+1,increment=inc) if n<8192 else None
    if duplicated is not None: assert not duplicated.full_turn


@pytest.mark.parametrize('n',[180,500,720])
def test_float32_duplicate_endpoint_does_not_get_cyclic_permission(n):
    c=fixture(n=n,increment=float(np.float32(math.tau/(n-1))))
    assert not c.full_turn
