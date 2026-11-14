"""좁은 자기 가림의 수용 위험과 실제 반환/일반 결측 거절을 구분한다."""
import math
import numpy as np
import pytest

from arena_autonomy.known_structure import current_mask_shadow, sector_rays
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.local_scene import scene_from_measurements

SECTORS=((-123.15,7.26),)


def example(change=None):
    angles=np.arange(512)*math.tau/512
    ranges=np.full(512,3.)
    ids=sector_rays(0,0,512,SECTORS,source_angles=angles)[0]
    ranges[ids]=.07
    if change:
        index,value=change
        if index=='left': index=ids[0]-1
        if index=='right': index=ids[-1]+1
        ranges[index]=value
    stamps=.1+np.arange(512)*.1/512
    scan=MeasurementScan(ranges_m=tuple(ranges),angles_rad=tuple(angles),acquired_at_s=tuple(stamps),
        range_min_m=.05,range_max_m=12.,source_frame_id='laser_frame',source_epoch=0,
        time_basis='simulation_acquisition')
    scene=scene_from_measurements(ranges,angles_rad=angles,acquired_at_s=stamps,t_ref_s=.2,
        range_min=.05,range_max=12.,source_frame_id='laser_frame',source_epoch=0,
        sensor_xy_m=(.0754,0.),sensor_yaw_rad=0.,acquisition_to_ref_se2=np.zeros((512,3)),
        rear_half_angle_deg=dict(rear_half_angle_deg=0.,sectors_deg=SECTORS,range_limits_m=((.05,.08),)),
        preserve_sampling=True)
    return scene,scan,np.array([[.02,-.085],[.024,-.09]]),np.full(512,.05)


def test_narrow_shadow_is_risk_not_new_free_measurement():
    scene,scan,q,pad=example(); before=scan.ranges_m
    result=current_mask_shadow(scene,scan,q,pad,SECTORS)
    assert result['accepted'] and result['residual_hidden_object_risk']
    assert not result['unknown_filled'] and not result['memory_rechecked_here']
    assert result['components'][0]['hidden_width_m']<.05
    assert scan.ranges_m==before and any(r.status=='masked' for r in scene.rays)


@pytest.mark.parametrize('value',[.14,math.inf,math.nan])
def test_near_or_missing_flank_is_not_permission(value):
    scene,scan,q,pad=example(('left',value))
    assert not current_mask_shadow(scene,scan,q,pad,SECTORS)['accepted']


def test_wide_distant_shadow_is_rejected():
    scene,scan,q,pad=example()
    q=np.array([[.0754,0.]])+ .4*np.array([[math.cos(math.radians(-123.15)),math.sin(math.radians(-123.15))]])
    result=current_mask_shadow(scene,scan,q,pad,SECTORS)
    assert not result['accepted'] and result['reason']=='wide_current_structure_shadow'


def test_undeclared_mask_is_rejected():
    scene,scan,q,pad=example()
    assert not current_mask_shadow(scene,scan,q,pad,())['accepted']


def test_visible_return_inside_nominal_sector_is_not_masked_as_structure():
    scene,scan,q,pad=example((337,.10))
    assert not current_mask_shadow(scene,scan,q,pad,SECTORS)['accepted']


@pytest.mark.parametrize('value',[math.nan,-.01])
def test_invalid_padding_is_rejected(value):
    scene,scan,q,pad=example();pad[0]=value
    assert not current_mask_shadow(scene,scan,q,pad,SECTORS)['accepted']
