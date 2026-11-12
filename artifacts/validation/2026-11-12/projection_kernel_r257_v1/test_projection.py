import math
from dataclasses import replace
import numpy as np
import pytest
import candidate
from test_native_geometry import inputs,original
from arena_autonomy.native_geometry import native_motion_scene as baseline

@pytest.mark.parametrize('count',[424,508,524,8192])
@pytest.mark.parametrize('cw',[False,True])
@pytest.mark.parametrize('mask',[0.,30.,{'rear_half_angle_deg':0.,'sectors_deg':[[80.,4.],[-120.,3.]]}])
def test_full_scene_exact(count,cw,mask):
    scan,poses,c=inputs(count,cw)
    assert candidate.native_motion_scene(scan,poses,.62,c,3,mask)==original(scan,poses,c,mask=mask)

@pytest.mark.parametrize('fault',['nan','inf','bool','shape','old_reference','bad_mask','negative_epoch','overflow','bearing_overflow'])
def test_invalid_input_or_geometry(fault):
    scan,poses,c=inputs(508,True);now=.62;epoch=3;mask=0.
    if fault=='nan':poses[0,0]=math.nan
    elif fault=='inf':poses[0,0]=math.inf
    elif fault=='bool':poses=poses.astype(bool)
    elif fault=='shape':poses=poses[:-1]
    elif fault=='old_reference':now=.51
    elif fault=='bad_mask':mask=180.
    elif fault=='negative_epoch':epoch=-1
    elif fault=='overflow':poses[0]=[1e308,0.,0.];c.lidar_x_m=1e308
    elif fault=='bearing_overflow':poses[0,2]=1e308;c.lidar_yaw_rad=1e308
    with pytest.raises(ValueError):baseline(scan,poses,now,c,epoch,mask)
    with pytest.raises(ValueError):candidate.native_motion_scene(scan,poses,now,c,epoch,mask)

def test_mask_boundary_signed_seam_and_unknown():
    scan,poses,c=inputs(508,True)
    center=math.radians(80);width=math.radians(4)+1e-12
    angles=[-math.pi,math.pi,-0.,0.,center-width,center+width]
    angles += [math.nextafter(a,d) for a in (center-width,center+width) for d in (-math.inf,math.inf)]
    angles=(angles*60)[:508]
    scan=replace(scan,angles_rad=tuple(angles))
    mask={'rear_half_angle_deg':30.,'sectors_deg':[[80.,4.]]}
    actual=candidate.native_motion_scene(scan,poses,.62,c,3,mask)
    assert actual==baseline(scan,poses,.62,c,3,mask)
    assert any(r.status=='unknown' for r in actual.rays)
    assert any(r.status=='masked' for r in actual.rays)

def test_noncontiguous_transform_and_points_only():
    scan,poses,c=inputs(508,True)
    array=np.zeros((508,6));array[:,::2]=poses;view=array[:,::2]
    scene=baseline(scan,poses,.62,c,3,0.)
    points,ids=candidate.native_motion_points(scan,view,.62,c,3)
    np.testing.assert_array_equal(points,np.array(scene.occupied_endpoints_xy_m))
    np.testing.assert_array_equal(ids,[r.source_index for r in scene.rays if r.status=='return'])
