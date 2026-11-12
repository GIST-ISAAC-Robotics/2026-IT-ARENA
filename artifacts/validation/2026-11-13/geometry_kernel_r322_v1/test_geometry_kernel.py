"""선택형 C++ 투영: 원천 순서·unknown·마스크 경계·부동소수 출력 동등성."""
import math
from dataclasses import replace
from types import SimpleNamespace
import numpy as np
import pytest
from arena_autonomy import numeric_kernels as backend
from arena_autonomy.local_scene import _blind_limits
from arena_autonomy.native_geometry import native_motion_scene,native_motion_points
from test_native_geometry import inputs,original

extension=pytest.importorskip('arena_autonomy._numeric_kernels')


@pytest.fixture(autouse=True)
def explicit_backend():
    assert hasattr(extension,'geometry'),'rebuild native extension for geometry tests'
    backend.configure('native_v1')
    yield
    backend.configure('python')


@pytest.mark.parametrize('n',[424,508,524])
@pytest.mark.parametrize('clockwise',[False,True])
@pytest.mark.parametrize('mask',[0.,30.,{'rear_half_angle_deg':30.,
    'sectors_deg':[[80.,4.],[-120.,3.]],'range_limits_m':[[.02,.15],[.06,.5]]}])
def test_full_ray_values_and_point_order_identical(n,clockwise,mask):
    scan,poses,c=inputs(n,clockwise)
    assert native_motion_scene(scan,poses,.62,c,3,mask)==original(scan,poses,c,mask=mask)
    points,indices=native_motion_points(scan,poses,.62,c,3)
    scene=original(scan,poses,c)
    np.testing.assert_array_equal(points,np.asarray(scene.occupied_endpoints_xy_m))
    np.testing.assert_array_equal(indices,[r.source_index for r in scene.rays if r.status=='return'])


def test_closed_mask_and_range_boundaries_nearby_values():
    mask={'rear_half_angle_deg':30.,'sectors_deg':[[80.,4.],[-120.,3.]],
          'range_limits_m':[[.02,.15],[.06,.5]]}
    rear,sectors,_=_blind_limits(mask)
    values=[]
    for a in [-math.pi,math.pi,-rear,rear,*[c+s*w for c,w in sectors for s in [-1,1]]]:
        for angle in [np.nextafter(a,-math.inf),a,np.nextafter(a,math.inf)]:
            for d in [.02,np.nextafter(.02,-math.inf),.15,np.nextafter(.15,math.inf),
                      .05,12.,12.001,.01,math.nan,math.inf,-math.inf]:values.append((angle,float(d)))
    scan,_,c=inputs(508,True)
    scan=replace(scan,angles_rad=tuple(a for a,d in values),ranges_m=tuple(d for a,d in values),
        acquired_at_s=tuple(.5+i*1e-5 for i in range(len(values))))
    poses=np.zeros((len(values),3))
    assert native_motion_scene(scan,poses,.62,c,3,mask)==original(scan,poses,c,mask=mask)


@pytest.mark.parametrize('fault',['nan','inf','bool','shape','old_reference','bad_mask','negative_epoch','overflow','bearing_overflow'])
def test_original_rejection_preserved(fault):
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
    with pytest.raises(ValueError):original(scan,poses,c,now,epoch,mask)
    with pytest.raises(ValueError):native_motion_scene(scan,poses,now,c,epoch,mask)


@pytest.mark.parametrize('fault',['length','type','readonly','noncontiguous','oversize','negative_sectors','bad_config','bad_pose'])
def test_native_buffer_boundary(fault):
    n=2;m=0;r=np.ones(n);a=np.zeros(n);p=np.zeros((n,3))
    config=np.array([.06,0.,0.,.05,12.,math.pi+1e-12,math.tau]);masks=np.empty((0,4));out=np.empty((n,7))
    if fault=='length':out=np.empty((1,7))
    elif fault=='type':r=r.astype(np.float32)
    elif fault=='readonly':out.flags.writeable=False
    elif fault=='noncontiguous':p=np.zeros((n,6))[:,::2]
    elif fault=='oversize':n=16385
    elif fault=='negative_sectors':m=-1
    elif fault=='bad_config':config[0]=math.nan
    elif fault=='bad_pose':p[0,0]=math.inf
    with pytest.raises((ValueError,BufferError)):extension.geometry(n,m,r,a,p,config,masks,out)


def test_python_default_and_old_extension_fallback(monkeypatch):
    backend.configure('python')
    assert backend.geometry(None,None,None,None,None,None) is None
    monkeypatch.setattr(backend,'_backend',SimpleNamespace(ABI_VERSION=1))
    assert backend.geometry(None,None,None,None,None,None) is None
