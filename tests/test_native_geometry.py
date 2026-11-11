from dataclasses import replace
from types import SimpleNamespace
import math
import numpy as np
import pytest
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.local_scene import scene_from_measurements,SensorRay
from arena_autonomy.native_geometry import native_motion_scene as build, native_motion_points

def inputs(n,clockwise):
    rng=np.random.default_rng(1111+n)
    ranges=rng.uniform(.01,15,n).tolist()
    for i,v in enumerate([math.nan,math.inf,-math.inf,0.]):ranges[i*7]=v
    angles=tuple((1 if clockwise else -1)*(math.pi-i*math.tau/n) for i in range(n))
    scan=MeasurementScan(tuple(ranges),angles,tuple(.5+i*.1/n for i in range(n)),time_basis='simulation_acquisition')
    poses=rng.normal(0,.1,(n,3))
    config=SimpleNamespace(lidar_x_m=.06,lidar_y_m=.013,lidar_yaw_rad=-.1)
    return scan,poses,config

def original(scan,poses,config,now=.62,epoch=3,mask=0.):
    return scene_from_measurements(scan.ranges_m,angles_rad=scan.angles_rad,
        acquired_at_s=scan.acquired_at_s,t_ref_s=now,range_min=scan.range_min_m,range_max=scan.range_max_m,
        source_frame_id=scan.source_frame_id,source_epoch=epoch,sensor_xy_m=(config.lidar_x_m,config.lidar_y_m),
        sensor_yaw_rad=config.lidar_yaw_rad,acquisition_to_ref_se2=poses.tolist(),rear_half_angle_deg=mask,preserve_sampling=True)

@pytest.mark.parametrize('n',[424,508,524])
@pytest.mark.parametrize('clockwise',[False,True])
@pytest.mark.parametrize('mask',[0.,30.,{'rear_half_angle_deg':0.,'sectors_deg':[[80.,4.],[-120.,3.]]}])
def test_complete_scene_is_exactly_equal(n,clockwise,mask):
    scan,poses,config=inputs(n,clockwise)
    assert build(scan,poses,.62,config,3,mask)==original(scan,poses,config,mask=mask)

@pytest.mark.parametrize('fault',['nan','inf','bool','shape','old_reference','bad_mask','negative_epoch','overflow'])
def test_invalid_boundary_and_output_still_rejected(fault):
    scan,poses,c=inputs(508,True);now=.62;epoch=3;mask=0.
    if fault=='nan':poses[0,0]=math.nan
    if fault=='inf':poses[0,0]=math.inf
    if fault=='bool':poses=poses.astype(bool)
    if fault=='shape':poses=poses[:-1]
    if fault=='old_reference':now=.51
    if fault=='bad_mask':mask=180.
    if fault=='negative_epoch':epoch=-1
    if fault=='overflow':poses[0]=[1e308,0.,0.];c.lidar_x_m=1e308
    with pytest.raises(ValueError):original(scan,poses,c,now,epoch,mask)
    with pytest.raises(ValueError):build(scan,poses,now,c,epoch,mask)

def test_public_ray_constructor_validation_not_disabled():
    with pytest.raises(ValueError):SensorRay((0.,0.),(math.inf,0.),.1,'return','laser_frame',0,0,(1.,0.))

@pytest.mark.parametrize('n',[424,508,524])
@pytest.mark.parametrize('clockwise',[False,True])
def test_points_only_match_full_reference_scene(n,clockwise):
    scan,poses,config=inputs(n,clockwise)
    scene=original(scan,poses,config)
    points,ids=native_motion_points(scan,poses,.62,config,3)
    assert np.array_equal(points,np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2))
    assert np.array_equal(ids,[r.source_index for r in scene.rays if r.status=='return'])

def test_empty_returns_stay_empty_and_unknown_remains_in_scene():
    scan,poses,config=inputs(508,True)
    scan=replace(scan,ranges_m=(math.inf,)*508)
    points,ids=native_motion_points(scan,poses,.62,config,3)
    assert points.shape==(0,2) and ids.shape==(0,)
    assert all(r.status=='unknown' for r in build(scan,poses,.62,config,3,0.).rays)
