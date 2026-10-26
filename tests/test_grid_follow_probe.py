import math
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

from g0_geometry import Footprint, TruthBox
from probe_grid_follow import (CAMERA, VEHICLE, RUNTIME, diagnose, json_value,
                               stationary_ranges, wall_sections)

SENSOR = dict(xyz_m=[.06,0.,.095], rate_hz=10., masked_rear_half_angle_deg=30.)
SLOT = dict(x=0., y=0., yaw_rad=0.)
ROOM = np.array([[[-2.,-1.],[3.,-1.]], [[3.,-1.],[3.,1.]],
                 [[3.,1.],[-2.,1.]], [[-2.,1.],[-2.,-1.]]])


def test_nearest_occluder_is_not_removed_when_inside_minimum_range():
    # 광선 원점 x=.06, 물체 가까운 면 x=.08: 2cm는 NaN으로 남아야 한다.
    target = TruthBox(.10,0.,.12,Footprint(.04,.15),label='close')
    ranges,labels = stationary_ranges(SLOT,[target],ROOM,SENSOR,500,.05,12.)
    assert math.isnan(ranges[250]) and labels[250]=='close'
    solo,_ = stationary_ranges(SLOT,[],ROOM,SENSOR,500,.05,12.)
    assert solo[250]==pytest.approx(2.94)


def test_short_target_does_not_occlude_scan_and_range_limit_stays_unknown():
    target = TruthBox(.4,0.,.06,label='low')
    ranges,labels = stationary_ranges(SLOT,[target],ROOM,SENSOR,500,.05,2.)
    assert math.isinf(ranges[250]) and labels[250]=='wall'


def test_world_rigid_transform_preserves_ranges():
    angle=.72; c,s=math.cos(angle),math.sin(angle)
    rotation=np.array([[c,-s],[s,c]])
    translation=np.array([3.,-2.])
    box=TruthBox(.4,.3,.12,label='other')
    before,_=stationary_ranges(SLOT,[box],ROOM,SENSOR,500,.05,12.)
    p=rotation@np.array([box.x_m,box.y_m])+translation
    after,_=stationary_ranges(dict(x=3.,y=-2.,yaw_rad=angle),
        [TruthBox(*p,.12,box.footprint,angle,label='other')],ROOM@rotation.T+translation,SENSOR,500,.05,12.)
    np.testing.assert_allclose(before,after,atol=1e-12,rtol=0.)


def test_current_configuration_paths_and_explicit_probe_mode():
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    lidar=vehicle['sensors']['lidar_2d']
    assert lidar['samples_per_scan']==500 and vehicle['drivetrain']['wheelbase_m']==.135
    with pytest.raises(ValueError):diagnose(np.ones(500),sensor,lidar,.135,'unexpected')
    with pytest.raises(ValueError):diagnose(np.ones(499),sensor,lidar,.135,'g1')


@pytest.mark.parametrize('mode',['g1','m3'])
def test_all_unknown_never_becomes_positive_or_a_departure_authorization(mode):
    lidar=dict(samples_per_scan=500,range_min_m=.05,range_max_m=12.)
    result=diagnose(np.full(500,np.inf),SENSOR,lidar,.135,mode)
    assert result['positive_request_calls']==0 and not result['driving_authorized']
    assert len(result['calls'])==20
    assert len({r['scan_first_s'] for r in result['calls']})==4


def test_nonfinite_output_is_explicit_not_zero_or_free_range():
    assert json_value(np.array([np.nan,np.inf,-np.inf,1.]))==[
        {'nonfinite':'nan'}, {'nonfinite':'+inf'}, {'nonfinite':'-inf'},1.]


def test_actual_runtime_box_visuals_are_read_at_current_sensor_plane():
    raw=(RUNTIME/'world.sdf').read_bytes()
    segments=wall_sections(raw,.095)
    assert segments.shape==(694*4,2,2) and np.isfinite(segments).all()


@pytest.mark.parametrize('fault',['tilted','mesh','negative_size','empty_plane'])
def test_unsupported_geometry_is_rejected_not_silently_skipped(fault):
    root=ET.fromstring((RUNTIME/'world.sdf').read_bytes())
    v=root.find("./world/model/link[@name='walls']/visual")
    if fault=='tilted':v.find('pose').text='0 0 .2 .1 0 0'
    if fault=='mesh':v.find('geometry/box').tag='mesh'
    if fault=='negative_size':v.find('geometry/box/size').text='-.1 .05 .4'
    with pytest.raises(ValueError):wall_sections(ET.tostring(root),1. if fault=='empty_plane' else .095)
