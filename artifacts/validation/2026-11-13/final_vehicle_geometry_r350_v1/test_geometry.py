"""확정 기하의 원점·범위·계획/보호·평가 일치. 실제 구동/접지 증명 아님."""
import copy
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml
from shapely.geometry import box
import g1_traffic_eval as ev
from test_g1_traffic_eval import synthetic_run

A=Path(__file__).resolve().parent
ROOT=A.parents[3]


def config():
    return yaml.safe_load((A/'vehicle.final.yaml').read_bytes())['vehicle']


def launch():
    spec=importlib.util.spec_from_file_location('r350_launch',ROOT/'src/arena_bringup/launch/simulation.launch.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    return mod


def geometry(v):
    return dict(length_m=v['footprint']['length_m'],width_m=v['footprint']['width_m'],
        wheelbase_m=v['drivetrain']['wheelbase_m'],track_width_m=v['drivetrain']['track_width_m'])


def test_body_and_tires_are_inside_envelope_and_physics_not_retuned():
    v=config();before=yaml.safe_load((ROOT/'src/arena_description/config/vehicle.yaml').read_bytes())['vehicle']
    mappings,params=launch()._vehicle_geometry_settings(v)
    assert mappings=={'body_geometry_x':'0.00365'}
    assert params=={'vehicle_length_m':.207,'vehicle_width_m':.15}
    assert v['body']['length_m']/2+v['body']['geometry_center_x_m']==pytest.approx(.1035)
    assert -v['body']['length_m']/2+v['body']['geometry_center_x_m']==pytest.approx(-.0962)
    assert v['drivetrain']['track_width_m']+v['drivetrain']['wheel_width_m']==pytest.approx(.15)
    assert v['body']['mass_kg']==before['body']['mass_kg']
    for key in ('motor','mechanical_differential','steering_servo','tire_contact','wheel_radius_m','max_steering_angle_rad'):
        assert v['drivetrain'][key]==before['drivetrain'][key]
    assert v['rule_envelope']==before['rule_envelope']


@pytest.mark.parametrize('field,value',[('length_m',.20),('width_m',.14),('length_m',float('nan'))])
def test_underestimated_or_invalid_planning_envelope_rejected(field,value):
    v=config();v['footprint'][field]=value
    with pytest.raises(ValueError):launch()._vehicle_geometry_settings(v)


def test_rendered_body_offset_does_not_move_axles_base_or_sensors():
    v=config();drive=v['drivetrain'];body=v['body']
    maps,_=launch()._vehicle_geometry_settings(v)
    maps.update(body_length=str(body['length_m']),body_width=str(body['width_m']),
        wheelbase=str(drive['wheelbase_m']),track_width=str(drive['track_width_m']),
        wheel_width=str(drive['wheel_width_m']),wheel_radius=str(drive['wheel_radius_m']))
    path=ROOT/'src/arena_description/models/arena_car/model.sdf.xacro'
    before=ET.fromstring(xacro.process_file(str(path)).toxml()).find('model')
    model=ET.fromstring(xacro.process_file(str(path),mappings=maps).toxml()).find('model')
    def pose(root,path):return list(map(float,root.find(path).text.split()))
    assert pose(model,"frame[@name='base_link']/pose")==pose(before,"frame[@name='base_link']/pose")
    assert pose(model,"link[@name='chassis']/pose")==pose(before,"link[@name='chassis']/pose")
    assert pose(model,"link[@name='d435i_link']/pose")==pose(before,"link[@name='d435i_link']/pose")
    assert pose(model,"link[@name='chassis']/collision/pose")[0]==.00365
    assert pose(model,"link[@name='chassis']/visual/pose")[0]==.00365
    for name,x,y in [('front_left',.064,.06195),('front_right',.064,-.06195),
                     ('rear_left',-.064,.06195),('rear_right',-.064,-.06195)]:
        node=model.find(f"link[@name='{name}_wheel']")
        assert list(map(float,node.find('pose').text.split()))[:3]==pytest.approx([x,y,.0325])
        assert float(node.find('collision/geometry/cylinder/length').text)==.0261
    nodes=model.findall('plugin/wheelbase')
    assert nodes and all(float(n.text)==.128 for n in nodes)


def test_new_ego_gap_is_conservative_and_default_is_unchanged():
    own=[(0.,0.,0.,0.),(.01,.001,0.,0.)]
    actor=[(0.,1.,0.,0.),(.01,1.,0.,0.),(.02,1.,0.,0.)]
    normal=ev.sampled_gaps(own,actor,.2,.15)
    assert normal==ev.sampled_gaps(own,actor,.2,.15,ego_length=.2,ego_width=.15)
    wider=ev.sampled_gaps(own,actor,.2,.15,ego_length=.207,ego_width=.15)
    assert wider[0]['gap_m']==pytest.approx(normal[0]['gap_m']-.0035)
    base=ev.swept_gap_lower_bounds(own,actor,.2,.15,.01,3.)
    bigger=ev.swept_gap_lower_bounds(own,actor,.2,.15,.01,3.,ego_length=.207,ego_width=.15)
    assert bigger[0]['lower_bound_m']<=base[0]['lower_bound_m']-.0035+1e-12


def test_course_evaluation_uses_selected_axles_and_envelope():
    from validate_g1_traffic import load_config
    cfg=load_config(ROOT/'config/tests/g1_traffic_v1.json')
    cfg['ego_geometry']=geometry(config())
    cfg['acceptance']['lap_probe']={**cfg['acceptance']['stopped_center'],'minimum_laps':1}
    case=dict(name='lap_probe',behavior_kind='stopped_center',route_kind='full_course',duration_s=10.,schedule=[])
    result=synthetic_run(3.2,config=cfg,case=case,road=box(-.225,-10,.225,100))
    assert result['ego_geometry']==cfg['ego_geometry']
    assert result['wheel_centers']['wheelbase_m']==.128
    assert result['wheel_centers']['track_m']==.1239
    assert not result['gates']['full_lap']
    assert result['gates']['two_wheel_surface']


@pytest.mark.parametrize('value',[{},dict(length_m=.207),dict(length_m=.207,width_m=.15,wheelbase_m=.128,track_width_m=float('inf'))])
def test_incomplete_or_invalid_evaluation_geometry_fails(value):
    with pytest.raises(ValueError):ev.ego_geometry({'ego_geometry':value})
