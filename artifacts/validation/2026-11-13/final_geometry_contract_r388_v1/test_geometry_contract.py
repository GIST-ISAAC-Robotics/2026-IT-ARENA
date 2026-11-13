"""선택형 기하 전달/legacy 재현 검증. ROS·장치·주행 없음. 실행은 지정 담당자만."""
import ast
import copy
from dataclasses import asdict,replace
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest
import yaml
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.vehicle_geometry import planning_parameters,preview_parameters,equivalent_steering_limit
from arena_autonomy.m3_execution_trial import M3ExecutionFollow
from arena_autonomy.m3_observed_follow import M3ObservedFollow
import probe_g1_hardware_shadow as probe
import run_hardware_sensor_preview as runner

A=Path(__file__).resolve().parent
ROOT=Path(os.environ['R388_GEOMETRY_ROOT']).resolve()
VEHICLE=ROOT/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'


def vehicle():return yaml.safe_load(VEHICLE.read_bytes())['vehicle']


def extract_function(path,name):
    tree=ast.parse(path.read_bytes());node=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
    scope={'math':math};exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


def test_legacy_header_and_roundtrip_keep_historical_lateral_defaults():
    header=json.loads((A/'input_header.jsonl').read_bytes())['data']
    assert not {'track_m','wheel_radius_m','wheel_width_m'} & header['follow_config'].keys()
    cfg=FollowConfig(**header['follow_config']);lat=ps.lateral_config(cfg)
    assert (lat.track_m,lat.wheel_radius_m,lat.wheel_width_m)==(.135,.0325,.012)
    assert FollowConfig(**asdict(cfg))==cfg


def test_selected_yaml_reaches_every_lateral_constructor():
    params=planning_parameters(vehicle())
    cfg=FollowConfig(occupancy_timing_mode='per_ray_candidate',length_m=params['vehicle_length_m'],
        width_m=params['vehicle_width_m'],wheelbase_m=.128,track_m=params['vehicle_track_width_m'],
        wheel_radius_m=params['vehicle_wheel_radius_m'],wheel_width_m=params['vehicle_wheel_width_m'])
    assert FollowConfig(**asdict(cfg))==cfg
    for lat in (ps.lateral_config(cfg),M3ExecutionFollow(cfg,allow_simulation_trial=True).lateral_config,
                M3ObservedFollow(cfg,allow_simulation_trial=True).lateral_config):
        assert (lat.track_m,lat.wheel_radius_m,lat.wheel_width_m)==(.1239,.0325,.0261)
        assert (lat.length_m,lat.width_m,lat.wheelbase_m)==(.207,.15,.128)


@pytest.mark.parametrize('field,value',[(f,v) for f in ('track_width_m','wheel_radius_m','wheel_width_m')
    for v in (0,-.01,float('nan'),float('inf'),True,'0.03')])
def test_explicit_invalid_dimensions_rejected(field,value):
    selected=vehicle();selected['drivetrain'][field]=value
    with pytest.raises((ValueError,TypeError)):planning_parameters(selected)


@pytest.mark.parametrize('field',('track_width_m','wheel_radius_m','wheel_width_m'))
def test_explicit_missing_dimension_is_not_legacy_fallback(field):
    selected=vehicle();del selected['drivetrain'][field]
    with pytest.raises(KeyError):planning_parameters(selected)


def test_invalid_enclosing_geometry_rejected():
    selected=vehicle();selected['drivetrain']['wheel_width_m']=.04
    with pytest.raises(ValueError,match='contain'):planning_parameters(selected)


def test_launch_maps_yaml_and_shared_steering_is_exact_original():
    launch=ROOT/'src/arena_bringup/launch/simulation.launch.py'
    original=A/'base/src/arena_bringup/launch/simulation.launch.py'
    mapping,params=extract_function(launch,'_vehicle_geometry_settings')(vehicle())
    assert params==planning_parameters(vehicle()) and mapping=={'body_geometry_x':'0.00365'}
    before=extract_function(original,'_equivalent_steering_limit')
    after=extract_function(launch,'_equivalent_steering_limit')
    for model in ('ideal_ackermann','fusion_cad_2026_11_13'):
        drive=copy.deepcopy(vehicle()['drivetrain']);drive['simulation_steering_model']=model
        assert before(drive)==after(drive)==equivalent_steering_limit(drive)


def test_ros_constructor_declares_and_consumes_selected_dimensions():
    tree=ast.parse((ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_bytes())
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='FollowConfig']
    assert len(calls)==1
    mapping={'track_m':'vehicle_track_width_m','wheel_radius_m':'vehicle_wheel_radius_m','wheel_width_m':'vehicle_wheel_width_m'}
    keys={kw.arg:ast.unparse(kw.value) for kw in calls[0].keywords}
    for field,param in mapping.items():assert 'declare_parameter' in keys[field] and param in keys[field]


def args(**changes):
    data=dict(output=Path('/tmp/r388-no-device'),seconds=20.,imu_port='/dev/absent-imu',
        c1_port='/dev/absent-c1',c1_executable=Path('/absent-sdk'),imu_source='probe',
        estimated_time=True,camera=False)
    return SimpleNamespace(**(data|changes))


def test_preview_default_remains_legacy_and_selected_yaml_only_adds_geometry():
    default=probe.ros_arguments(args(),ROOT)
    assert not any('vehicle_track' in v or 'vehicle_wheel' in v for v in default)
    assert '--vehicle-config' not in runner.commands(args())['node']
    selected=args(vehicle_config=VEHICLE);override=probe.vehicle_geometry_override(selected)
    assert override['parameters']==preview_parameters(vehicle())
    assert not {'max_speed_mps','motion_gear_ratio','motion_scan_time_policy'} & override['parameters'].keys()
    selected_ros=probe.ros_arguments(selected,ROOT)
    for name,value in override['parameters'].items():assert name+':='+str(value) in selected_ros
    assert override['parameters']['wheelbase_m']==.128
    assert override['parameters']['motion_rear_axle_x_m']==-.064
    command=runner.commands(selected)['node'];assert command[command.index('--vehicle-config')+1]==str(VEHICLE)


def test_invalid_preview_selection_fails_before_device_io(tmp_path):
    selected=vehicle();selected['drivetrain']['wheel_width_m']=-.1
    path=tmp_path/'bad.yaml';path.write_text(yaml.safe_dump({'vehicle':selected}))
    output=tmp_path/'must-stay-absent'
    result=subprocess.run([sys.executable,'-B',str(ROOT/'scripts/run_hardware_sensor_preview.py'),
        '--output',str(output),'--imu-port','absent-imu','--c1-port','absent-c1',
        '--c1-executable','absent-sdk','--c1-sha256','0'*64,'--vehicle-config',str(path),'--plan-only'],
        capture_output=True,text=True)
    assert result.returncode!=0 and not output.exists() and 'finite positive' in result.stderr
