"""명시 차량 기하의 ROS/측방/비구동 preview 전달 계약. 장치·ROS 실행 없음."""
import ast
import copy
from dataclasses import asdict
import math
from pathlib import Path
import struct
import subprocess
import sys
from types import SimpleNamespace

import pytest
import yaml

ROOT=Path(__file__).resolve().parents[1]
for relative in ('src/arena_autonomy','src/arena_vehicle_interface','scripts'):
    sys.path.insert(0,str(ROOT/relative))
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.m3_execution_trial import M3ExecutionFollow
from arena_autonomy.m3_observed_follow import M3ObservedFollow
from arena_autonomy.vehicle_geometry import planning_parameters,preview_parameters,equivalent_steering_limit,float32_limit_not_above
import probe_g1_hardware_shadow as probe
import run_hardware_sensor_preview as runner

VEHICLE=ROOT/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'


def vehicle():return yaml.safe_load(VEHICLE.read_bytes())['vehicle']


def launch_function(name):
    path=ROOT/'src/arena_bringup/launch/simulation.launch.py'
    node=next(n for n in ast.parse(path.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name==name)
    scope=dict(math=math,struct=struct)
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


def test_legacy_keywords_and_roundtrip_keep_historical_dimensions():
    cfg=FollowConfig(wheelbase_m=.128,length_m=.207,width_m=.15)
    lat=ps.lateral_config(cfg)
    assert (lat.track_m,lat.wheel_radius_m,lat.wheel_width_m)==(.135,.0325,.012)
    assert FollowConfig(**asdict(cfg))==cfg


def test_explicit_yaml_reaches_all_three_lateral_constructors():
    params=planning_parameters(vehicle())
    cfg=FollowConfig(occupancy_timing_mode='per_ray_candidate',length_m=params['vehicle_length_m'],
        width_m=params['vehicle_width_m'],wheelbase_m=.128,track_m=params['vehicle_track_width_m'],
        wheel_radius_m=params['vehicle_wheel_radius_m'],wheel_width_m=params['vehicle_wheel_width_m'])
    assert FollowConfig(**asdict(cfg))==cfg
    for lat in (ps.lateral_config(cfg),M3ExecutionFollow(cfg,allow_simulation_trial=True).lateral_config,
                M3ObservedFollow(cfg,allow_simulation_trial=True).lateral_config):
        assert (lat.length_m,lat.width_m,lat.wheelbase_m)==(.207,.15,.128)
        assert (lat.track_m,lat.wheel_radius_m,lat.wheel_width_m)==(.1239,.0325,.0261)


@pytest.mark.parametrize('field,value',[(field,value) for field in ('track_width_m','wheel_radius_m','wheel_width_m')
    for value in (0.,-.01,float('nan'),float('inf'),True,'0.03')])
def test_invalid_explicit_geometry_never_silently_defaults(field,value):
    selected=vehicle();selected['drivetrain'][field]=value
    with pytest.raises((ValueError,TypeError)):planning_parameters(selected)


@pytest.mark.parametrize('field',('track_width_m','wheel_radius_m','wheel_width_m'))
def test_missing_explicit_geometry_rejected(field):
    selected=vehicle();del selected['drivetrain'][field]
    with pytest.raises(KeyError):planning_parameters(selected)


def test_wheels_must_fit_planning_envelope():
    selected=vehicle();selected['drivetrain']['wheel_width_m']=.04
    with pytest.raises(ValueError,match='contain'):planning_parameters(selected)


def test_launch_and_ros_follow_constructor_forward_dimensions():
    geometry,params=launch_function('_vehicle_geometry_settings')(vehicle())
    assert params==planning_parameters(vehicle()) and geometry=={'body_geometry_x':'0.00365'}
    tree=ast.parse((ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_bytes())
    calls=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='FollowConfig']
    assert len(calls)==1
    keys={kw.arg:ast.unparse(kw.value) for kw in calls[0].keywords}
    for field,param in {'track_m':'vehicle_track_width_m','wheel_radius_m':'vehicle_wheel_radius_m','wheel_width_m':'vehicle_wheel_width_m'}.items():
        assert 'declare_parameter' in keys[field] and param in keys[field]


@pytest.mark.parametrize('model',('ideal_ackermann','fusion_cad_2026_11_13'))
def test_preview_sender_limit_matches_launch_and_float32_stays_inside_physical_limit(model):
    selected=vehicle();selected['drivetrain']['simulation_steering_model']=model
    physical=launch_function('_equivalent_steering_limit')(selected['drivetrain'])
    assert physical==equivalent_steering_limit(selected['drivetrain'])
    sent=preview_parameters(selected)['max_steering_angle_rad']
    assert sent==launch_function('_float32_limit_not_above')(physical)==float32_limit_not_above(physical)<=physical
    assert struct.unpack('<f',struct.pack('<f',sent))[0]==sent
    bits=struct.unpack('<I',struct.pack('<f',sent))[0]
    assert struct.unpack('<f',struct.pack('<I',bits+1))[0]>physical


def args(**changes):
    values=dict(output=Path('/tmp/geometry-plan-only'),seconds=20.,imu_port='/dev/absent-imu',c1_port='/dev/absent-c1',
        c1_executable=Path('/absent-sdk'),imu_source='probe',estimated_time=True,camera=False)
    return SimpleNamespace(**(values|changes))


def test_preview_default_unchanged_and_selection_adds_only_geometry():
    default=probe.ros_arguments(args(),ROOT)
    assert not any('vehicle_track' in value or 'vehicle_wheel' in value for value in default)
    assert '--vehicle-config' not in runner.commands(args())['node']
    selected=args(vehicle_config=VEHICLE);override=probe.vehicle_geometry_override(selected)
    assert override['parameters']==preview_parameters(vehicle())
    assert not {'max_speed_mps','motion_scan_time_policy','motion_gear_ratio'} & override['parameters'].keys()
    ros=probe.ros_arguments(selected,ROOT)
    for name,value in override['parameters'].items():assert name+':='+str(value) in ros
    assert override['parameters']['motion_rear_axle_x_m']==-.064
    assert override['parameters']['motion_wheel_radius_m']==.0325
    command=runner.commands(selected)['node'];assert command[command.index('--vehicle-config')+1]==str(VEHICLE)


def test_invalid_selected_geometry_fails_before_device_io(tmp_path):
    selected=vehicle();selected['drivetrain']['wheel_width_m']=-.1
    path=tmp_path/'invalid.yaml';path.write_text(yaml.safe_dump({'vehicle':selected}))
    output=tmp_path/'must-remain-absent'
    result=subprocess.run([sys.executable,'-B',str(ROOT/'scripts/run_hardware_sensor_preview.py'),
        '--output',str(output),'--imu-port','absent-imu','--c1-port','absent-c1','--c1-executable','absent-sdk',
        '--c1-sha256','0'*64,'--vehicle-config',str(path),'--plan-only'],capture_output=True,text=True)
    assert result.returncode!=0 and not output.exists() and 'finite positive' in result.stderr
