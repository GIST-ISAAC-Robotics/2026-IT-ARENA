"""최종 YAML→양 노드 overlay/host JSON의 순수 검증. ROS/장치/시뮬 없음."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

import pytest
import yaml

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(HERE))
import generate_profile as gp
from arena_autonomy.local_path import mask_scan
from arena_autonomy.measurement_scan import MeasurementScan
from arena_vehicle_interface.motor_feedback_runtime import MotorFeedbackRuntime
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec

VEHICLE=ROOT/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml'
SENSOR=ROOT/'src/arena_description/config/b0390_c1_cad_20261113_final_frontwire_6mm.yaml'
ASSUMPTIONS=HERE/'assumptions.example.yaml'
OLDMASK=ROOT/'artifacts/validation/2026-11-13/cad_mount_r347_v1/frontwire_6mm_nominal_mask.json'


def inputs():
    vehicle=gp.read_yaml(VEHICLE)['vehicle'];sensor=gp.read_yaml(SENSOR)
    sensor['lidar']['scan_plane_meshes']=[str((SENSOR.parent/name).resolve()) for name in sensor['lidar']['scan_plane_meshes']]
    return vehicle,sensor,gp.read_yaml(ASSUMPTIONS)


def test_final_geometry_mask_gear_and_source_time_reach_both_named_nodes():
    vehicle,sensor,a=inputs();before=copy.deepcopy((vehicle,sensor,a))
    profile,motor,mask,record=gp.build(vehicle,sensor,a)
    for name in ('local_pursuit','lidar_safety'):
        p=profile[name]['ros__parameters']
        assert (p['wheelbase_m'],p['vehicle_length_m'],p['vehicle_width_m'])==(.128,.207,.15)
        assert p['max_steering_angle_rad']==.4578436017036438
        assert (p['motion_rear_axle_x_m'],p['motion_lidar_x_m'],p['motion_lidar_y_m'])==(-.064,.0754,0.)
        assert p['motion_feedback_mode']=='drive_motor_shaft' and p['motion_gear_ratio']==2.5*81/17
        assert p['motion_wheel_radius_m']==.0325 and p['motion_imu_topic']=='/hardware_runtime/imu'
        assert p['execution_environment']=='hardware_runtime_candidate_v1' and not p['use_sim_time']
        assert p['motion_scan_time_policy']=='host_estimated_v1' and p['motion_scan_timing_verified'] is False
        assert p['lidar_input_representation']=='native_estimated_v1' and p['rear_blind_half_angle_deg']==0.
        assert json.loads(p['lidar_blind_sectors_json'])==mask and len(mask['sectors_deg'])==4
    planner=profile['local_pursuit']['ros__parameters']
    assert (planner['vehicle_track_width_m'],planner['vehicle_wheel_width_m'],planner['vehicle_wheel_radius_m'])==(.1239,.0261,.0325)
    assert motor['counts_per_motor_revolution']==28 and motor['gear_ratio']==2.5*81/17 and motor['wheel_radius_m']==.0325
    assert not any('max_speed_mps' in group['ros__parameters'] for group in profile.values())
    assert record['selected_marker']['xyz_m'][2]==.09 and record['reproduction_only']['applied_to_runtime'] is False
    assert record['assumed']['status']=='provisional_not_installed_calibration'
    assert not record['hardware_calibration_verified'] and not record['physical_actuation_verified']
    assert not record['stream_contracts']['native_lidar']['frame_checked_by_current_autonomy']
    assert (vehicle,sensor,a)==before


def test_r347_marker140_is_separate_reproduction_not_runtime_fifth_sector():
    profile,motor,mask,record=gp.build(*inputs())
    saved=json.loads(OLDMASK.read_text())
    reference=record['reproduction_only']['mask']
    assert len(saved['sectors_deg'])==len(reference['sectors_deg'])==5 and len(mask['sectors_deg'])==4
    for field in ('sectors_deg','range_limits_m'):
        for produced,original in zip(reference[field],saved[field]):assert produced==pytest.approx(original,abs=1e-12,rel=0)
        assert mask[field]==reference[field][:4]


def test_sensor_yaw_rotates_only_angular_centres_not_bands_or_protection_widths():
    vehicle,sensor,a=inputs();plain=gp.build(vehicle,sensor,a)[2]
    a['lidar_yaw_rad']=.3
    profile,_,rotated,_=gp.build(vehicle,sensor,a)
    assert rotated['range_limits_m']==plain['range_limits_m']
    for old,new in zip(plain['sectors_deg'],rotated['sectors_deg']):
        assert new[0]==pytest.approx(math.remainder(old[0]-math.degrees(.3),360.)) and new[1]==old[1]
    assert profile['lidar_safety']['ros__parameters']['motion_lidar_yaw_rad']==.3


def test_mask_retains_distant_return_and_leaves_no_return_unknown():
    _,_,mask,_=gp.build(*inputs())
    angle=math.radians(mask['sectors_deg'][0][0]);near=sum(mask['range_limits_m'][0])/2
    scan=MeasurementScan(ranges_m=(near,1.,math.nan),angles_rad=(angle,angle,angle),
        acquired_at_s=(10.,10.001,10.002),source_frame_id='declared_sensor',source_epoch=1,
        range_min_m=.02,range_max_m=12.,time_basis='unverified_estimate')
    result=mask_scan(scan,gp.blind_parameter(0.,json.dumps(mask)))
    assert math.isnan(result.ranges[0]) and result.ranges[1]==1. and math.isnan(result.ranges[2])


@pytest.mark.parametrize('field,value',[('track_width_m',.135),('wheel_width_m',.012),('wheelbase_m',.135),('wheel_radius_m',.025)])
def test_legacy_geometry_cannot_return_silently(field,value):
    vehicle,sensor,a=inputs();vehicle['drivetrain'][field]=value
    with pytest.raises(ValueError,match='final geometry'):gp.build(vehicle,sensor,a)


@pytest.mark.parametrize('fault',['count','gear','source','marker140','missing_width'])
def test_wrong_selected_source_or_contract_is_rejected(fault):
    vehicle,sensor,a=inputs()
    if fault=='count':vehicle['sensors']['drive_feedback']['drive_motor_encoder']['counts_per_motor_revolution']=2048
    elif fault=='gear':vehicle['drivetrain']['motor']['gear_ratio']=10.
    elif fault=='source':sensor['mount_provenance']['source_step_sha256']='different'
    elif fault=='marker140':vehicle['identification_marker']['xyz_m'][2]=.14
    else:vehicle['drivetrain'].pop('wheel_width_m')
    with pytest.raises((ValueError,KeyError)):gp.build(vehicle,sensor,a)


@pytest.mark.parametrize('fault',['missing_sign','invalid_sign','nan_axis','verified','missing_note','imu_frame','command_topic'])
def test_unresolved_mounting_needs_explicit_provisional_declarations(fault):
    vehicle,sensor,a=inputs()
    if fault=='missing_sign':a.pop('encoder_sign')
    elif fault=='invalid_sign':a['encoder_sign']=0
    elif fault=='nan_axis':a['imu_roll_rad']=math.nan
    elif fault=='verified':a['status']='calibrated'
    elif fault=='missing_note':a['note']=''
    elif fault=='imu_frame':a['imu_frame_id']='base_link'
    else:a['native_lidar_topic']='/drive/safe'
    with pytest.raises(ValueError):gp.build(vehicle,sensor,a)


def test_negative_explicit_encoder_sign_is_not_overwritten():
    vehicle,sensor,a=inputs();a['encoder_sign']=-1
    _,motor,_,_=gp.build(vehicle,sensor,a)
    spec=MotorEncoderSpec(motor['counts_per_motor_revolution'],motor['gear_ratio'],motor['wheel_radius_m'])
    converted=MotorFeedbackRuntime(spec,**{key:motor[key] for key in ('encoder_sign','maximum_interval_ns','maximum_age_ns','maximum_speed_mps')})
    assert converted.sign==-1


@pytest.mark.parametrize('fault',['missing_guard','old_shadow','wrong_clock','wrong_mask','wrong_gear','wrong_host_count'])
def test_composed_profile_cannot_drop_contract_or_mix_old_mask(fault):
    profile,motor,mask,registration=gp.build(*inputs())
    if fault=='missing_guard':profile.pop('lidar_safety')
    elif fault=='old_shadow':profile=gp.read_yaml(ROOT/'config/hardware/g1_input_shadow_v1.yaml')
    elif fault=='wrong_clock':profile['lidar_safety']['ros__parameters']['motion_scan_timing_verified']=True
    elif fault=='wrong_mask':profile['local_pursuit']['ros__parameters']['lidar_blind_sectors_json']=OLDMASK.read_text()
    elif fault=='wrong_gear':profile['lidar_safety']['ros__parameters']['motion_gear_ratio']=10.
    else:motor['counts_per_motor_revolution']=2048
    with pytest.raises(ValueError,match='missing/mixed'):gp.validate_final(profile,motor,registration)


def test_generate_roundtrip_hashes_and_never_overwrites_sources(tmp_path):
    paths=(VEHICLE,SENSOR,ASSUMPTIONS,OLDMASK)
    before={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    output=tmp_path/'new'
    result=gp.generate(VEHICLE,SENSOR,ASSUMPTIONS,output)
    profile=gp.read_yaml(output/'runtime_geometry.yaml');motor=json.loads((output/'motor_feedback.json').read_text())
    registration=json.loads((output/'registration.json').read_text())
    assert result['sectors']==4 and gp.validate_final(profile,motor,registration)['owned_parameters_match']
    manifest=json.loads((output/'output_manifest.json').read_text())
    assert all(gp.digest(output/name)==value for name,value in manifest.items())
    gp.verify_manifest(output)
    assert {str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}==before
    with pytest.raises(ValueError,match='fresh'):gp.generate(VEHICLE,SENSOR,ASSUMPTIONS,output)


def test_empty_manifest_or_output_edit_cannot_pass_check(tmp_path):
    output=tmp_path/'new';gp.generate(VEHICLE,SENSOR,ASSUMPTIONS,output)
    (output/'motor_feedback.json').write_text('{}')
    with pytest.raises(ValueError,match='changed'):gp.verify_manifest(output)
    (output/'output_manifest.json').write_text('{}')
    with pytest.raises(ValueError,match='exact generated'):gp.verify_manifest(output)


def test_yaml_duplicate_key_is_not_last_value_wins(tmp_path):
    path=tmp_path/'bad.yaml';path.write_text('wheelbase_m: 0.128\nwheelbase_m: 0.135\n')
    with pytest.raises(ValueError,match='duplicate'):gp.read_yaml(path)
