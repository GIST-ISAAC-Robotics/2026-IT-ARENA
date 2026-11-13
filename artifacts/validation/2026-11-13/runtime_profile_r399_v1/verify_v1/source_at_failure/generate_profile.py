#!/usr/bin/env python3
"""최종 CAD의 기하/관측 계약 overlay 생성. ROS/장치/주행 실행 및 정책 자동 채택 없음."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

import yaml

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
for path in (ROOT/'src/arena_autonomy',ROOT/'src/arena_vehicle_interface'):
    sys.path.insert(0,str(path))
from arena_autonomy.vehicle_geometry import preview_parameters
from arena_autonomy.cad_mount import mount
from arena_autonomy.local_scene import blind_parameter
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec,MOTOR_TOPIC,MOTOR_JOINT,MOTOR_FRAME
from arena_vehicle_interface.motor_feedback_runtime import MotorFeedbackRuntime

CAD_SHA='c9c203b5a66585ccde71313ea888c16804a1ddb71e6e91514a46bebededbfce6'
ASSUMPTION_KEYS={'schema','status','note','native_lidar_topic','native_lidar_frame_id','lidar_yaw_rad',
    'camera_topic','camera_frame_id','imu_topic','imu_frame_id','imu_roll_rad','imu_pitch_rad','encoder_sign',
    'motor_maximum_interval_ns','motor_maximum_age_ns','motor_maximum_speed_mps'}


class ExactLoader(yaml.SafeLoader):pass


def unique_mapping(loader,node,deep=False):
    result={}
    for key_node,value_node in node.value:
        key=loader.construct_object(key_node,deep=deep)
        if key in result:raise ValueError('duplicate YAML key: '+str(key))
        result[key]=loader.construct_object(value_node,deep=deep)
    return result


ExactLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,unique_mapping)


def read_yaml(path):return yaml.load(Path(path).read_text(encoding='utf-8'),Loader=ExactLoader)
def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def json_text(value):return json.dumps(value,ensure_ascii=False,allow_nan=False,indent=2)+'\n'


def finite(value,name):
    if type(value) not in (int,float) or not math.isfinite(value):raise ValueError(name+' must be finite numeric')
    return float(value)


def assumptions(data):
    if type(data) is not dict or set(data)!=ASSUMPTION_KEYS:raise ValueError('exact explicit assumptions required')
    if data['schema']!='runtime_profile_assumptions_v1' or data['status']!='provisional_not_installed_calibration':
        raise ValueError('this candidate cannot assert installed calibration')
    if type(data['note']) is not str or not data['note'].strip():raise ValueError('assumption note required')
    for name in ('native_lidar_topic','camera_topic','imu_topic','native_lidar_frame_id','camera_frame_id','imu_frame_id'):
        value=data[name]
        if type(value) is not str or not value or any(c.isspace() for c in value):raise ValueError(name+' must be explicit')
        if name.endswith('topic') and (not value.startswith('/') or value in ('/drive','/drive/safe','/actuation/tx')):
            raise ValueError('separate sensor topic required')
    if data['imu_frame_id']!='lsm6dsox_sensor':raise ValueError('r394 runner emits lsm6dsox_sensor; do not silently relabel')
    for key in ('lidar_yaw_rad','imu_roll_rad','imu_pitch_rad'):finite(data[key],key)
    if abs(data['lidar_yaw_rad'])>math.pi:raise ValueError('explicit lidar yaw within -pi..pi required')
    return copy.deepcopy(data)


def final_vehicle(vehicle,sensor):
    if (vehicle['cad_geometry_provenance']['source_sha256']!=CAD_SHA
            or sensor['mount_provenance']['source_step_sha256']!=CAD_SHA):
        raise ValueError('matching final CAD provenance required')
    drive=vehicle['drivetrain'];feedback=vehicle['sensors']['drive_feedback'];encoder=feedback['drive_motor_encoder']
    expected=dict(wheelbase_m=.128,track_width_m=.1239,wheel_radius_m=.0325,wheel_width_m=.0261)
    if any(drive[key]!=value for key,value in expected.items()):raise ValueError('final geometry differs; no legacy fallback')
    if feedback['enabled'] is not True or feedback['mode']!='drive_motor_shaft':raise ValueError('explicit motor feedback required')
    gearing=drive['motor']['gearing_reference']
    if (gearing['internal_reduction'],gearing['spur_teeth'],gearing['pinion_teeth'])!=(2.5,81,17):
        raise ValueError('final 2.5 x 81/17 gearing required')
    if drive['motor']['gear_ratio']!=2.5*81/17:raise ValueError('gear ratio/reference mismatch')
    if (encoder['counts_per_motor_revolution']!=28 or encoder['pulses_per_revolution_per_channel']!=7
            or encoder['quadrature_edges_per_pulse']!=4):raise ValueError('explicit final 7PPR x4 count contract required')
    if vehicle['identification_marker']['xyz_m'][2]!=.09:raise ValueError('final YAML marker .09 required; .14 is separate reproduction')
    return preview_parameters(vehicle)


def mask_from_profile(vehicle,sensor,*,marker_z=None):
    marker=vehicle['identification_marker'];xyz=list(marker['xyz_m'])
    if marker_z is not None:xyz[2]=marker_z
    _,text=mount('<sdf><model name="profile_only"><link name="chassis"/></model></sdf>',sensor,
        vehicle['drivetrain']['wheel_radius_m'],vehicle['body']['height_m'],xyz,
        marker['printed_board_size_m'],marker['enabled'])
    return json.loads(text)


def build(vehicle,sensor,declared):
    a=assumptions(declared);geometry=final_vehicle(vehicle,sensor)
    if sensor['lidar']['known_structure_mask']!='structure_aware':raise ValueError('explicit final structure policy required')
    lidar_xyz=[finite(v,'lidar xyz') for v in sensor['lidar']['xyz_m']]
    if len(lidar_xyz)!=3:raise ValueError('lidar xyz required')
    mask=mask_from_profile(vehicle,sensor);reference=mask_from_profile(vehicle,sensor,marker_z=.14)
    if len(mask['sectors_deg'])!=4 or len(reference['sectors_deg'])!=5:raise ValueError('final/reproduction marker separation changed')
    # CAD bounds are in base XY. Masks operate on sensor ray angles, before motion projection.
    if a['lidar_yaw_rad']!=0:
        yaw=math.degrees(a['lidar_yaw_rad'])
        mask['sectors_deg']=[[math.remainder(center-yaw,360.),half] for center,half in mask['sectors_deg']]
    mask_json=json.dumps(mask,separators=(',',':'),allow_nan=False)
    rear=finite(sensor['lidar']['masked_rear_half_angle_deg'],'rear blind angle')
    if rear!=0.:raise ValueError('final section mask must not inherit legacy rear wedge')
    blind_parameter(rear,mask_json)
    drive=vehicle['drivetrain'];encoder=vehicle['sensors']['drive_feedback']['drive_motor_encoder']
    spec=MotorEncoderSpec(encoder['counts_per_motor_revolution'],drive['motor']['gear_ratio'],drive['wheel_radius_m'])
    motor=dict(schema='runtime_motor_feedback_v1',counts_per_motor_revolution=spec.counts_per_motor_revolution,
        gear_ratio=spec.gear_ratio,wheel_radius_m=spec.wheel_radius_m,encoder_sign=a['encoder_sign'],
        maximum_interval_ns=a['motor_maximum_interval_ns'],maximum_age_ns=a['motor_maximum_age_ns'],
        maximum_speed_mps=a['motor_maximum_speed_mps'])
    MotorFeedbackRuntime(spec,**{key:motor[key] for key in ('encoder_sign','maximum_interval_ns','maximum_age_ns','maximum_speed_mps')})
    shared=dict(execution_environment='hardware_runtime_candidate_v1',use_sim_time=False,
        lidar_compensation='both',lidar_input_representation='native_estimated_v1',
        motion_scan_timing_verified=False,motion_scan_time_policy='host_estimated_v1',
        motion_feedback_mode='drive_motor_shaft',motion_gear_ratio=spec.gear_ratio,motion_wheel_radius_m=spec.wheel_radius_m,
        motion_rear_axle_x_m=geometry['motion_rear_axle_x_m'],motion_lidar_x_m=lidar_xyz[0],motion_lidar_y_m=lidar_xyz[1],
        motion_lidar_yaw_rad=float(a['lidar_yaw_rad']),motion_imu_topic=a['imu_topic'],motion_imu_frame_id=a['imu_frame_id'],
        motion_imu_roll_rad=float(a['imu_roll_rad']),motion_imu_pitch_rad=float(a['imu_pitch_rad']),
        motion_max_motor_feedback_mps=float(motor['maximum_speed_mps']),
        motion_gyro_abs_limit_rad_s=math.radians(finite(sensor['imu']['gyro_full_scale_dps'],'gyro range')),
        wheelbase_m=geometry['wheelbase_m'],vehicle_length_m=geometry['vehicle_length_m'],vehicle_width_m=geometry['vehicle_width_m'],
        max_steering_angle_rad=geometry['max_steering_angle_rad'],rear_blind_half_angle_deg=rear,
        lidar_blind_sectors_json=mask_json,known_structure_policy='structure_aware_v1',partial_observation_policy='bounded_local_v4')
    profile={'local_pursuit':{'ros__parameters':{**shared,**geometry,'lidar_x_m':lidar_xyz[0]}},
             'lidar_safety':{'ros__parameters':dict(shared)}}
    registration=dict(schema='runtime_profile_registration_v1',status='geometry_time_overlay_only_not_complete_launch',
        assumed=a,cad_source_sha256=CAD_SHA,owned_ros_parameters=profile,expected_motor_config=motor,
        selected_marker=dict(source='vehicle_yaml',xyz_m=vehicle['identification_marker']['xyz_m'],mask_sector_count=4,
            installed_calibrated=False),
        reproduction_only=dict(marker_z_m=.14,mask_sector_count=5,mask=reference,applied_to_runtime=False),
        sensor_provenance=dict(camera=copy.deepcopy(sensor['camera']),lidar_status=sensor['lidar']['status'],
            lidar_xyz_m=lidar_xyz,imu_status=sensor['imu']['status'],mount=copy.deepcopy(sensor['mount_provenance'])),
        stream_contracts=dict(native_lidar=dict(topic=a['native_lidar_topic'],frame_id=a['native_lidar_frame_id'],
            time_basis='unverified_estimate',frame_checked_by_current_autonomy=False),
            imu=dict(topic=a['imu_topic'],frame_id=a['imu_frame_id'],time_contract_verified=False),
            camera=dict(topic=a['camera_topic'],frame_id=a['camera_frame_id'],exposure_time_verified=False),
            motor=dict(topic=MOTOR_TOPIC,joint=MOTOR_JOINT,frame_id=MOTOR_FRAME,time_contract_verified=False)),
        ros_remappings=dict(local_pursuit=[['/scan/native',a['native_lidar_topic']],['/camera/color/image_raw',a['camera_topic']]],
            lidar_safety=[['/scan/native',a['native_lidar_topic']]]),
        host_arguments=dict(motor_feedback_config='motor_feedback.json',imu_topic=a['imu_topic'],
            max_steering_rad=geometry['max_steering_angle_rad'],enable_drive_wire_default=False,
            max_speed_source='explicit r398 policy; never copied from vehicle simulation speed'),
        unresolved=['r398 algorithm/speed/start policy and final CLI merge',
            'runtime C1/RGB producer topics/domain and expected source frames; preview bridge is domain216-only',
            'native LiDAR frame admission has no current ROS parameter; frame/axis binding remains explicit preflight',
            'installed encoder sign/IMU axes/lidar yaw/camera extrinsics and clock assumptions unverified'],
        physical_actuation_verified=False,hardware_calibration_verified=False,unknown_filled_as_free=False)
    return profile,motor,mask,registration


def validate_final(profile,motor,registration):
    errors=[]
    if type(profile) is not dict or type(motor) is not dict:raise ValueError('explicit profile and motor objects required')
    # Generated registration is evidence, not a new source of calibration or defaults.
    # Its hash is checked by the CLI before an externally composed profile is compared.
    for node,group in registration['owned_ros_parameters'].items():
        actual=profile.get(node,{}).get('ros__parameters',{})
        for key,expected in group['ros__parameters'].items():
            if key not in actual or type(actual[key]) is not type(expected) or actual[key]!=expected:
                errors.append(node+'.'+key)
    expected=registration['expected_motor_config']
    if set(motor)!=set(expected):errors.append('motor.schema_keys')
    for key,value in expected.items():
        if key not in motor or type(motor[key]) is not type(value) or motor[key]!=value:errors.append('motor.'+key)
    if errors:raise ValueError('missing/mixed runtime contract: '+', '.join(errors))
    return dict(owned_parameters_match=True,complete_launch_verified=False,physical_actuation_verified=False)


def verify_manifest(directory):
    manifest=json.loads((directory/'output_manifest.json').read_text(encoding='utf-8'))
    expected={'runtime_geometry.yaml','motor_feedback.json','selected_mask.json','registration.json'}
    if type(manifest) is not dict or set(manifest)!=expected:raise ValueError('exact generated output manifest required')
    if any(digest(directory/name)!=value for name,value in manifest.items()):raise ValueError('generated output changed')


def generate(vehicle_path,sensor_path,assumption_path,output):
    if output.exists():raise ValueError('fresh output directory required')
    vehicle=read_yaml(vehicle_path)['vehicle'];sensor=read_yaml(sensor_path)
    sensor['lidar']['scan_plane_meshes']=[str((sensor_path.parent/path).resolve(strict=True)) for path in sensor['lidar']['scan_plane_meshes']]
    profile,motor,mask,registration=build(vehicle,sensor,read_yaml(assumption_path))
    inputs=[vehicle_path,sensor_path,assumption_path,*map(Path,sensor['lidar']['scan_plane_meshes'])]
    registration['input_sources']=[dict(path=str(path.resolve()),sha256=digest(path)) for path in inputs]
    validate_final(profile,motor,registration)
    output.mkdir(parents=True,exist_ok=False)
    files={'runtime_geometry.yaml':yaml.safe_dump(profile,sort_keys=False,allow_unicode=True),
        'motor_feedback.json':json_text(motor),'selected_mask.json':json_text(mask),
        'registration.json':json_text(registration)}
    for name,content in files.items():(output/name).write_text(content,encoding='utf-8')
    (output/'output_manifest.json').write_text(json_text({name:digest(output/name) for name in files}),encoding='utf-8')
    return dict(output=str(output),selected_marker_z_m=.09,sectors=4,complete_launch_verified=False)


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='action',required=True)
    gen=sub.add_parser('generate')
    for key in ('vehicle','sensor','assumptions','output'):gen.add_argument('--'+key,type=Path,required=True)
    check=sub.add_parser('check');check.add_argument('--directory',type=Path,required=True)
    check.add_argument('--profile',type=Path);check.add_argument('--motor-config',type=Path)
    args=p.parse_args()
    if args.action=='generate':result=generate(args.vehicle,args.sensor,args.assumptions,args.output)
    else:
        verify_manifest(args.directory)
        registration=json.loads((args.directory/'registration.json').read_text(encoding='utf-8'))
        profile=read_yaml(args.profile or args.directory/'runtime_geometry.yaml')
        motor=json.loads((args.motor_config or args.directory/'motor_feedback.json').read_text(encoding='utf-8'))
        result=validate_final(profile,motor,registration)
    print(json_text(result),end='')


if __name__=='__main__':main()
