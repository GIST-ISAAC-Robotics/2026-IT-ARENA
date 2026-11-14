"""명시 실물 정책을 기존 설치 limits 생성기에 연결하는 파일 전용 stage helper."""
import argparse
import copy
import json
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = next(parent for parent in Path(__file__).resolve().parents
            if (parent/'scripts/runtime_installed_profile.py').is_file())
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'),
                str(ROOT/'src/arena_vehicle_interface')]
import yaml
import runtime_installed_limits as limits
import runtime_installed_profile as installed
from arena_autonomy.crop_edge_selection import estimated_environment, validate_policy_selection
from arena_autonomy.runtime_node_backend import validate_corner_selection, validate_selection
from arena_autonomy.race_runtime_policy.current_route_priority import enabled
from arena_autonomy.lidar_source_frame import registered_source_frame

COMMON = dict(execution_environment='hardware_runtime_candidate_v1', use_sim_time=False,
    lidar_compensation='both', lidar_input_representation='native_estimated_v1',
    motion_scan_timing_verified=False, motion_scan_time_policy='host_estimated_v1',
    g1_traffic_recovery_trial=True, g1_guard_observation_policy='selected_r181',
    partial_observation_policy='bounded_local_v4', g1_crop_edge_component_trial=True,
    g1_observation_input_scope='host_estimated_crop_v1', race_control_mode='b3_candidate')
PLANNER = dict(g1_follow_enabled=True, g1_g2_simulation_trial=True,
    g1_runtime_policy='cad_runtime_r377_v1', g1_decision_execution='process_candidate',
    g1_submission_policy='fresh_native_v1', g1_current_path_recheck='selected_v1',
    g1_current_recheck_execution='worker_event_v1', g1_numeric_execution='native_v1',
    g1_expiry_route_reference_trial=True, g1_current_corner_budget='r396_v1',
    g1_trace_detail='compact_v1')
# 정책만 선택한다. 기존 base의 추월 profile/속도/기하/측정값은 자동 채택하지 않는다.
PHYSICAL = ('motion_lidar_frame_id', 'motion_imu_frame_id', 'motion_lidar_x_m',
    'motion_lidar_y_m', 'motion_lidar_yaw_rad', 'motion_imu_roll_rad',
    'motion_imu_pitch_rad', 'motion_gyro_bias_rad_s')
HOST = ('runtime_port', 'clock_policy', 'motor_feedback_config')


def required_inputs():
    return dict(assembly_id=None, shared_ros_parameters={key:None for key in PHYSICAL},
                **{key:None for key in HOST})


def policy_overlay():
    return {node: {'ros__parameters': {**COMMON, **(PLANNER if node == 'local_pursuit' else {})}}
            for node in limits.NODES}


def check_policy(profile):
    for node in limits.NODES:
        p = profile[node]['ros__parameters']
        if any(type(p.get(k)) is not type(v) or p[k] != v for k,v in COMMON.items()):
            raise ValueError(node+' explicit shared policy mismatch')
        args = dict(execution_environment=p['execution_environment'], use_sim_time=p['use_sim_time'],
            motion_verified=p['motion_scan_timing_verified'], motion_time_policy=p['motion_scan_time_policy'],
            motion_mode=p['lidar_compensation'], lidar_input_representation=p['lidar_input_representation'])
        if not estimated_environment(**args):
            raise ValueError('physical estimated contract mismatch')
        validate_policy_selection(p['g1_crop_edge_component_trial'], p['partial_observation_policy'], p['g1_traffic_recovery_trial'])
        if not enabled(SimpleNamespace(traffic_recovery_trial=p['g1_traffic_recovery_trial'],
                                      guard_observation_policy=p['g1_guard_observation_policy'])):
            raise ValueError('current route policy not selected')
    p=profile['local_pursuit']['ros__parameters']
    if any(type(p.get(k)) is not type(v) or p[k] != v for k,v in PLANNER.items()):
        raise ValueError('planner runtime selection mismatch')
    validate_selection(p['g1_runtime_policy'], execution=p['g1_decision_execution'],
        submission=p['g1_submission_policy'], recheck_in_worker=True,
        retain_reference=p['g1_expiry_route_reference_trial'], numeric_execution=p['g1_numeric_execution'],
        hardware_shadow=False)
    validate_corner_selection(p['g1_current_corner_budget'], runtime_selected=True,
        observation_scope=p['g1_observation_input_scope'], **args)


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')


def draft(output):
    check_policy(policy_overlay())
    output.mkdir(parents=True, exist_ok=False)
    (output/'policy_overlay.yaml').write_text(yaml.safe_dump(policy_overlay(),sort_keys=False),encoding='utf-8')
    save_json(output/'required_inputs.json', required_inputs())
    save_json(output/'status.json', dict(policy_only=True, executable_runtime_profile=False,
        installed_profile=None, servo_neutral=None, mcu_maximum_speed_mps=None,
        source_frozen=False, installed=False, hardware_ready=False, device_io=False))


def prepare(base_path, inputs_path, installed_folder, output):
    base_raw=base_path.read_bytes()
    inputs_raw=inputs_path.read_bytes()
    base=limits.profile_values(base_raw)
    inputs=installed.read_json(inputs_path)
    if set(inputs) != set(required_inputs()) or set(inputs.get('shared_ros_parameters',{})) != set(PHYSICAL):
        raise ValueError('explicit hardware inputs fields required')
    if any(not isinstance(inputs.get(k),str) or not inputs[k].strip() for k in ('assembly_id',*HOST)):
        raise ValueError('assembly/port/clock/motor feedback paths are required; no defaults')
    physical=inputs['shared_ros_parameters']
    if any(value is None for value in physical.values()):
        raise ValueError('physical frame/mount/bias inputs remain pending')
    for key,value in physical.items():
        if key.endswith('frame_id'):
            if not isinstance(value,str) or not value.strip():raise ValueError('explicit frame required')
        else:
            installed.literal(value,'double')
    if not registered_source_frame(physical['motion_lidar_frame_id']):
        raise ValueError('existing registered lidar source frame required; no implicit frame rename')
    imu_frame = physical['motion_imu_frame_id']
    if len(imu_frame) > 256 or any(c.isspace() or not c.isprintable() for c in imu_frame):
        raise ValueError('existing IMU frame contract mismatch')
    bundle=installed.verify(installed_folder)
    if inputs['assembly_id'] != bundle['assembly_id']:
        raise ValueError('hardware inputs and installed bundle assembly differ')
    board=installed.read_json(installed_folder/'runtime_board.json')
    selected=copy.deepcopy(base)
    for node,section in policy_overlay().items():
        selected[node]['ros__parameters'].update(section['ros__parameters'], **physical)
    check_policy(selected)
    if base_raw != base_path.read_bytes() or inputs_raw != inputs_path.read_bytes():
        raise ValueError('input changed during selection')
    output.mkdir(parents=True,exist_ok=False)
    (output/'runtime_base.original.yaml').write_bytes(base_raw)
    (output/'hardware_inputs.original.json').write_bytes(inputs_raw)
    policy_path=output/'runtime_policy.yaml'
    policy_path.write_text(yaml.safe_dump(selected,sort_keys=False,allow_unicode=True),encoding='utf-8')
    limits.prepare(installed_folder,policy_path,output/'selected')
    limits.verify(output/'selected',installed_folder)
    save_json(output/'handover.json',dict(assembly_id=bundle['assembly_id'],
        required_host_inputs={k:inputs[k] for k in HOST},
        supplied_mcu_maximum_speed_mps=board['maximum_speed_mps'],
        moving_pass_selected=selected['local_pursuit']['ros__parameters'].get('g1_g2_moving_pass_trial'),
        moving_pass_profile=selected['local_pursuit']['ros__parameters'].get('g1_g2_moving_pass_profile'),
        trace_detail=selected['local_pursuit']['ros__parameters']['g1_trace_detail'],
        installed_bundle_verified=True, measured_inputs_independently_verified=False,
        runtime_launch_validated=False, installed=False, hardware_ready=False, nodes_started=False,
        device_io=False, firmware_written=False, speed_limits_propagated=False,
        base_sha256=limits.digest(base_raw), inputs_sha256=limits.digest(inputs_raw)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    commands=parser.add_subparsers(dest='command',required=True)
    d=commands.add_parser('draft');d.add_argument('--output',type=Path,required=True)
    p=commands.add_parser('prepare')
    p.add_argument('--base-params',type=Path,required=True)
    p.add_argument('--hardware-inputs',type=Path,required=True)
    p.add_argument('--installed-profile',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:
        if args.command=='draft':draft(args.output)
        else:prepare(args.base_params,args.hardware_inputs,args.installed_profile,args.output)
    except (ValueError,OSError,TypeError,KeyError,yaml.YAMLError) as exc:
        print(json.dumps(dict(error=str(exc),device_io=False)));return 2
    print(json.dumps(dict(output=str(args.output),installed=False,hardware_ready=False,device_io=False)))
    return 0


if __name__=='__main__':
    raise SystemExit(main())
