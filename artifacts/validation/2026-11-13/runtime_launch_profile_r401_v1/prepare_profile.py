"""저장 header + r399 overlay 합성. ROS/제어기/장치 import나 실행 없음."""
import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
R377 = HERE.parent / 'released_cad_drive_r377_v1'
R399 = HERE.parent / 'runtime_profile_r399_v1/generated_v1'

FOLLOW = {
    'signed_motion_policy': 'signed_motion_policy',
    'partial_observation_policy': 'partial_observation_policy',
    'known_structure_policy': 'known_structure_policy',
    'known_structure_sectors_json': 'lidar_blind_sectors_json',
    'wheelbase_m': 'wheelbase_m', 'length_m': 'vehicle_length_m', 'width_m': 'vehicle_width_m',
    'track_m': 'vehicle_track_width_m', 'wheel_radius_m': 'vehicle_wheel_radius_m',
    'wheel_width_m': 'vehicle_wheel_width_m', 'wall_offset_m': 'target_wall_distance_m',
    'max_steering_rad': 'max_steering_angle_rad', 'headway_s': 'g1_solo_headway_s',
    'lateral_acceleration_mps2': 'lateral_acceleration_limit_mps2',
    'current_lane_preference_trial': 'g1_g2_current_lane_preference_trial',
    'current_lane_headroom_mps': 'g1_g2_current_lane_headroom_mps',
    'side_departure_trial': 'g1_g2_side_departure_trial',
    'released_rear_sweep_trial': 'g1_g2_released_rear_sweep_trial',
    'static_side_release_trial': 'g1_g2_observed_motion_gates',
    **{name: 'g1_' + name for name in (
        'solo_route_only', 'corridor_arc_agreement', 'ray_wall_continuity', 'piecewise_wall_trial',
        'solo_straight_budget_trial', 'solo_corner_approach_trial', 'oblique_gap_trial',
        'parallel_hold_trial', 'pass_release_trial', 'guard_curve_trial', 'guard_observation_policy',
        'traffic_recovery_trial', 'released_body_gap_trial', 'traffic_path_recovery_trial',
        'rear_lane_keep_trial', 'rear_lane_keep_policy', 'corridor_fallback_enabled',
        'short_boundary_prefix_enabled', 'recent_observation_enabled', 'boundary_evidence_mode',
        'occupancy_timing_mode', 'road_half_m', 'open_route_speed_mps', 'blend_open_speed_by_length',
        'preview_turn_speed', 'preview_above_base_speed', 'boundary_bridge',
        'traffic_boundary_refresh_trial', 'low_speed_arc_reacquire', 'standing_wall_reacquire',
        'narrow_occluder_bridge', 'junction_continuity', 'expiry_route_reference_trial')},
}
EXECUTION = {
    'preparation_policy': 'g1_g2_preparation_policy',
    'persistence_window_policy': 'g1_g2_persistence_window',
    'follow_entry_space_enabled': 'g1_g2_follow_entry_space',
    'preparation_hold_policy': 'g1_g2_preparation_hold',
    'initial_reference_policy': 'g1_g2_initial_reference',
    'entry_motion_gate': 'g1_g2_observed_motion_gates',
    **{name: 'g1_g2_' + name for name in (
        'moving_follow_reserve', 'closing_follow_reserve', 'recent_forward_reserve',
        'low_motion_reserve', 'moving_pace_floor', 'continuous_forward_pace',
        'moving_pass_trial', 'moving_pass_profile', 'reacquire_policy', 'entry_policy',
        'motion_pause_policy', 'relative_yield_policy', 'stopped_entry_centering', 'adaptive_entry_preview')},
}
BYPASS = {'corridor_policy': 'g1_g2_corridor_policy',
          'steerable_transition': 'g1_g2_observed_motion_gates',
          'rear_wall_witness': 'g1_g2_rear_wall_witness'}


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def recorded_header():
    with (R377 / 'run1/follow_decisions.jsonl').open(encoding='utf8') as stream:
        row = json.loads(next(stream))
    assert row['kind'] == 'header' and row['data']['schema'] == 5
    return row['data']


def scalar_defaults(path, class_name):
    tree = ast.parse(path.read_text(encoding='utf8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    result = {}
    for item in cls.body:
        if isinstance(item, ast.AnnAssign):
            try:
                result[item.target.id] = ast.literal_eval(item.value)
            except ValueError:
                pass  # HandoffExecutionConfig.bypass는 별도 typed config.
    return result


def compose():
    header = recorded_header()
    manifest = json.loads((R399 / 'output_manifest.json').read_bytes())
    assert all(sha(R399 / name) == expected for name, expected in manifest.items())
    registration = json.loads((R399 / 'registration.json').read_bytes())
    profile = copy.deepcopy(registration['owned_ros_parameters'])
    local = profile['local_pursuit']['ros__parameters']
    guard = profile['lidar_safety']['ros__parameters']
    target = copy.deepcopy(header)
    target['follow_config'].update(track_m=local['vehicle_track_width_m'],
        wheel_radius_m=local['vehicle_wheel_radius_m'], wheel_width_m=local['vehicle_wheel_width_m'],
        known_structure_sectors_json=local['lidar_blind_sectors_json'])
    assert len(json.loads(header['follow_config']['known_structure_sectors_json'])['sectors_deg']) == 5
    assert len(json.loads(local['lidar_blind_sectors_json'])['sectors_deg']) == 4
    mappings = []

    def add(field, parameter, value, group):
        if parameter in local:
            assert local[parameter] == value, ('owned parameter conflict', parameter)
        local[parameter] = value
        mappings.append(dict(group=group, field=field, parameter=parameter, value=value))

    groups = [('follow_config', target['follow_config'], FOLLOW,
               'local_follow.py', 'FollowConfig', {'max_speed_mps'}),
              ('execution_config', target['execution_config'], EXECUTION,
               'bypass_handoff.py', 'HandoffExecutionConfig', {'bypass', 'observed_forward_pace'}),
              ('execution_config.bypass', target['execution_config']['bypass'], BYPASS,
               'bypass_entry_follow.py', 'BypassExecutionConfig', set())]
    for group, values, mapping, source, cls, derived in groups:
        for field, parameter in mapping.items():
            add(field, parameter, values[field], group)
        defaults = scalar_defaults(ROOT / 'src/arena_autonomy/arena_autonomy' / source, cls)
        for key, value in values.items():
            if key not in mapping and key not in derived:
                assert defaults[key] == value, ('unmapped constructor default differs', group, key)
                mappings.append(dict(group=group, field=key, parameter=None,
                                     value=value, source='unchanged constructor default'))
    assert target['execution_config']['observed_forward_pace'] == (
        local['g1_g2_observed_motion_gates'] and local['g1_g2_moving_pace_floor'])
    with zipfile.ZipFile(R377 / 'sources.zip') as archive:
        member = 'src/arena_bringup/launch/simulation.launch.py'
        frozen = archive.read(member)
        source = frozen.decode('utf8')
        assert '("g1_open_1p5",1.5)' in source and '"acceleration_mps2": 1.0' in source
        evidence = dict(archive='artifacts/validation/2026-11-13/released_cad_drive_r377_v1/sources.zip',
            archive_sha256=sha(R377 / 'sources.zip'), member=member,
            member_sha256=hashlib.sha256(frozen).hexdigest(), selected_speed_profile='g1_open_1p5',
            launch_command_sha256=sha(R377 / 'launch_command.json'))
    launch_args = json.loads((R377 / 'launch_command.json').read_bytes())
    assert 'speed_profile:=g1_open_1p5' in launch_args
    assert 'sensor_wall_timeout_s:=30.0' in launch_args and 'autonomy_control_rate_hz:=50.0' in launch_args
    local.update(g1_follow_enabled=True, g1_g2_simulation_trial=True,
        g1_runtime_policy='cad_runtime_r377_v1', g1_decision_execution='process_candidate',
        g1_submission_policy='fresh_native_v1', g1_current_path_recheck='selected_v1',
        g1_current_recheck_execution='worker_event_v1', g1_numeric_execution='native_v1',
        max_speed_mps=1.5, min_speed_mps=.20, acceleration_mps2=1.0, control_rate_hz=50.0,
        scan_timeout_s=.45, image_timeout_s=1.0, sensor_wall_timeout_s=30.0,
        vision_execution='process', start_signal_mode='official_4red_local_candidate',
        race_control_mode='legacy', development_speed_limit_enabled=False,
        g1_marker_late_start_trial=False, timing_probe=False)
    assert target['follow_config']['max_speed_mps'] == min(1., local['max_speed_mps'])
    assert local['max_speed_mps'] == target['follow_config']['open_route_speed_mps']
    mappings.extend([
        dict(group='follow_config', field='max_speed_mps', parameter='max_speed_mps',
             value=target['follow_config']['max_speed_mps'], source='min(1.0, ROS max_speed_mps=1.5)'),
        dict(group='execution_config', field='observed_forward_pace', parameter=None,
             value=target['execution_config']['observed_forward_pace'],
             source='g1_g2_observed_motion_gates AND g1_g2_moving_pace_floor')])
    for key, value in header['motion_config'].items():
        for owner in (local, guard):
            name = 'motion_' + key
            if name in owner:
                assert owner[name] == value, ('r399 motion differs from saved numbers', name)
            owner[name] = value
    # r403의 명시 frame 수용 계약. 원 MeasurementScan.source_frame_id를 바꾸지 않는다.
    selected_frame = registration['stream_contracts']['native_lidar']['frame_id']
    assert selected_frame == 'c1_unmounted_preview'
    local['motion_lidar_frame_id'] = guard['motion_lidar_frame_id'] = selected_frame
    target['motion_config']['lidar_frame_id'] = selected_frame
    guard.update(sensor_wall_timeout_s=30.0, signed_motion_policy=local['signed_motion_policy'],
        g1_oblique_gap_trial=local['g1_oblique_gap_trial'],
        g1_traffic_recovery_trial=local['g1_traffic_recovery_trial'],
        g1_recent_observation_enabled=local['g1_recent_observation_enabled'], timing_probe=False)
    bindings = dict(ros_remappings=registration['ros_remappings'], stream_contracts=registration['stream_contracts'],
        host_arguments=dict(motor_feedback_config=(R399 / 'motor_feedback.json').relative_to(ROOT).as_posix(),
            imu_topic=local['motion_imu_topic'], max_speed_mps=local['max_speed_mps'],
            max_steering_rad=local['max_steering_angle_rad'], enable_drive_wire=False),
        runtime_selection=dict(parameter='g1_runtime_policy', common_default='off',
            selected=local['g1_runtime_policy'], submission_receipt='runtime_dual_latest_v1',
            source_snapshot_latest=True, auto_arm=False, host_started=False, firmware_flash=False),
        source_frame_admission_pending=False, selected_motion_lidar_frame_id=selected_frame,
        runtime_producer_domain_topic_pending=False, producer_integration_tested=False,
        b3_operational_binding_pending=True, calibrated=False)
    return profile, target, mappings, bindings, evidence


def yaml_text(profile):
    lines = ['# r401 명시 비구동 준비. 기본 활성/실물 교정/host wire 허가가 아니다.']
    for node, content in profile.items():
        lines.extend([node + ':', '  ros__parameters:'])
        for key, value in content['ros__parameters'].items():
            lines.append('    ' + key + ': ' + json.dumps(value, ensure_ascii=False, allow_nan=False))
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    assert output.is_relative_to(HERE) and not output.exists(), 'new owned output directory required'
    profile, expected, mapping, bindings, evidence = compose()
    output.mkdir(parents=True)
    (output / 'runtime_profile.yaml').write_text(yaml_text(profile), encoding='utf8')
    for name, data in [('expected_header.json', expected), ('parameter_mapping.json', mapping),
                       ('bindings.json', bindings), ('source_evidence.json', evidence)]:
        (output / name).write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf8')
    print(json.dumps(dict(prepared=True, policy_executed=False, ros_started=False,
                         parameter_counts={k: len(v['ros__parameters']) for k, v in profile.items()})))


if __name__ == '__main__':
    main()
