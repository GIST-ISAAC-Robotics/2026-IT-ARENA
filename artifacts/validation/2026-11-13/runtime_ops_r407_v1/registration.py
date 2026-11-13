"""r401의 기하/시각/원 frame을 유지하는 B3 운영 선택만 검사한다."""
import copy
import json
from pathlib import Path

CHANGES={
    'local_pursuit':{'race_control_mode':'b3_candidate',
        'development_speed_limit_enabled':True,'development_speed_limit_mps':.3},
    'lidar_safety':{'race_control_mode':'b3_candidate'},
}
DOMAIN=219


def derive(original):
    selected=copy.deepcopy(original)
    for node,values in CHANGES.items():selected[node]['ros__parameters'].update(values)
    return selected


def validate(original,selected,bindings):
    assert selected==derive(original),'only registered B3/development-limit differences allowed'
    for node in CHANGES:
        p=selected[node]['ros__parameters']
        assert p['execution_environment']=='hardware_runtime_candidate_v1'
        assert p['use_sim_time'] is False and p['motion_scan_timing_verified'] is False
        assert p['motion_scan_time_policy']=='host_estimated_v1'
        assert p['motion_lidar_frame_id']=='c1_unmounted_preview'
    p=selected['local_pursuit']['ros__parameters']
    assert p['vehicle_track_width_m']==.1239 and p['vehicle_wheel_width_m']==.0261
    assert p['vehicle_wheel_radius_m']==.0325 and p['motion_gear_ratio']==2.5*81/17
    assert p['max_steering_angle_rad']==.4578436017036438
    assert p['g1_runtime_policy']=='cad_runtime_r377_v1'
    assert p.get('g1_current_corner_budget','off')=='off'
    assert bindings['host_arguments']['enable_drive_wire'] is False
    assert bindings['host_arguments']['max_speed_mps']==p['max_speed_mps']==1.5
    assert bindings['host_arguments']['max_steering_rad']==p['max_steering_angle_rad']
    return dict(allowed_parameter_changes=copy.deepcopy(CHANGES),geometry_time_frame_unchanged=True,
        operational_enable_drive_wire=False,pty_test_override_only=True,synthetic_encoder_ros_publisher=False)


def read(root):
    import yaml
    root=Path(root)
    original=yaml.safe_load((root/'reference/r401_runtime_profile.yaml').read_bytes())
    selected=yaml.safe_load((root/'prepared/runtime_profile.yaml').read_bytes())
    bindings=json.loads((root/'prepared/bindings.json').read_bytes())
    assert (root/'prepared/bindings.json').read_bytes()==(root/'reference/r401_bindings.json').read_bytes()
    return selected,bindings,validate(original,selected,bindings)
