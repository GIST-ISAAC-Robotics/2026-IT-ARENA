"""프로필/출처의 제한 정합 검사. ROS/장치나 주행 계산을 실행하지 않는다."""
from dataclasses import asdict
import json

import pytest

from profile_support import PREPARED, R399, load_json, profile, verify_manifest
from prepare_profile import recorded_header


@pytest.fixture(scope='module', autouse=True)
def frozen():
    before = verify_manifest()
    yield
    assert verify_manifest() == before


def test_r399_owned_fields_preserved_in_both_nodes():
    prepared = profile()
    owned = load_json(R399 / 'registration.json')['owned_ros_parameters']
    assert set(prepared) == {'local_pursuit', 'lidar_safety'}
    for node, content in owned.items():
        actual = prepared[node]['ros__parameters']
        for name, value in content['ros__parameters'].items():
            assert actual[name] == value, (node, name)
        assert actual['motion_scan_timing_verified'] is False
        assert actual['motion_scan_time_policy'] == 'host_estimated_v1'
        assert actual['motion_gear_ratio'] == 2.5 * 81 / 17


def test_header_delta_is_only_final_tire_geometry_and_selected_marker_mask():
    old = recorded_header()
    expected = load_json(PREPARED / 'expected_header.json')
    assert expected['execution_config'] == old['execution_config']
    motion = dict(expected['motion_config'])
    assert motion.pop('lidar_frame_id') == 'c1_unmounted_preview'
    assert motion == old['motion_config']
    before, after = old['follow_config'], dict(expected['follow_config'])
    assert {k for k in after if k not in before} == {'track_m', 'wheel_radius_m', 'wheel_width_m'}
    assert (after.pop('track_m'), after.pop('wheel_radius_m'), after.pop('wheel_width_m')) == (.1239, .0325, .0261)
    new_mask = json.loads(after['known_structure_sectors_json'])
    old_mask = json.loads(before['known_structure_sectors_json'])
    assert new_mask['sectors_deg'] == old_mask['sectors_deg'][:4]
    assert new_mask['range_limits_m'] == old_mask['range_limits_m'][:4]
    assert old_mask['sectors_deg'][4] == [180., 9.591829893301503]
    after['known_structure_sectors_json'] = before['known_structure_sectors_json']
    assert after == before


def test_expected_header_uses_current_typed_configs_without_hidden_defaults():
    from arena_autonomy.local_follow import FollowConfig
    from arena_autonomy.bypass_handoff import HandoffExecutionConfig
    from arena_autonomy.lidar_motion import MotionConfig
    expected = load_json(PREPARED / 'expected_header.json')
    assert asdict(FollowConfig(**expected['follow_config'])) == expected['follow_config']
    assert asdict(HandoffExecutionConfig.from_mapping(expected['execution_config'])) == expected['execution_config']
    assert asdict(MotionConfig(**expected['motion_config'])) == expected['motion_config']


def test_every_header_field_has_an_explicit_mapping_or_checked_constructor_default():
    expected = load_json(PREPARED / 'expected_header.json')
    mapping = load_json(PREPARED / 'parameter_mapping.json')
    groups = {'follow_config': expected['follow_config'],
              'execution_config': {k: v for k, v in expected['execution_config'].items() if k != 'bypass'},
              'execution_config.bypass': expected['execution_config']['bypass']}
    local = profile()['local_pursuit']['ros__parameters']
    for group, values in groups.items():
        records = [row for row in mapping if row['group'] == group]
        assert len(records) == len(values) and {row['field'] for row in records} == set(values)
        for row in records:
            assert row['value'] == values[row['field']]
            if row['parameter'] and row['field'] != 'max_speed_mps':
                assert local[row['parameter']] == row['value']


def test_preserved_speed_and_guard_policy_are_not_motor_admission_limits():
    p = profile(); local = p['local_pursuit']['ros__parameters']; guard = p['lidar_safety']['ros__parameters']
    assert (local['max_speed_mps'], local['g1_open_route_speed_mps'], local['min_speed_mps'],
            local['acceleration_mps2']) == (1.5, 1.5, .20, 1.)
    assert (local['control_rate_hz'], local['scan_timeout_s'], local['image_timeout_s'],
            local['sensor_wall_timeout_s'], guard['sensor_wall_timeout_s']) == (50., .45, 1., 30., 30.)
    assert local['race_control_mode'] == 'legacy'
    assert not local['development_speed_limit_enabled']
    for name in ('partial_observation_policy', 'known_structure_policy', 'lidar_blind_sectors_json',
                 'signed_motion_policy', 'g1_oblique_gap_trial', 'g1_traffic_recovery_trial',
                 'g1_recent_observation_enabled'):
        assert local[name] == guard[name]
    assert local['motion_max_motor_feedback_mps'] == guard['motion_max_motor_feedback_mps'] == 5.


def test_explicit_runtime_selection_keeps_common_default_and_wire_disabled():
    from arena_autonomy.runtime_node_backend import POLICY, validate_selection
    local = profile()['local_pursuit']['ros__parameters']
    assert local['g1_runtime_policy'] == POLICY
    options = dict(execution=local['g1_decision_execution'], submission=local['g1_submission_policy'],
        recheck_in_worker=local['g1_current_recheck_execution'] == 'worker_event_v1',
        retain_reference=local['g1_expiry_route_reference_trial'],
        numeric_execution=local['g1_numeric_execution'], hardware_shadow=False)
    assert validate_selection(POLICY, **options) and not validate_selection('off', **options)
    bindings = load_json(PREPARED / 'bindings.json')
    assert bindings['runtime_selection']['common_default'] == 'off'
    assert bindings['runtime_selection']['submission_receipt'] == 'runtime_dual_latest_v1'
    assert bindings['host_arguments']['enable_drive_wire'] is False
    assert bindings['stream_contracts']['native_lidar']['frame_id'] == 'c1_unmounted_preview'
    assert bindings['selected_motion_lidar_frame_id'] == 'c1_unmounted_preview'
    assert not bindings['source_frame_admission_pending'] and bindings['b3_operational_binding_pending']
    assert not any('enable_drive_wire' in content['ros__parameters'] for content in profile().values())
