"""ROS 없이 기록 파일 무결성 및 재생 발행 경계를 검사한다."""
import hashlib
import json
import math
from pathlib import Path
import sys

import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.bag_contract import (
    INPUTS, OBSERVATIONS, TOPICS, REQUIRED, load_manifest, replay_topic, sha256_file,
    update_digest, quantiles, eof_stopped, compare_commands, dds_metadata,
)


@pytest.fixture
def recording(tmp_path):
    (tmp_path/'bag').mkdir()
    for path in ('receipt.jsonl', 'parameters.json', 'bag/metadata.yaml', 'bag/data.mcap'):
        (tmp_path/path).write_bytes(b'test')
    manifest = dict(schema=1, complete=True, topics=TOPICS,
        stats={t: dict(count=1) for t in REQUIRED},
        files_sha256={str(p.relative_to(tmp_path)): sha256_file(p)
                      for p in tmp_path.rglob('*') if p.is_file()})
    def save():
        (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    save()
    return tmp_path, manifest, save


def test_manifest_inventory(recording):
    root, manifest, _ = recording
    assert load_manifest(root) == manifest


def test_dds_metadata_accepts_mapping_and_attributes():
    from types import SimpleNamespace
    values = dict(source_timestamp=12, received_timestamp=15, publication_sequence_number=3,
                  reception_sequence_number=2, publisher_gid=bytes([1, 2]))
    assert dds_metadata(values) == dds_metadata(SimpleNamespace(**values))
    assert dds_metadata(values)['publisher_gid'] == '0102'
    assert dds_metadata({})['publication_sequence_number'] is None


@pytest.mark.parametrize('field,value', [('complete', False), ('schema', 2), ('schema', 3),
    ('drive_feedback_mode', 'drive_motor_shaft'), ('queue_drops', {'/scan': 1}),
    ('error', ['disk failure']), ('clock_regressions', 1)])
def test_reject_partial_and_loss(recording, field, value):
    root, manifest, save = recording
    manifest[field] = value
    save()
    with pytest.raises(ValueError):
        load_manifest(root)


def test_corrupt_bytes_rejected(recording):
    root, _, _ = recording
    (root/'bag/data.mcap').write_bytes(b'changed')
    with pytest.raises(ValueError, match='integrity'):
        load_manifest(root)


def test_path_escape_rejected(recording, tmp_path):
    root, manifest, save = recording
    manifest['files_sha256']['../outside.mcap'] = '00'
    save()
    with pytest.raises(ValueError, match='integrity'):
        load_manifest(root)


@pytest.mark.parametrize('path', ['receipt.jsonl', 'parameters.json', 'bag/metadata.yaml', 'bag/data.mcap'])
def test_missing_inventory_rejected(recording, path):
    root, manifest, save = recording
    del manifest['files_sha256'][path]
    save()
    with pytest.raises(ValueError):
        load_manifest(root)


def test_missing_sensor_rejected(recording):
    root, manifest, save = recording
    manifest['stats']['/scan']['count'] = 0
    save()
    with pytest.raises(ValueError, match='Required topics'):
        load_manifest(root)


@pytest.mark.parametrize('topic', [*OBSERVATIONS, '/cmd_vel', '/odom', '/clock', '/ground_truth'])
def test_commands_truth_and_clock_never_sensor_replayed(topic):
    with pytest.raises(ValueError):
        replay_topic(topic)


@pytest.mark.parametrize('topic', INPUTS)
def test_only_sensors_are_namespaced(topic):
    assert replay_topic(topic) == '/replay'+topic


def test_topic_contract_not_extensible_through_manifest(recording):
    root, manifest, save = recording
    manifest['topics'] = dict(TOPICS, **{'/odom': 'nav_msgs/msg/Odometry'})
    save()
    with pytest.raises(ValueError, match='contract'):
        load_manifest(root)


def test_digest_includes_stamp_length_and_order():
    def digest(rows):
        h = hashlib.sha256()
        for data, stamp in rows:
            update_digest(h, data, stamp)
        return h.hexdigest()
    assert len({digest(rows) for rows in [
        [(b'a', 1), (b'bc', 2)], [(b'ab', 1), (b'c', 2)],
        [(b'a', 2), (b'bc', 2)], [(b'bc', 2), (b'a', 1)]]}) == 4


def test_quantiles_empty_singleton_and_order():
    assert quantiles([])['p95'] is None
    assert quantiles([4]) == dict(count=1, p50=4, p95=4, p99=4, max=4)
    assert quantiles(list(reversed(range(101))))['p95'] == 95


def test_eof_stop_requires_new_tail_commands():
    assert not eof_stopped([[1_000_000_000, 0., 0.]]*20, 1_000_000_000)
    tail = [[2_250_000_000+i*20_000_000, 0., 0.] for i in range(10)]
    assert eof_stopped(tail, 1_000_000_000)
    tail[-1][1] = .1
    assert not eof_stopped(tail, 1_000_000_000)
    tail[-1][1] = math.nan
    assert not eof_stopped(tail, 1_000_000_000)


def test_command_comparison_excludes_eof_and_prestart():
    result = compare_commands([[10, 1., .2], [20, 2., .4]],
        [[5, 100., 1.], [15, 1.5, .3], [25, 0., 0.]], 20)
    assert result['samples'] == 1
    assert result['speed_absolute_error_mps']['max'] == .5


from arena_vehicle_interface.bag_contract import (  # noqa: E402
    inputs_for, topics_for, required_for, manifest_feedback_mode, resolve_replay_feedback,
)

LEGACY_PARAMS = {'lidar_compensation': 'both', 'motion_imu_topic': '/imu/data', 'motion_wheel_radius_m': .025}
MOTOR_PARAMS = {'lidar_compensation': 'both', 'motion_imu_topic': '/imu/data', 'motion_feedback_mode': 'drive_motor_shaft',
                'motion_wheel_radius_m': .0325, 'motion_gear_ratio': 15.}


def test_motor_contract_has_single_feedback_topic_and_no_wheel_pair():
    inputs = inputs_for('drive_motor_shaft')
    assert inputs['/drive_motor/encoder'] == 'sensor_msgs/msg/JointState'
    assert '/wheel_states' not in inputs and '/wheel_states' not in topics_for('drive_motor_shaft')
    assert '/drive_motor/encoder' in required_for('drive_motor_shaft')
    assert INPUTS == inputs_for('rear_wheel_pair_legacy') and '/drive_motor/encoder' not in TOPICS
    with pytest.raises(ValueError):
        inputs_for('configured')
    with pytest.raises(ValueError):
        replay_topic('/drive_motor/encoder')  # legacy 기본 계약에서는 허용하지 않음
    assert replay_topic('/drive_motor/encoder', 'drive_motor_shaft') == '/replay/drive_motor/encoder'


def test_schema2_motor_manifest_loads_with_declared_mode(recording):
    root, manifest, save = recording
    manifest.update(schema=2, drive_feedback_mode='drive_motor_shaft', topics=topics_for('drive_motor_shaft'),
                    stats={t: dict(count=1) for t in required_for('drive_motor_shaft')})
    save()
    assert manifest_feedback_mode(load_manifest(root)) == 'drive_motor_shaft'
    manifest['topics'] = TOPICS  # 모드와 다른 토픽 계약
    save()
    with pytest.raises(ValueError, match='Topic contract'):
        load_manifest(root)


def test_schema_mode_rules():
    assert manifest_feedback_mode({'schema': 1}) == 'rear_wheel_pair_legacy'
    for bad in ({'schema': 1, 'drive_feedback_mode': 'rear_wheel_pair_legacy'}, {'schema': 2},
                {'schema': 2, 'drive_feedback_mode': 'configured'}, {'schema': 3, 'drive_feedback_mode': 'drive_motor_shaft'}):
        with pytest.raises(ValueError):
            manifest_feedback_mode(bad)


def test_old_recording_replays_with_recorded_legacy_geometry_not_current_yaml():
    feedback, params = resolve_replay_feedback({'schema': 1}, {'local_pursuit': dict(LEGACY_PARAMS),
                                                               'lidar_safety': dict(LEGACY_PARAMS)})
    assert feedback == dict(mode='rear_wheel_pair_legacy', topic='/wheel_states', wheel_radius_m=.025, gear_ratio=None,
                            geometry_source='recorded_parameters', current_vehicle_yaml_used=False)
    assert all(p['motion_feedback_mode'] == 'rear_wheel_pair_legacy' and p['motion_wheel_radius_m'] == .025
               for p in params.values())
    missing = {k: v for k, v in LEGACY_PARAMS.items() if k != 'motion_wheel_radius_m'}
    feedback, params = resolve_replay_feedback({'schema': 1}, {'a': dict(missing), 'b': dict(missing)})
    assert feedback['geometry_source'] == 'legacy_controller_default' and feedback['wheel_radius_m'] == .025


def test_motor_recording_requires_recorded_ratio_and_matching_mode():
    manifest = {'schema': 2, 'drive_feedback_mode': 'drive_motor_shaft'}
    feedback, params = resolve_replay_feedback(manifest, {'a': dict(MOTOR_PARAMS), 'b': dict(MOTOR_PARAMS)})
    assert feedback['topic'] == '/drive_motor/encoder' and feedback['gear_ratio'] == 15.
    bad_sets = [
        {**MOTOR_PARAMS, 'motion_gear_ratio': None},
        {k: v for k, v in MOTOR_PARAMS.items() if k != 'motion_gear_ratio'},
        {k: v for k, v in MOTOR_PARAMS.items() if k != 'motion_wheel_radius_m'},
        {k: v for k, v in MOTOR_PARAMS.items() if k != 'motion_feedback_mode'},
        {**MOTOR_PARAMS, 'motion_feedback_mode': 'rear_wheel_pair_legacy'},
        {**MOTOR_PARAMS, 'motion_gear_ratio': float('nan')},
        {**MOTOR_PARAMS, 'motion_gear_ratio': 0},
    ]
    for bad in bad_sets:
        with pytest.raises(ValueError):
            resolve_replay_feedback(manifest, {'a': dict(MOTOR_PARAMS), 'b': bad})
    with pytest.raises(ValueError):
        resolve_replay_feedback({'schema': 1}, {'a': dict(MOTOR_PARAMS), 'b': dict(MOTOR_PARAMS)})
    with pytest.raises(ValueError, match='disagree'):
        resolve_replay_feedback(manifest, {'a': dict(MOTOR_PARAMS), 'b': {**MOTOR_PARAMS, 'motion_gear_ratio': 12.}})


def test_replay_and_record_scripts_do_not_read_current_vehicle_yaml():
    scripts = Path(__file__).resolve().parents[1] / 'scripts'
    for name in ('replay_sensor_bag.py', 'audit_sensor_bag.py'):
        source = (scripts / name).read_text(encoding='utf-8')
        assert 'vehicle.yaml' not in source and 'vehicle_config' not in source
    replay = (scripts / 'replay_sensor_bag.py').read_text(encoding='utf-8')
    assert 'resolve_replay_feedback(' in replay and "'/wheel_states'" not in replay
