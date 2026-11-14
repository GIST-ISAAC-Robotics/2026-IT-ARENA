import hashlib
import json

import pytest

from arena_vehicle_interface.bag_contract import load_manifest, required_for, topics_for
from audit_sensor_probe import load_probe, read_json
from sensor_recording_profile import PROBE_SCHEMA, capture_complete, profile_metadata, select_profile

MODE = 'drive_motor_shaft'


def test_autonomy_defaults_keep_original_contract():
    p = select_profile('autonomy_sim', None, None, MODE)
    assert p['schema'] == 2 and p['use_sim_time'] is True
    assert p['topics'] == topics_for(MODE) and p['required'] == required_for(MODE)
    assert p['parameter_nodes'] == ('local_pursuit', 'lidar_safety')


@pytest.mark.parametrize('clock', ['simulation', 'system'])
@pytest.mark.parametrize('mode', ['drive_motor_shaft', 'rear_wheel_pair_legacy'])
def test_probe_explicit_clock_sensors_and_no_controllers(clock, mode):
    # Feedback topic is selected from the real contract below; no invented aliases.
    from arena_vehicle_interface.drive_feedback import TOPIC_BY_MODE
    topic = TOPIC_BY_MODE[mode]
    p = select_profile('sensor_probe', clock, ['/imu/data', topic], mode)
    assert p['schema'] == PROBE_SCHEMA and p['use_sim_time'] == (clock == 'simulation')
    assert p['parameter_nodes'] == () and p['required'] == {'/imu/data', topic}
    assert not profile_metadata(p)['controller_replay_eligible']
    assert not profile_metadata(p)['sensor_acquisition_time_verified']


@pytest.mark.parametrize('name,clock,topics', [
    ('sensor_probe', None, ['/imu/data']), ('sensor_probe', 'system', None),
    ('sensor_probe', 'system', []), ('sensor_probe', 'system', ['/imu/data']*2),
    ('sensor_probe', 'system', ['/drive']), ('sensor_probe', 'system', ['/drive/safe']),
    ('sensor_probe', 'system', ['/odom']), ('sensor_probe', 'system', ['/clock']),
    ('sensor_probe', 'system', ['/wheel_states']), ('sensor_probe', 'system', [True]),
    ('autonomy_sim', 'system', None), ('autonomy_sim', None, ['/imu/data']),
    ('unknown', 'system', ['/imu/data']),
])
def test_incompatible_or_unsafe_profile_rejected(name, clock, topics):
    with pytest.raises(ValueError):
        select_profile(name, clock, topics, MODE)


def test_sensor_capture_completion_is_not_controller_or_clock_approval():
    p = select_profile('sensor_probe', 'system', ['/imu/data'], MODE)
    good = dict(errors=[], dropped={}, regressions=0, parameters={}, stats={'/imu/data': {'count': 1}})
    assert capture_complete(p, **good)
    for change in (dict(errors=['failure']), dict(dropped={'/imu/data': 1}), dict(regressions=1),
                   dict(parameters={'local_pursuit': {}}), dict(stats={})):
        assert not capture_complete(p, **dict(good, **change))
    old = select_profile('autonomy_sim', None, None, MODE)
    assert not capture_complete(old, **good)


@pytest.fixture
def recording(tmp_path):
    p = select_profile('sensor_probe', 'system', ['/imu/data'], MODE)
    files = {'receipt.jsonl': b'placeholder', 'parameters.json': b'{}',
             'bag/metadata.yaml': b'placeholder', 'bag/data.mcap': b'placeholder'}
    for rel, data in files.items():
        path = tmp_path/rel
        path.parent.mkdir(exist_ok=True)
        path.write_bytes(data)
    manifest = dict(schema=PROBE_SCHEMA, complete=True, drive_feedback_mode=MODE, topics=p['topics'],
                    **profile_metadata(p), stats={'/imu/data': {'count': 1}}, queue_drops={}, error=[],
                    clock_regressions=0, files_sha256={k: hashlib.sha256(v).hexdigest() for k, v in files.items()})
    def save():
        (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    save()
    return tmp_path, manifest, save


def test_probe_manifest_is_not_accepted_by_old_controller_replay(recording):
    root, manifest, _ = recording
    assert load_probe(root) == manifest
    with pytest.raises(ValueError, match='unsupported'):
        load_manifest(root)


@pytest.mark.parametrize('key,value', [
    ('complete', False), ('schema', 2), ('recorder_use_sim_time', True),
    ('recorder_use_sim_time', 0), ('sensor_acquisition_time_verified', True),
    ('controller_replay_eligible', True), ('recording_profile', 'autonomy_sim'),
    ('queue_drops', {'/imu/data': 1}), ('clock_regressions', 1), ('error', ['error']),
    ('stats', {'/imu/data': {'count': True}}), ('stats', {'/imu/data': {'count': 0}}),
    ('topics', {'/drive': 'ackermann_msgs/msg/AckermannDriveStamped'}),
    ('topics', {'/imu/data': 'std_msgs/msg/String'}),
])
def test_manifest_rejection(recording, key, value):
    root, manifest, save = recording
    manifest[key] = value
    save()
    with pytest.raises(ValueError):
        load_probe(root)


@pytest.mark.parametrize('path', ['../outside', 'C:/outside', '/outside', 'bag\\outside'])
def test_inventory_cannot_escape(recording, path):
    root, manifest, save = recording
    manifest['files_sha256'][path] = '0'*64
    save()
    with pytest.raises(ValueError):
        load_probe(root)


@pytest.mark.parametrize('change', ['missing', 'changed', 'extra_mcap', 'controller_parameters'])
def test_file_integrity_and_capture_boundary(recording, change):
    root, manifest, save = recording
    if change == 'missing':
        del manifest['files_sha256']['receipt.jsonl']
    elif change == 'changed':
        (root/'receipt.jsonl').write_bytes(b'changed')
    elif change == 'extra_mcap':
        (root/'bag/extra.mcap').write_bytes(b'extra')
    else:
        data = b'{"local_pursuit": {}}'
        (root/'parameters.json').write_bytes(data)
        manifest['files_sha256']['parameters.json'] = hashlib.sha256(data).hexdigest()
    save()
    with pytest.raises(ValueError):
        load_probe(root)


def test_json_reader_has_size_bound(tmp_path):
    path = tmp_path/'oversize.json'
    path.write_bytes(b' '*101)
    with pytest.raises(ValueError, match='bounded'):
        read_json(path, max_bytes=100)
