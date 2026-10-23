from types import SimpleNamespace

import pytest

import sensor_probe_preflight as tool
from test_rosbag_writer_compat import HumbleMetadata, JazzyMetadata


@pytest.fixture
def environment(tmp_path):
    class Writer:
        def __init__(self):
            pytest.fail('preflight must not create a writer')
        def open(self):
            pytest.fail('no open')
        write = create_topic = close = open
    def forbidden(*args, **kwargs):
        pytest.fail('no ROS initialization or node')
    modules = {
        'rclpy': SimpleNamespace(init=forbidden, create_node=forbidden),
        'rclpy.serialization': SimpleNamespace(serialize_message=lambda msg: b'cdr',
                                                deserialize_message=lambda payload, cls: cls()),
        'rosbag2_py': SimpleNamespace(SequentialWriter=Writer, TopicMetadata=JazzyMetadata,
                                     get_registered_writers=lambda: {'mcap'},
                                     get_registered_readers=lambda: {'mcap'}),
        'rosidl_runtime_py.utilities': SimpleNamespace(get_message=lambda name: dict),
        'yaml': SimpleNamespace(),
    }
    def run(**kwargs):
        kwargs.setdefault('capture', tmp_path/'artifacts/uncreated/sensor_recording')
        kwargs.setdefault('clock_mode', 'system')
        kwargs.setdefault('topics', ['/imu/data', '/drive_motor/encoder'])
        kwargs.setdefault('disk_usage', lambda path: SimpleNamespace(free=11*1024**3))
        kwargs.setdefault('version', (3, 12, 3))
        return tool.inspect_environment(**kwargs, root=tmp_path, loader=modules.__getitem__)
    return modules, run, tmp_path


@pytest.mark.parametrize('metadata', [HumbleMetadata, JazzyMetadata])
def test_available_api_is_not_hardware_ready_and_does_not_initialize(environment, metadata):
    modules, run, tmp_path = environment
    modules['rosbag2_py'].TopicMetadata = metadata
    before = set(tmp_path.rglob('*'))
    result = run()
    assert result['checks_passed']
    assert set(tmp_path.rglob('*')) == before
    assert not any(result[k] for k in ('ros_node_started', 'device_opened', 'writer_opened',
                                      'mcap_write_verified', 'dds_delivery_verified',
                                      'sensor_time_verified', 'hardware_verified', 'ready_to_drive'))
    assert set(result['topics']) == {'/imu/data', '/drive_motor/encoder'}


@pytest.mark.parametrize('missing', ['rclpy', 'rclpy.serialization', 'rosbag2_py', 'rosidl_runtime_py.utilities', 'yaml'])
def test_missing_import_reported_without_install(environment, missing):
    modules, _, root = environment
    def loader(name):
        if name == missing:
            raise ModuleNotFoundError('fixture')
        return modules[name]
    r = tool.inspect_environment(root/'artifacts/new', 'system', ['/scan'], root=root,
                                 loader=loader, disk_usage=lambda path: SimpleNamespace(free=11*1024**3))
    assert not r['checks_passed']
    assert not r['checks']['import:'+missing]['ok']
    assert not r['packages_installed']


@pytest.mark.parametrize('change', ['no_close', 'no_query', 'no_mcap', 'broken_metadata', 'broken_serialization'])
def test_binding_plugin_or_serialization_failure_does_not_silently_fallback(environment, change):
    modules, run, _ = environment
    bag = modules['rosbag2_py']
    if change == 'no_close':
        bag.SequentialWriter.close = None
    elif change == 'no_query':
        bag.get_registered_readers = None
    elif change == 'no_mcap':
        bag.get_registered_writers = lambda: {'sqlite3'}
    elif change == 'broken_metadata':
        bag.TopicMetadata = lambda **args: SimpleNamespace(name='wrong')
    else:
        modules['rclpy.serialization'].deserialize_message = lambda *args: {'unexpected': True}
    assert not run()['checks_passed']


@pytest.mark.parametrize('free', [0, 10*1024**3-1, 10*1024**3])
def test_disk_bound_matches_recorder(environment, free):
    _, run, _ = environment
    result = run(disk_usage=lambda path: SimpleNamespace(free=free))
    assert result['checks']['free_space']['ok'] is (free >= 10*1024**3)


def test_existing_directory_and_outside_artifacts_not_allowed(environment):
    _, run, root = environment
    path = root/'artifacts/existing'
    path.mkdir(parents=True)
    assert not run(capture=path)['checks_passed']
    assert not run(capture=root/'outside')['checks_passed']
    assert not run(capture=root/'artifacts')['checks_passed']


def test_python_version_and_invalid_sensor_selection(environment):
    _, run, _ = environment
    assert not run(version=(3, 9, 0))['checks_passed']
    for topics in (['/drive'], ['/wheel_states'], ['/imu/data', '/imu/data']):
        with pytest.raises(ValueError):
            run(topics=topics)


def test_existing_report_not_overwritten_or_probed(tmp_path, monkeypatch):
    output = tmp_path/'existing.json'
    output.write_text('keep')
    monkeypatch.setattr(tool, 'inspect_environment', lambda *args: pytest.fail('no probing'))
    with pytest.raises(FileExistsError):
        tool.main(['--capture', str(tmp_path/'artifacts/new'), '--clock-mode', 'system',
                   '--sensor-topic', '/scan', '--output', str(output)])
    assert output.read_text() == 'keep'
