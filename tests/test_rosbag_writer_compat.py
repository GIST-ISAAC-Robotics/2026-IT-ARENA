from types import SimpleNamespace

import pytest

from rosbag_writer_compat import check_writer_api, topic_metadata, close_writer


class HumbleMetadata:
    def __init__(self, name, type, serialization_format, offered_qos_profiles=''):
        self.name, self.type = name, type
        self.serialization_format, self.offered_qos_profiles = serialization_format, offered_qos_profiles


class JazzyMetadata:
    id = None
    def __init__(self, id, name, type, serialization_format):
        self.id, self.name, self.type = id, name, type
        self.serialization_format = serialization_format


def test_old_jazzy_only_call_fails_with_humble_signature():
    with pytest.raises(TypeError, match='id'):
        HumbleMetadata(id=1, name='/scan', type='sensor_msgs/msg/LaserScan', serialization_format='cdr')


@pytest.mark.parametrize('cls', [HumbleMetadata, JazzyMetadata])
def test_metadata_preserves_topic_type_and_cdr_without_blanket_fallback(cls):
    result = topic_metadata(SimpleNamespace(TopicMetadata=cls), 2, '/scan', 'sensor_msgs/msg/LaserScan')
    assert result.name == '/scan' and result.type == 'sensor_msgs/msg/LaserScan'
    assert result.serialization_format == 'cdr'
    if cls is JazzyMetadata:
        assert result.id == 2
    else:
        assert not hasattr(result, 'id') and result.offered_qos_profiles == ''


def test_unrelated_typeerror_is_not_retried():
    calls = []
    class Broken:
        id = None
        def __init__(self, **kwargs):
            calls.append(kwargs)
            raise TypeError('unrelated binding failure')
    with pytest.raises(TypeError, match='unrelated'):
        topic_metadata(SimpleNamespace(TopicMetadata=Broken), 2, '/scan', 'sensor_msgs/msg/LaserScan')
    assert len(calls) == 1 and calls[0]['id'] == 2


@pytest.mark.parametrize('value', [True, -1, 65536, 1.0])
def test_invalid_id_rejected(value):
    with pytest.raises(ValueError):
        topic_metadata(SimpleNamespace(TopicMetadata=HumbleMetadata), value, '/scan', 'sensor_msgs/msg/LaserScan')


@pytest.mark.parametrize('missing', ['open', 'write', 'create_topic', 'close'])
def test_writer_api_must_support_explicit_close(missing):
    cls = type('Writer', (), {n: lambda self: None for n in ('open', 'write', 'create_topic', 'close') if n != missing})
    with pytest.raises(RuntimeError, match=missing):
        check_writer_api(SimpleNamespace(SequentialWriter=cls))


def test_complete_writer_api():
    cls = type('Writer', (), {n: lambda self: None for n in ('open', 'write', 'create_topic', 'close')})
    check_writer_api(SimpleNamespace(SequentialWriter=cls))


def test_absent_writer_cleanup_preserves_startup_error():
    errors = ['startup error']
    assert close_writer(None, errors)
    assert errors == ['startup error']


def test_successful_close_is_once_without_fabricated_error():
    calls, errors = [], []
    writer = SimpleNamespace(close=lambda: calls.append('close'))
    assert close_writer(writer, errors)
    assert calls == ['close'] and errors == []


@pytest.mark.parametrize('prior', [[], ['write failed'], ['startup failed', 'another error']])
@pytest.mark.parametrize('exception', [OSError('storage'), RuntimeError('already closed'), TypeError('binding')])
def test_close_error_not_retried_and_prior_error_not_lost(prior, exception):
    calls, errors = [], list(prior)
    def fail():
        calls.append('close')
        raise exception
    assert not close_writer(SimpleNamespace(close=fail), errors)
    assert calls == ['close']
    assert errors == prior+['writer close failed: '+repr(exception)]


@pytest.mark.parametrize('exception', [KeyboardInterrupt(), SystemExit(9)])
def test_process_control_baseexception_not_swallowed(exception):
    errors = []
    def fail():
        raise exception
    with pytest.raises(type(exception)):
        close_writer(SimpleNamespace(close=fail), errors)
    assert errors == []
