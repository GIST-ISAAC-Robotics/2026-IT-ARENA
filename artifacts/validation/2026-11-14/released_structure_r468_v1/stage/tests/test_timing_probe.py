"""계측 비활성 동일성, 예외 전파, 메시지 비변형 및 시각 종류 구분."""
from pathlib import Path
import sys
from types import SimpleNamespace
import json
import pytest
pytest.importorskip('std_msgs')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.timing_probe import TimingProbe


def fake_node(enabled, **parameters):
    sent = []
    node = SimpleNamespace(declare_parameter=lambda name, default: SimpleNamespace(
        value=enabled if name == 'timing_probe' else parameters.get(name, default)),
        create_publisher=lambda *a: SimpleNamespace(publish=sent.append),
        create_service=lambda *a: SimpleNamespace(callback=a[2]),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2_000_000_000)),
        get_name=lambda: 'test')
    return node, sent


def test_disabled_is_exact_original_callable():
    node, sent = fake_node(False)
    probe = TimingProbe(node)
    callback = lambda message: message
    assert probe.wrap('test', callback) is callback
    assert not sent
    assert probe.snapshot_service is None


def test_message_return_and_header_unchanged():
    node, sent = fake_node(True)
    probe = TimingProbe(node)
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=500_000_000)))
    wrapped = probe.wrap('image', lambda m: m)
    assert wrapped(msg) is msg
    assert msg.header.stamp.sec == 1
    result = json.loads(sent[0].data)
    assert result['source_age_ms'] == 500
    assert result['duration_ms'] >= 0
    assert result['wall_interval_ms'] is None
    assert result['sequence'] == 1
    wrapped(msg)
    assert json.loads(sent[-1].data)['wall_interval_ms'] >= 0
    assert json.loads(sent[-1].data)['sequence'] == 2


def test_exception_is_recorded_and_not_swallowed():
    node, sent = fake_node(True)
    def fail():
        raise ValueError('original')
    with pytest.raises(ValueError, match='original'):
        TimingProbe(node).wrap('control', fail)()
    assert len(sent) == 1
    assert json.loads(sent[0].data)['source_ns'] is None


def test_snapshot_counts_survive_diagnostic_log_loss():
    node, sent = fake_node(True)
    probe = TimingProbe(node)
    for _ in range(5):
        probe.wrap('imu', lambda: None)()
    probe.wrap('wheels', lambda: None)()
    sent.clear()
    response = probe.snapshot(None, SimpleNamespace())
    assert response.success
    assert json.loads(response.message) == {'node': 'test', 'completed': {'imu': 5, 'wheels': 1}}


def test_stage_preserves_values_exceptions_and_disabled_behavior():
    for enabled in (False, True):
        node, sent = fake_node(enabled)
        probe = TimingProbe(node)
        value = object()
        def callback(msg):
            assert probe.call('one', lambda v: v, msg) is value
            def fail():
                raise ValueError('stage failed')
            with pytest.raises(ValueError, match='stage failed'):
                probe.call('failure', fail)
            return msg
        assert probe.wrap('test', callback)(value) is value
        assert probe.active_stages is None
        if enabled:
            stages = json.loads(sent[-1].data)['stages']
            assert set(stages) == {'one', 'failure'}
            assert all(s['count'] == 1 and s['duration_ms'] >= 0 for s in stages.values())
        else:
            assert not sent


def test_buffered_has_no_publication_or_file_until_export(tmp_path):
    path = tmp_path/'timing.jsonl'
    node, sent = fake_node(True, timing_probe_mode='buffered', timing_probe_buffer_path=str(path))
    probe = TimingProbe(node)
    msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=1, nanosec=3)))
    def callback(value):
        return probe.call('pass', lambda x: x, value)
    wrapped = probe.wrap('imu', callback)
    assert wrapped(msg) is msg
    assert not sent and not path.exists()
    response = probe.export(None, SimpleNamespace())
    assert response.success
    metadata = json.loads(response.message)
    assert metadata['completed'] == {'imu': 1}
    assert metadata['timing_export']['dropped'] == 0
    row = json.loads(path.read_text())
    assert row['source_ns'] == 1000000003 and row['sequence'] == 1
    assert row['stages']['pass']['count'] == 1
    assert not probe.records
    wrapped(msg)
    assert probe.completed['imu'] == 2 and not probe.records and not sent
    assert not probe.export(None, SimpleNamespace()).success


def test_buffer_overflow_and_export_collision_fail_closed(tmp_path):
    path = tmp_path/'timing.jsonl'
    node, sent = fake_node(True, timing_probe_mode='buffered', timing_probe_capacity=1,
                           timing_probe_buffer_path=str(path))
    probe = TimingProbe(node)
    for _ in range(3):
        probe.wrap('imu', lambda: None)()
    assert len(probe.records) == 1 and probe.dropped == 2 and not sent
    response = probe.export(None, SimpleNamespace())
    assert not response.success
    assert json.loads(response.message)['completed']['imu'] == 3
    prior = path.read_bytes()
    other = TimingProbe(node)
    assert not other.export(None, SimpleNamespace()).success
    assert path.read_bytes() == prior


@pytest.mark.parametrize('params', [dict(timing_probe_mode='unknown'),
    dict(timing_probe_capacity=0), dict(timing_probe_capacity=100001),
    dict(timing_probe_capacity=True), dict(timing_probe_mode='buffered')])
def test_rejects_invalid_buffer_configuration(params):
    node, _ = fake_node(True, **params)
    with pytest.raises(ValueError):
        TimingProbe(node)


def test_buffered_exception_preserves_original_and_records(tmp_path):
    node, sent = fake_node(True, timing_probe_mode='buffered',
                           timing_probe_buffer_path=str(tmp_path/'failed.jsonl'))
    probe = TimingProbe(node)
    def fail():
        raise ValueError('original')
    with pytest.raises(ValueError, match='original'):
        probe.wrap('control', fail)()
    assert probe.completed == {'control': 1} and len(probe.records) == 1 and not sent
