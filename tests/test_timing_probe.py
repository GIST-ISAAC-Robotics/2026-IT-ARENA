"""계측 비활성 동일성, 예외 전파, 메시지 비변형 및 시각 종류 구분."""
from pathlib import Path
import sys
from types import SimpleNamespace
import json
import pytest
pytest.importorskip('std_msgs')
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.timing_probe import TimingProbe


def fake_node(enabled):
    sent = []
    node = SimpleNamespace(declare_parameter=lambda *a: SimpleNamespace(value=enabled),
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
