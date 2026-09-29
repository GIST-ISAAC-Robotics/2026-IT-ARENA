"""숫자 주소 선택은 운반 경로만 바꾸며 기존 호스트 키 검증을 유지한다."""
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest


@pytest.mark.parametrize('address', ['', '192.0.2.1', 'fe80::1%18', 'unverified.example'])
def test_numeric_transport_keeps_canonical_host_and_reject_policy(monkeypatch, address):
    events = {}
    class StopAtConnect(Exception):
        pass
    class Client:
        def load_host_keys(self, path):
            events['keys'] = path
        def set_missing_host_key_policy(self, policy):
            events['policy'] = policy
        def connect(self, host, **kwargs):
            events['host'], events['connect'] = host, kwargs
            raise StopAtConnect()
        def close(self):
            events['closed'] = True
    class RejectPolicy:
        pass
    monkeypatch.setitem(sys.modules, 'paramiko', SimpleNamespace(SSHClient=Client, RejectPolicy=RejectPolicy))
    spec = importlib.util.spec_from_file_location('jetson_access_tested',
        Path(__file__).resolve().parents[1]/'scripts/jetson_access.py')
    module = importlib.util.module_from_spec(spec)
    # 모듈의 로컬 의존성 경로 추가도 시험 뒤 복원한다.
    monkeypatch.setattr(sys, 'path', list(sys.path))
    spec.loader.exec_module(module)
    sock = SimpleNamespace(close=lambda: events.update(socket_closed=True))
    def connect_socket(endpoint, timeout):
        events['endpoint'] = endpoint
        assert timeout == 10
        return sock
    monkeypatch.setattr(module.socket, 'create_connection', connect_socket)
    request = dict(host='jetson-orin.local', port=22, user='test', password='test-only',
                   known_hosts='known_hosts.fixture', connect_address=address)
    monkeypatch.setattr(sys, 'stdin', io.StringIO(json.dumps(request)))
    with pytest.raises(ValueError if address == 'unverified.example' else StopAtConnect):
        module.main()
    assert events['keys'] == 'known_hosts.fixture'
    assert isinstance(events['policy'], RejectPolicy) and events['closed']
    if address == 'unverified.example':
        assert 'endpoint' not in events and 'connect' not in events
    else:
        assert events['host'] == 'jetson-orin.local'
        assert events['connect']['sock'] is (sock if address else None)
        if address:
            assert events['endpoint'] == (address, 22) and events['socket_closed']
