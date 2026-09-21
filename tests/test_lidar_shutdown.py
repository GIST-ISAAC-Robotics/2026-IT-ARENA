import importlib.util
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace

import pytest


REPO = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("lidar_shutdown", REPO / "scripts/lidar_shutdown.py")
shutdown = importlib.util.module_from_spec(spec)
spec.loader.exec_module(shutdown)


def test_sigint_only_goes_to_launch(monkeypatch):
    calls = []
    process = SimpleNamespace(pid=42, poll=lambda: None,
        send_signal=lambda sig: calls.append(("parent", sig)), wait=lambda timeout: 0)
    monkeypatch.setattr(shutdown.os, "killpg", lambda pid, sig: calls.append(("group", sig)))
    assert shutdown.stop_launch(process) == {}
    assert calls == [("parent", signal.SIGINT)]


def test_timeout_escalates_only_after_parent_signal(monkeypatch):
    calls = []
    def wait(timeout):
        if timeout == 20:
            raise subprocess.TimeoutExpired("launch", timeout)
        return -15
    process = SimpleNamespace(pid=42, poll=lambda: None,
        send_signal=lambda sig: calls.append(("parent", sig)), wait=wait)
    monkeypatch.setattr(shutdown.os, "killpg", lambda pid, sig: calls.append(("group", sig)))
    assert shutdown.stop_launch(process)["forced_cleanup"] == "SIGTERM"
    assert calls == [("parent", signal.SIGINT), ("group", signal.SIGTERM)]


def test_parent_success_cannot_hide_child_failure():
    log = "\n".join([
        "[INFO] [child]: process started with pid [12]",
        "[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)",
        "[ERROR] [child]: process has died [pid 12, exit code -2, cmd 'example'].",
    ])
    audit = shutdown.audit_shutdown(log, 0)
    assert not audit["clean"]
    assert audit["failed_children"] == {"12": -2}
    assert audit["errors_before_shutdown_count"] == 0
    assert audit["errors_after_shutdown_count"] == 1


def test_missing_child_exit_or_empty_log_is_not_clean():
    assert not shutdown.audit_shutdown("", 0)["clean"]
    log = "[INFO] [child]: process started with pid [12]"
    assert shutdown.audit_shutdown(log, 0)["unaccounted_child_pids"] == ["12"]
    log += "\n[INFO] [child]: process has finished cleanly [pid 12]"
    assert shutdown.audit_shutdown(log, 0)["clean"]
    assert not shutdown.audit_shutdown(log, -15, "SIGTERM")["clean"]


def service_fixture(monkeypatch):
    partition = 'arena_autonomy_test_123'
    monkeypatch.setattr(shutdown.os, 'getpid', lambda: 123)
    monkeypatch.setenv('GZ_PARTITION', partition)
    process = SimpleNamespace(pid=42, poll=lambda: None)
    log = '[INFO] [gz-1]: process started with pid [43]'
    before = {42: {'ppid': 123, 'start': '10'}, 43: {'ppid': 42, 'start': '11'},
              44: {'ppid': 43, 'start': '12'}, 45: {'ppid': 42, 'start': '13'}}
    return partition, process, log, before


def test_service_waits_for_owned_server_and_not_ros_siblings(monkeypatch):
    partition, process, log, before = service_fixture(monkeypatch)
    samples = iter([before, {42: before[42], 45: before[45]}])
    monkeypatch.setattr(shutdown, 'group_processes', lambda pgid: next(samples))
    calls = []
    def run(command, **kwargs):
        calls.append(command)
        return SimpleNamespace(returncode=0, stdout='data: true\n', stderr='')
    monkeypatch.setattr(shutdown.subprocess, 'run', run)
    result = shutdown.stop_gazebo_service(process, log, partition)
    assert result['verified'] and result['server_processes_before'] == [43, 44]
    assert result['remaining_server_pids'] == []
    assert calls[0][-2:] == ['--req', 'stop: true']


@pytest.mark.parametrize('invalid', ['wrong_argument', 'wrong_environment', 'no_server', 'wrong_group'])
def test_service_cannot_stop_unverified_partition_or_unowned_server(monkeypatch, invalid):
    partition, process, log, before = service_fixture(monkeypatch)
    if invalid == 'wrong_argument':
        partition = 'user_gazebo'
    elif invalid == 'wrong_environment':
        monkeypatch.setenv('GZ_PARTITION', 'user_gazebo')
    elif invalid == 'no_server':
        log = ''
    else:
        before = {}
    monkeypatch.setattr(shutdown, 'group_processes', lambda pgid: before)
    monkeypatch.setattr(shutdown.subprocess, 'run', lambda *a, **kw: pytest.fail('unsafe service call'))
    assert not shutdown.stop_gazebo_service(process, log, partition)['verified']


@pytest.mark.parametrize('acknowledged', [True, False])
def test_service_acknowledgement_is_not_process_exit(monkeypatch, acknowledged):
    partition, process, log, before = service_fixture(monkeypatch)
    monkeypatch.setattr(shutdown, 'group_processes', lambda pgid: before)
    monkeypatch.setattr(shutdown.subprocess, 'run', lambda *a, **kw: SimpleNamespace(
        returncode=0, stdout=f'data: {str(acknowledged).lower()}\n', stderr=''))
    result = shutdown.stop_gazebo_service(process, log, partition, timeout_s=0)
    assert not result['verified'] and result['remaining_server_pids'] == [43, 44]


def test_service_timeout_is_recorded_for_fallback(monkeypatch):
    partition, process, log, before = service_fixture(monkeypatch)
    monkeypatch.setattr(shutdown, 'group_processes', lambda pgid: before)
    def run(*a, **kw):
        raise subprocess.TimeoutExpired('gz', 8)
    monkeypatch.setattr(shutdown.subprocess, 'run', run)
    result = shutdown.stop_gazebo_service(process, log, partition)
    assert not result['verified'] and 'error' in result


def test_reused_server_pid_does_not_count_as_owned_process(monkeypatch):
    partition, process, log, before = service_fixture(monkeypatch)
    samples = iter([before, {43: {'ppid': 42, 'start': 'new'}}])
    monkeypatch.setattr(shutdown, 'group_processes', lambda pgid: next(samples))
    monkeypatch.setattr(shutdown.subprocess, 'run', lambda *a, **kw: SimpleNamespace(
        returncode=0, stdout='data: true\n', stderr=''))
    assert shutdown.stop_gazebo_service(process, log, partition)['verified']
