import importlib.util
from pathlib import Path
import signal
import subprocess
from types import SimpleNamespace


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
