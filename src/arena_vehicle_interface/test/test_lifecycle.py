from unittest.mock import Mock
import signal

import pytest

from arena_vehicle_interface import node_lifecycle


def test_stop_signal_finishes_spin_before_cleanup(monkeypatch):
    handlers = {}
    events = []

    def replace_handler(signum, callback):
        previous = handlers.get(signum, signal.SIG_DFL)
        handlers[signum] = callback
        return previous

    def spin(_node, timeout_sec):
        events.append("spin")
        handlers[signal.SIGINT](signal.SIGINT, None)
        events.append("callback_returned")

    node = Mock()
    node.destroy_node.side_effect = lambda: events.append("destroy")
    monkeypatch.setattr(node_lifecycle.signal, "signal", replace_handler)
    monkeypatch.setattr(node_lifecycle.rclpy, "init", Mock())
    monkeypatch.setattr(node_lifecycle.rclpy, "ok", lambda: True)
    monkeypatch.setattr(node_lifecycle.rclpy, "spin_once", spin)
    monkeypatch.setattr(node_lifecycle.rclpy, "shutdown", lambda: events.append("shutdown"))
    node_lifecycle.run_node(lambda: node)
    assert events == ["spin", "callback_returned", "destroy", "shutdown"]
    assert all(handler == signal.SIG_DFL for handler in handlers.values())


def test_real_callback_error_is_not_hidden(monkeypatch):
    node = Mock()
    shutdown = Mock()
    monkeypatch.setattr(node_lifecycle.signal, "signal", lambda *_: signal.SIG_DFL)
    monkeypatch.setattr(node_lifecycle.rclpy, "init", Mock())
    monkeypatch.setattr(node_lifecycle.rclpy, "ok", lambda: True)
    monkeypatch.setattr(node_lifecycle.rclpy, "spin_once", Mock(side_effect=RuntimeError("actual fault")))
    monkeypatch.setattr(node_lifecycle.rclpy, "shutdown", shutdown)
    with pytest.raises(RuntimeError, match="actual fault"):
        node_lifecycle.run_node(lambda: node)
    node.destroy_node.assert_called_once()
    shutdown.assert_called_once()
