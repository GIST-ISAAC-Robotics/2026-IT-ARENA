"""ROS 설치 없이 종료 순서와 opt-in executor 경계를 검사하는 단위 시험."""
import importlib.util
from pathlib import Path
import signal
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch


class LifecycleModesTest(unittest.TestCase):
    def exercise(self, mode, failure=None):
        events, handlers, calls = [], {}, []
        context = object()
        class Node:
            def __init__(self):
                self.context = context
            def declare_parameter(self, name, default):
                calls.append((name, default))
                return SimpleNamespace(value=mode)
            def destroy_node(self):
                events.append('destroy')
                if failure == 'destroy':
                    raise RuntimeError('destroy failed')
        node = Node()
        def spin():
            events.append('spin')
            if failure == 'callback':
                raise RuntimeError('callback failed')
            if events.count('spin') == 3:
                handlers[signal.SIGINT](signal.SIGINT, None)
            events.append('callback_returned')
        class Executor:
            def __init__(self, context):
                assert context is node.context
                events.append('executor')
            def add_node(self, actual):
                assert actual is node
                events.append('add')
            def spin_once(self, timeout_sec):
                assert timeout_sec == .1
                spin()
            def remove_node(self, actual):
                assert actual is node
                events.append('remove')
            def shutdown(self, timeout_sec):
                assert timeout_sec == 1.
                events.append('executor_shutdown')
        def legacy_spin(actual, timeout_sec):
            assert actual is node and timeout_sec == .1
            spin()
        rclpy = SimpleNamespace(init=Mock(), ok=lambda: True, spin_once=legacy_spin,
                                shutdown=lambda: events.append('ros_shutdown'))
        def factory():
            if failure == 'factory':
                raise RuntimeError('factory failed')
            return node
        def handler(signum, callback):
            old = handlers.get(signum, signal.SIG_DFL)
            handlers[signum] = callback
            return old
        modules = {'rclpy': rclpy,
            'rclpy.signals': SimpleNamespace(SignalHandlerOptions=SimpleNamespace(NO=0)),
            'rclpy.executors': SimpleNamespace(SingleThreadedExecutor=Executor)}
        source = Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface/arena_vehicle_interface/node_lifecycle.py'
        spec = importlib.util.spec_from_file_location('lifecycle_test_subject', source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, modules), patch.object(signal, 'signal', handler):
            spec.loader.exec_module(module)
            if failure:
                with self.assertRaisesRegex(RuntimeError, failure+' failed'):
                    module.run_node(factory)
            elif mode not in ('legacy', 'retained'):
                with self.assertRaisesRegex(ValueError, 'executor_mode'):
                    module.run_node(factory)
            else:
                module.run_node(factory)
        self.assertTrue(all(h == signal.SIG_DFL for h in handlers.values()))
        self.assertEqual(events[-1], 'ros_shutdown')
        if failure != 'factory':
            self.assertEqual(calls, [('executor_mode', 'legacy')])
            self.assertEqual(events.count('destroy'), 1)
        return events

    def test_legacy_keeps_default_and_destroys_after_callback(self):
        events = self.exercise('legacy')
        self.assertEqual(events, ['spin', 'callback_returned']*3+['destroy', 'ros_shutdown'])

    def test_retained_adds_once_and_cleans_executor_after_stop_output(self):
        events = self.exercise('retained')
        self.assertEqual(events, ['executor', 'add']+['spin', 'callback_returned']*3+
                         ['destroy', 'remove', 'executor_shutdown', 'ros_shutdown'])

    def test_callback_failure_is_not_hidden(self):
        for mode in ('legacy', 'retained'):
            with self.subTest(mode=mode):
                self.exercise(mode, 'callback')

    def test_destroy_failure_still_cleans_executor_and_context(self):
        events = self.exercise('retained', 'destroy')
        self.assertEqual(events[-4:], ['destroy', 'remove', 'executor_shutdown', 'ros_shutdown'])

    def test_factory_failure_still_cleans_context(self):
        self.assertEqual(self.exercise('retained', 'factory'), ['ros_shutdown'])

    def test_unknown_mode_rejected_before_spin(self):
        self.assertEqual(self.exercise('unknown'), ['destroy', 'ros_shutdown'])


if __name__ == '__main__':
    unittest.main()
