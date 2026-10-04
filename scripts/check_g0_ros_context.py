#!/usr/bin/env python3
"""G0 격리 Context/명시 executor의 수명만 검사. Gazebo·구동 출력 없음."""
import json
import time


def main():
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    context = Context()
    executor, node = None, None
    ticks = []
    try:
        rclpy.init(context=context, signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('g0_context_contract_probe', context=context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        node.create_timer(.01, lambda: ticks.append(time.monotonic()))
        deadline = time.monotonic()+2.
        while len(ticks) < 3 and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        passed = len(ticks) >= 3
        print(json.dumps({'context_executor_passed': passed, 'timer_ticks': len(ticks), 'drive_output_topics': []}))
        return 0 if passed else 1
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown(timeout_sec=2.)
        if node is not None:
            node.destroy_node()
        context.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
