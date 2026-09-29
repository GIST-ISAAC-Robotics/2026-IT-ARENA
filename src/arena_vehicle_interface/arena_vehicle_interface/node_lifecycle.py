"""콜백이 반환된 뒤 ROS 노드를 정리하는 공통 실행 수명 관리."""

import signal

import rclpy
from rclpy.signals import SignalHandlerOptions


def run_node(factory, args=None):
    # 메시지 수신의 C++/Python 변환 도중 SIGINT가 예외나 context 종료를
    # 일으키지 않게 합니다. 종료 신호는 다음 spin 경계에서 처리합니다.
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    previous_handlers = {}
    node = None
    executor = None
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_stop)
        node = factory()
        mode = str(node.declare_parameter('executor_mode', 'legacy').value)
        if mode not in ('legacy', 'retained'):
            raise ValueError('executor_mode must be legacy or retained')
        # 아직 기본 동작은 바꾸지 않는다. 명시적 A/B 시험에서만 노드 등록을 유지한다.
        if mode == 'retained':
            from rclpy.executors import SingleThreadedExecutor
            executor = SingleThreadedExecutor(context=node.context)
            executor.add_node(node)
        while rclpy.ok() and not stop_requested:
            if executor is None:
                rclpy.spin_once(node, timeout_sec=0.1)
            else:
                executor.spin_once(timeout_sec=0.1)
    finally:
        try:
            try:
                # 콜백이 반환된 뒤, ROS context가 살아 있을 때 노드의 정지 출력을 보낸다.
                if node is not None:
                    node.destroy_node()
            finally:
                try:
                    if executor is not None:
                        executor.remove_node(node)
                        executor.shutdown(timeout_sec=1.)
                finally:
                    if rclpy.ok():
                        rclpy.shutdown()
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
