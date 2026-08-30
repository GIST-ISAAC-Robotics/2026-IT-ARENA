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
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, request_stop)
        node = factory()
        while rclpy.ok() and not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        finally:
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
