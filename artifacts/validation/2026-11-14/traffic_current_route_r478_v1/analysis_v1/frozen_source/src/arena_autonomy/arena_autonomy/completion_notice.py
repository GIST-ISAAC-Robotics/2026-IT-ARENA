"""계산하지 않는 완료 알림 스레드. 수신/공유 버퍼 처리는 ROS 소유자만 한다."""
import threading


class CompletionNotice:
    def __init__(self, connection, trigger):
        self.connection, self.trigger = connection, trigger
        self.stopping = threading.Event()
        self.consumed = threading.Event()
        self.fault = None
        self.notifications = 0
        self.thread = threading.Thread(target=self._run, name='arena-result-notice', daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stopping.is_set():
                if not self.connection.poll(.05):
                    continue
                # trigger보다 먼저 clear한다. 콜백이 즉시 실행돼도 ACK를 잃지 않는다.
                self.consumed.clear()
                self.notifications += 1
                self.trigger()
                while not self.stopping.is_set() and not self.consumed.wait(.05):
                    pass
        except Exception as error:
            if not self.stopping.is_set():
                self.fault = type(error).__name__+': '+str(error)

    def acknowledge(self):
        self.consumed.set()

    def close(self):
        self.stopping.set()
        self.consumed.set()
        self.thread.join(timeout=1.)
        return dict(alive=self.thread.is_alive(), fault=self.fault, notifications=self.notifications)
