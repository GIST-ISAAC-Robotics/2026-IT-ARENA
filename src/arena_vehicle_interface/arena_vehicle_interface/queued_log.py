"""텍스트 상태 로그의 호출자 조회/I/O를 제어 콜백 밖에서 수행한다."""
import queue
import threading


class QueuedLog:
    def __init__(self, write, capacity=256):
        if capacity < 1:
            raise ValueError('capacity must be positive')
        self.write = write
        self.queue = queue.Queue(maxsize=capacity)
        self.closed = threading.Event()
        self.accepted = self.written = self.dropped = self.failed = 0
        self.thread = threading.Thread(target=self._run, name='state_log_writer', daemon=True)
        self.thread.start()

    def submit(self, text):
        # ROS 제어/센서 상태를 worker에서 읽지 않고 완성된 문자열만 전달한다.
        if self.closed.is_set():
            self.dropped += 1
            return False
        try:
            self.queue.put_nowait(text)
        except queue.Full:
            self.dropped += 1
            return False
        self.accepted += 1
        return True

    def _run(self):
        while not self.closed.is_set() or not self.queue.empty():
            try:
                text = self.queue.get(timeout=.05)
            except queue.Empty:
                continue
            try:
                self.write(text)
                self.written += 1
            except Exception:
                # 진단 출력 실패로 안전 제어를 멈추지 않는다. 오류 수는 숨기지 않는다.
                self.failed += 1
            finally:
                self.queue.task_done()

    def snapshot(self):
        return dict(accepted=self.accepted, written=self.written, dropped=self.dropped,
                    failed=self.failed, pending=self.queue.unfinished_tasks,
                    worker_alive=self.thread.is_alive())

    def close(self, timeout=2.):
        self.closed.set()
        self.thread.join(timeout=timeout)
        return not self.thread.is_alive()
