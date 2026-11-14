import sys
from pathlib import Path
import threading

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src/arena_vehicle_interface'))
from arena_vehicle_interface.queued_log import QueuedLog


def test_preserves_order_and_drains_on_shutdown():
    lines = []
    sink = QueuedLog(lines.append)
    for i in range(20):
        assert sink.submit(str(i))
    assert sink.close()
    assert lines == [str(i) for i in range(20)]
    assert sink.snapshot() == dict(accepted=20, written=20, dropped=0, failed=0, pending=0, worker_alive=False)
    assert not sink.submit('after close')


def test_blocked_writer_does_not_block_submission_and_reports_overflow():
    entered, release = threading.Event(), threading.Event()
    def write(_):
        entered.set()
        release.wait(3.)
    sink = QueuedLog(write, capacity=1)
    try:
        assert sink.submit('first') and entered.wait(1.)
        assert sink.submit('second')
        assert not sink.submit('third')
        assert sink.snapshot()['dropped'] == 1
        assert not sink.close(timeout=.001)
    finally:
        release.set()
        assert sink.close()


def test_failed_write_reported_without_stopping_worker():
    def fail(_):
        raise OSError('output unavailable')
    sink = QueuedLog(fail)
    assert sink.submit('test')
    assert sink.close()
    assert sink.snapshot()['failed'] == 1
