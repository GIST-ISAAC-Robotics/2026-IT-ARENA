"""시험 전용: 실제 MCAP writer의 정리 호출/스레드 예외를 관찰한다."""
import json
import os
from pathlib import Path
import threading

import rosbag2_py

fault = os.environ['ARENA_WRITER_FAULT']
assert fault in ('normal', 'close', 'write_and_close')
events = Path(os.environ['ARENA_WRITER_EVENTS'])
original_writer = rosbag2_py.SequentialWriter
calls = []
uncaught = []
original_hook = threading.excepthook


def hook(args):
    uncaught.append(dict(thread=args.thread.name, error=repr(args.exc_value)))
    original_hook(args)


class FaultWriter:
    def __init__(self):
        self.inner = original_writer()
        self.writes = 0
        self.close_calls = 0

    def open(self, *args):
        return self.inner.open(*args)

    def create_topic(self, *args):
        return self.inner.create_topic(*args)

    def write(self, *args):
        if fault == 'write_and_close' and self.writes >= 3:
            raise OSError('injected_write_failure')
        self.inner.write(*args)
        self.writes += 1

    def close(self):
        self.close_calls += 1
        calls.append(dict(close_call=self.close_calls, writes=self.writes))
        if self.close_calls > 1:
            raise RuntimeError('injected_non_idempotent_close')
        # Real close first: this checks exception handling, not corrupt-file recovery.
        self.inner.close()
        if fault != 'normal':
            raise OSError('injected_close_failure')


threading.excepthook = hook
rosbag2_py.SequentialWriter = FaultWriter
from record_sensor_bag import main

try:
    raise SystemExit(main())
finally:
    with events.open('x') as stream:
        json.dump(dict(close_calls=calls, uncaught_thread_errors=uncaught), stream, indent=2)
