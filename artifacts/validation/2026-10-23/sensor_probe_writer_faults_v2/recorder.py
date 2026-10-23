"""실제 MCAP writer에 시험용 오류를 주입하고 기록기 종료 코드를 전달한다."""
import os
import rosbag2_py

original_writer = rosbag2_py.SequentialWriter
fault = os.environ['ARENA_SENSOR_PROBE_WRITER_FAULT']
assert fault in ('write', 'close')


class FaultWriter:
    def __init__(self):
        self.inner = original_writer()
        self.writes = 0
        self.closed = False

    def open(self, *args):
        return self.inner.open(*args)

    def create_topic(self, *args):
        return self.inner.create_topic(*args)

    def write(self, *args):
        if fault == 'write' and self.writes >= 3:
            raise OSError('injected_writer_write_failure')
        self.writes += 1
        return self.inner.write(*args)

    def close(self):
        if self.closed:
            return
        self.inner.close()
        self.closed = True
        if fault == 'close':
            raise OSError('injected_writer_close_failure')


rosbag2_py.SequentialWriter = FaultWriter
from record_sensor_bag import main
raise SystemExit(main())
