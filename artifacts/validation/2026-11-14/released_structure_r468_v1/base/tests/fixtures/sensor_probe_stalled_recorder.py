"""시험 전용: close가 반환하지 않는 writer. 실제 전원/디스크 고장은 아님."""
import json
import os
from pathlib import Path
import threading
import time

import rosbag2_py

original = rosbag2_py.SequentialWriter


class StalledWriter:
    def __init__(self): self.inner = original()
    def open(self,*args): return self.inner.open(*args)
    def create_topic(self,*args): return self.inner.create_topic(*args)
    def write(self,*args): return self.inner.write(*args)
    def close(self):
        with Path(os.environ['ARENA_STALLED_CLOSE_EVENT']).open('x') as stream:
            json.dump(dict(pid=os.getpid(),entered_close_monotonic_ns=time.monotonic_ns()),stream)
        threading.Event().wait()


rosbag2_py.SequentialWriter = StalledWriter
from record_sensor_bag import main
raise SystemExit(main())
