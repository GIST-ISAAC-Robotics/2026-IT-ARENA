"""선택형 시뮬 ROS의 두 계산 슬롯. 기본 실행/실물에는 연결하지 않는다."""
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
sys.path.insert(0, str(A.parent/'current_deceleration_r334_v1'))
from deceleration import CurrentDecelerationProcess
from ros_delivery import base


class Worker(CurrentDecelerationProcess):
    def __init__(self, header, *, clock, delays, wake):
        super().__init__(header, work_timeout=3.)
        self.connection = base.BothConnections(self.plan.connection, self.current.connection)
        self.plan = base.DelayedLane(self.plan, clock, delays[0], wake)
        self.current = base.DelayedLane(self.current, clock, delays[1], wake)
        self.cleanup = None

    def close(self):
        rows = super().close()
        if rows is None:
            return None
        self.cleanup = dict(forced=any(r['forced'] for r in rows),
                            exitcode=next((r['exitcode'] for r in rows if r['exitcode'] != 0), 0),
                            lanes=rows)
        return self.cleanup
