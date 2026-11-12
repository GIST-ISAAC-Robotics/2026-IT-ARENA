"""r250의 단계 지연을 유지하되 완료 가능 시각에 한 번 알린다. 평가 전용."""
import importlib.util
from pathlib import Path

source=Path(__file__).resolve().parents[1]/'measured_stages_runtime_r250_v1/staged_delay.py'
spec=importlib.util.spec_from_file_location('r250_staged_base',source)
base=importlib.util.module_from_spec(spec);spec.loader.exec_module(base)


class EventStagedDelay(base.StagedDelay):
    def __init__(self,worker,clock,plan_s,recheck_s,arm_wakeup,cancel_wakeup):
        super().__init__(worker,clock,plan_s,recheck_s)
        self.arm_wakeup=arm_wakeup;self.cancel_wakeup=cancel_wakeup
        self.wake_deadline=None

    def _cancel(self):
        if self.wake_deadline is not None:self.cancel_wakeup()
        self.wake_deadline=None

    def notified(self):
        # 타이머는 이 함수 전에 취소된다. 남은 지연이 있으면 다음 pump가 재예약한다.
        self.wake_deadline=None

    def pump(self):
        result=super().pump()
        if result is not None or self.current is None:
            self._cancel()
        elif self.held is not None:
            reference,stage,_=self.current
            deadline=reference+self.delays[stage]
            if self.wake_deadline!=deadline:
                self._cancel()
                self.wake_deadline=deadline
                self.arm_wakeup(max(1e-6,deadline-self.clock()))
        return result

    def invalidate(self):
        self._cancel();super().invalidate()

    def close(self):
        self._cancel();return super().close()
