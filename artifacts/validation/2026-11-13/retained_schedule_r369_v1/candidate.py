"""선택형 RELEASED 보관 경로 검사 제출 억제. 점유/기한/새 계획의 의미는 바꾸지 않는다."""
from arena_autonomy.runtime_dual_state import RuntimeDualProcess


class ScheduledProcess(RuntimeDualProcess):
    def __init__(self, *args, avoid_superseded_retained=False, **kwargs):
        if type(avoid_superseded_retained) is not bool:
            raise ValueError('explicit boolean scheduling option required')
        self.avoid_superseded_retained = avoid_superseded_retained
        self.skipped_retained_submissions = 0
        super().__init__(*args, **kwargs)

    def offer_next_current(self, snapshot, *, now_s):
        retained = self.retained
        if (self.avoid_superseded_retained and not self.closed and not self.fault
                and self.saved_plan is None and self.plan.inflight is not None
                and self.current.inflight is None and self.ready is None
                and retained is not None and self._usable(retained)
                and retained['output'][2].get('normal',{}).get('selector_mode') == 'RELEASED'):
            # 이 분기는 자유 공간/새 증거를 만들지 않는다. 기존 검사를 새로 제출하지 않을 뿐이다.
            self.skipped_retained_submissions += 1
            return False
        return super().offer_next_current(snapshot, now_s=now_s)
