"""기준 시각/원천 시각의 남은 예산을 제출 전에 비교한다. 발행 TTL은 그대로다."""
import math
from pathlib import Path
import sys
A=Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent/'current_phase_r323_v1'))
from phase import CurrentPhaseProcess
from arena_autonomy.decision_delivery import source_identity


class CurrentDeadlineProcess(CurrentPhaseProcess):
    current_schedule_budget_s=.100

    def offer_next_current(self,snapshot,*,now_s):
        ref=snapshot['reference_s'];first=source_identity(snapshot)[1]
        if not all(math.isfinite(v) for v in (now_s,ref,first)) or now_s<ref:return False
        # 예측은 일을 시작할지에만 사용한다. 실제 완료/적용 검사는 부모 그대로다.
        if now_s-ref+self.current_schedule_budget_s>=.2:return False
        if now_s-first+self.current_schedule_budget_s>=.3:return False
        return super().offer_next_current(snapshot)
