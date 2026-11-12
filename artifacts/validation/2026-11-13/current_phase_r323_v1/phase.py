"""새 발행 이후 입력으로 보조 확인을 시작하는 저장 재생 후보. 기본/ROS 미연결."""
import math
from pathlib import Path
import sys
A=Path(__file__).resolve().parent
sys.path.insert(0,str(A.parent.parent/'2026-11-12/current_priority_r286_v1'))
from priority import CurrentPriorityProcess


class CurrentPhaseProcess(CurrentPriorityProcess):
    # 실행 선택을 위한 예산이다. 확인 결과의 실제 시한/기하 검사를 대체하지 않는다.
    retained_schedule_budget_s=.100

    def offer_next_current(self,snapshot):
        ref=snapshot['reference_s']
        if not math.isfinite(ref):raise ValueError('finite current reference required')
        # 방금 발행한 각도를 넣으면서 발행 전 기준시각을 다시 쓰지 않는다.
        # 새 명령 이후의 운동 이력까지 들어온 다음 확인을 시작한다.
        if ref<=self.last_application:return False
        if self.saved_plan is None and self.retained is not None:
            source=self.retained['output'][2]['normal'].get('path_source_s')
            if not isinstance(source,(int,float)) or not math.isfinite(source):return False
            if ref-source+self.retained_schedule_budget_s>=self.geometry_age_limit:
                return False
        return super().offer_next_current(snapshot)
