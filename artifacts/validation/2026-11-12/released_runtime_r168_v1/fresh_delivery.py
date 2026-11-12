"""시뮬 분석용: 이미 묵은 회전보다 다음 새 회전에 계산 시작을 맞춘다."""
import math
from arena_autonomy.decision_delivery import DecisionDelivery


class FreshStartDelivery(DecisionDelivery):
    # 50Hz 수신 콜백 한 주기+5ms; 관측/명령 TTL이나 출력 간격이 아니다.
    completion_window_s=.025

    def offer(self,snapshot):
        raw=snapshot['scan']
        if raw.get('representation')!='native_measurements_v1':
            raise ValueError('native scan required for this scheduling experiment')
        stamps=raw['measurements']['acquired_at_s']
        if not stamps or any(not isinstance(t,(int,float)) or isinstance(t,bool) or not math.isfinite(t) for t in stamps):
            raise ValueError('finite acquisition times required')
        age=snapshot['reference_s']-max(stamps)
        if age<0 or age>self.completion_window_s+1e-9:
            return False
        return super().offer(snapshot)
