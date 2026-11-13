"""명시 선택형 새 native 회전 제출. 평가 지연이나 출력 허가를 만들지 않는다.

r114 prefix의 제출 함수를 이관한다. 25ms는 50Hz 수신부에서 새 계산을
시작할 창이며, 원 취득시각·센서/명령 TTL·명령 적용 간격은 그대로다.
"""
import math
from arena_autonomy.decision_delivery import DecisionDelivery


class FreshStartDelivery(DecisionDelivery):
    completion_window_s = .025

    def __init__(self, worker, *, retain_expired_reference=False, **kwargs):
        if type(retain_expired_reference) is not bool:
            raise ValueError('explicit expiry reference selection required')
        if retain_expired_reference and not callable(getattr(worker, 'mark_source_expiry', None)):
            raise ValueError('expiry reference requires a reason-aware worker')
        self.retain_expired_reference = retain_expired_reference
        super().__init__(worker, **kwargs)

    def poll(self, *args, **kwargs):
        item = super().poll(*args, **kwargs)
        if (self.retain_expired_reference and item and item.get('kind') == 'discarded'
                and item.get('reason') == 'decision_source_expired'):
            self.worker.mark_source_expiry()
        return item

    def offer(self, snapshot):
        raw = snapshot['scan']
        if raw.get('representation') != 'native_measurements_v1':
            raise ValueError('native scan required for this scheduling experiment')
        stamps = raw['measurements']['acquired_at_s']
        if not stamps or any(not isinstance(t, (int, float)) or isinstance(t, bool)
                             or not math.isfinite(t) for t in stamps):
            raise ValueError('finite acquisition times required')
        age = snapshot['reference_s']-max(stamps)
        if age < 0 or age > self.completion_window_s+1e-9:
            return False
        return super().offer(snapshot)
