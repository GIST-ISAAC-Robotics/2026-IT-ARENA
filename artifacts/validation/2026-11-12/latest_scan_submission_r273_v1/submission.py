"""선택형 최신 미사용 입력 제출. 이후 현재 재검사/원 시간 예산은 별도 유지한다."""
import math
from arena_autonomy.decision_delivery import source_identity


def eligible(snapshot, *, now_s, last_reference_s, last_source, policy):
    if policy not in ('fresh', 'latest'):
        raise ValueError('explicit submission policy required')
    ref = snapshot['reference_s']
    if (not math.isfinite(now_s) or not math.isfinite(ref) or now_s < ref
            or math.isnan(last_reference_s) or ref-last_reference_s < .1-1e-9):
        return False
    raw = snapshot['scan']
    if raw.get('representation') != 'native_measurements_v1':
        raise ValueError('native scan required')
    stamps = raw['measurements']['acquired_at_s']
    if (not stamps or any(type(x) not in (int, float) or not math.isfinite(x) for x in stamps)
            or any(b <= a for a,b in zip(stamps,stamps[1:]))):
        raise ValueError('ordered finite original ray timestamps required')
    if stamps[-1] > ref or source_identity(snapshot) == last_source:
        return False
    if now_s-stamps[0] >= .3 or now_s-ref >= .2:
        return False
    return policy == 'latest' or now_s-stamps[-1] <= .025+1e-9
