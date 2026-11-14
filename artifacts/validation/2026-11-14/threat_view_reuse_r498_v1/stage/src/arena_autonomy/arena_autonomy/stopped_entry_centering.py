"""가까운 정지차 앞의 진입 준비용 조향 명령: 독립적인 속도 허가를 만들지 않는다."""
from bisect import bisect_right
import math


def stopped_shaft_window(history, now):
    """현재까지 .20초의 단일 축 속도가 0이고 누락·외삽이 기존 운동 계약 안인지 확인한다."""
    times, values = history.wheels.times, history.wheels.values
    if not math.isfinite(now) or len(times) < 2 or len(times) != len(values):
        return False
    # 미래 표본을 읽지 않고, 창 시작 직전 표본으로 왼쪽 경계까지 덮는다.
    lo = bisect_right(times, now - .20) - 1
    hi = bisect_right(times, now + 1e-10)
    if lo < 0 or hi-lo < 2 or now-times[hi-1] > history.config.max_extrapolation_s+1e-9:
        return False
    return (all(values[j] == 0. for j in range(lo, hi))
            and all(0 < times[j]-times[j-1] <= history.config.max_input_gap_s+1e-9
                    for j in range(lo+1, hi)))
