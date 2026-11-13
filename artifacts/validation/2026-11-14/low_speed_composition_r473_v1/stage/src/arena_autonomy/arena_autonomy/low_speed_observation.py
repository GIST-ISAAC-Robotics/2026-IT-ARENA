"""저속 목표와 양자화된 실측 입구를 구분한다. 실제 운동값은 수정하지 않는다."""
import math


def within_scope(speed, target, cap, headroom=0.):
    """명시된 한 카운트 여유만 입구에 적용한다. 제동/영역 계산은 호출자 실제 속도다."""
    values = speed, target, cap, headroom
    return (all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                for v in values)
            and cap > 0 and 0 <= headroom <= cap and 0 < target <= cap
            and 0 <= speed <= cap + headroom)


def encoder_headroom(cfg):
    # 기존 명시된 encoder 1카운트 속도 파라미터다. 이름은 과거 기록/ROS 호환을
    # 위해 유지하며, 0이면 새 실측 여유가 생기지 않는다. 실물에 자동 추정하지 않는다.
    return cfg.current_lane_headroom_mps
