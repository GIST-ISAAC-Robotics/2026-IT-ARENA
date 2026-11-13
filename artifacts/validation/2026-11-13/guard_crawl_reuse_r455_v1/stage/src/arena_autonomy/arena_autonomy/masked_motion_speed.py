"""후방 꼬리 가림 때문에 거부된 높은 목표를 낮춰 같은 이동 검사를 다시 한다.

새 가림 허용량을 만들지 않는다. 실제 측정 속도/현재와 직전 조향을 그대로
쓰고, 기존 1 cm 가림 꼬리 기준과 미관측/전방 가림 거부를 모두 유지한다.
"""
from arena_autonomy.pass_release_trial import region_blocked


def lower_masked_motion(region, partial, target, selector, evaluate):
    """evaluate(target)는 같은 입력의 (region, context)를 반환해야 한다."""
    if (partial.get('allowed') is not True or partial.get('reason') != 'fully_observed'
            or region.get('unobserved') != 0 or region.get('masked_front_samples') != 0
            or not (region.get('masked_left') or region.get('masked_right'))
            or region.get('masked_tail_extent_m', 0.) <= selector.masked_tail_allowance_m
            or target <= .15):
        return None
    attempts=[]
    for cap in (selector.guard_speed_cap_mps, .25, .15):
        if cap>=target or cap>selector.guard_speed_cap_mps:
            continue
        candidate, context=evaluate(cap)
        blocked=region_blocked(candidate,selector)
        attempts.append(dict(target_mps=cap,region=candidate,blocked=blocked))
        if blocked is None:
            return dict(accepted=True,target_mps=cap,region=candidate,context=context,
                diagnostic=dict(applied=True,original_target_mps=target,original_region=region,
                    attempts=attempts,measured_motion_preserved=True,unknown_filled=False))
    return dict(accepted=False,diagnostic=dict(applied=False,original_target_mps=target,
        original_region=region,attempts=attempts,measured_motion_preserved=True,unknown_filled=False))
