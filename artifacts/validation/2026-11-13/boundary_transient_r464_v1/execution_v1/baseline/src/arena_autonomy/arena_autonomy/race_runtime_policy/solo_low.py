"""정지 근처 SOLO의 자체 마스크 거절에 낮은 양수 목표를 한 번 재검사."""
import math

from . import current_region as base
CAP = .15


def rebase(output, submitted, current, scene, history, cfg, *, require_current_region=False):
    before = base.rebase(output, submitted, current, scene, history, cfg,
                         require_current_region=require_current_region)
    if before is None or before['speed_mps'] > 0:
        return before
    normal = output[2].get('normal') or {}
    region = (before.get('current_route_proof') or {}).get('region') or {}
    speed = current['speed_mps']
    if (not math.isfinite(speed) or not 0 <= speed <= CAP or output[0] <= CAP
            or output[2].get('handoff_state') != 'FOLLOW' or normal.get('selector_mode') != 'SOLO'
            or before.get('reason') not in ('route_current_large_motion_masked_toward_left',
                                            'route_current_large_motion_masked_toward_right')
            or region.get('unobserved') != 0 or region.get('masked_front_samples') != 0
            or region.get('queries', 0) <= 0):
        return before
    # 경로/현재 입력/측정속도/시각은 그대로다. 과거 검사 허가를 재사용하지 않고
    # 낮은 양수 상한에서 점유·현재 영역을 포함한 기존 함수를 통째로 다시 검사한다.
    low = (CAP, output[1], output[2])
    checked = base.rebase(low, submitted, current, scene, history, cfg,
                          require_current_region=True)
    accepted = (checked is not None and 0 < checked['speed_mps'] <= CAP
                and checked['steering_rad'] == before['steering_rad']
                and checked['reference_s'] == before['reference_s'])
    result = checked if accepted else before
    return {**result, 'current_solo_low_retry': dict(applied=accepted, cap_mps=CAP,
        original_reason=before['reason'], original_target_mps=output[0], measured_speed_mps=speed,
        retry_reason=None if checked is None else checked['reason'], full_current_check=True,
        unknown_filled=False, source_timestamp_renewed=False, changed_steering=False)}
