"""정지 근처 SOLO에서도 기존 작은 차체 옆 공백 검사를 동일하게 적용한다."""
import math
from .crop_current_scope import partial_retry_scope

from . import solo_low as prior
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.released_body_gap import check

CAP = .15


def rebase(output, submitted, current, scene, history, cfg, *, require_current_region=False, crop_current_scope=False):
    before = prior.rebase(output, submitted, current, scene, history, cfg,
                          require_current_region=require_current_region)
    normal = output[2].get('normal') or {}
    speed = current['speed_mps']
    if (before is None or before['speed_mps'] > 0
            or before.get('reason') != 'route_current_large_motion_region_unobserved'
            or not math.isfinite(speed) or not 0 <= speed <= CAP or output[0] <= CAP
            or output[2].get('handoff_state') != 'FOLLOW' or normal.get('selector_mode') != 'SOLO'
            or normal.get('boundary_from_memory') is not False
            or not partial_retry_scope(normal.get('partial_observation') or {},
                crop_current_scope=crop_current_scope)):
        return before
    low = (CAP, output[1], output[2])
    # 현재 좌표/점유/유효 시간과 조향은 기존 구현으로 다시 검사한다.
    core = prior.base.candidate_policy.rebase(low, submitted, current, scene, history, cfg)
    if (core is None or not 0 < core['speed_mps'] <= CAP
            or core['steering_rad'] != before['steering_rad']
            or core['reference_s'] != before['reference_s']):
        return before
    raw = MeasurementScan(**thaw(current['scan']['measurements']))
    bounded = check(scene, raw, speed, core['speed_mps'], core['steering_rad'],
        current['previous_steering_rad'], current['dt_s'], cfg,
        prior.base.ps.lateral_config(cfg), prior.base.ps.SelectorConfig())
    # 1cm/6도/5cm 공백과 전방 제외, 실제 속도 한계 등 기존 검사를 그대로 사용한다.
    # 같은 입력의 실제 unobserved 개수는 지우지 않는다.
    if not bounded['accepted']:
        return {**before, 'current_solo_gap_retry': dict(applied=False, cap_mps=CAP, proof=bounded)}
    proof = dict(before['current_route_proof'], region=bounded['region'], accepted=True,
                 bounded_current_risk=bounded, unknown_filled=False)
    return {**core, 'current_route_proof': proof,
            'current_solo_gap_retry': dict(applied=True, cap_mps=CAP, proof=bounded,
                original_reason=before['reason'], measured_speed_mps=speed,
                source_timestamp_renewed=False, old_permission_reused=False,
                same_steering=True, current_geometry_and_occupancy_checked=True)}
