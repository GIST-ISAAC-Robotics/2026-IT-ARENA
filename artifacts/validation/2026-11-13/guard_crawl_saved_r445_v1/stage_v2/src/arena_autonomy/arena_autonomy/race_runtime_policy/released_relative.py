"""기본 미연결: RELEASED parallel의 현재 차체 옆 공백만 상대 거리로 비교한다.

원 목표/선택 경로/상태/TTL을 바꾸지 않는다. r370 전체 결과를 먼저 구하고,
고정 이웃 거리 하나로 정지한 현재 입력만 같은 RELEASED 경로로 재검사한다.
"""
import math

from arena_autonomy import pass_release_trial as ps
from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan

from . import solo_relative as prior
# r370 -> r352 -> r346 -> r343 -> r281의 같은 현재 기하 정책.
core_policy = prior.prior.prior.prior.base.candidate_policy
DIAGNOSTIC = 'current_released_relative_body'
CAP = .10


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def rebase(output, submitted, current, scene, history, cfg, *, require_current_region=False):
    before = prior.rebase(output, submitted, current, scene, history, cfg,
                          require_current_region=require_current_region)
    if before is None or before['speed_mps'] > 0:
        return before
    normal = output[2].get('normal') or {}
    partial = normal.get('partial_observation') or {}
    route = before.get('current_route_proof') or {}
    fixed = route.get('bounded_current_risk') or {}
    fixed_gap = fixed.get('proof') or {}
    measured, previous, dt = current['speed_mps'], current['previous_steering_rad'], current['dt_s']
    values = (output[0], output[1], measured, previous, dt, before['steering_rad'])
    if (require_current_region is not True or not cfg.released_body_gap_trial
            or not cfg.traffic_recovery_trial or not all(finite(v) for v in values)
            or not 0 < output[0] <= CAP or not 0 <= measured <= .15 or not 0 < dt <= .1
            or max(abs(output[1]), abs(previous), abs(before['steering_rad'])) > cfg.max_steering_rad
            or output[2].get('handoff_state') != 'FOLLOW'
            or normal.get('selector_mode') != 'RELEASED' or normal.get('selector_choice') != 'parallel'
            or normal.get('reason') != 'parallel_hold_follow'
            or normal.get('boundary_from_memory') is not False
            or partial.get('allowed') is not True or partial.get('reason') != 'fully_observed'
            or before.get('reason') != 'route_current_large_motion_region_unobserved'
            or before.get('rebased_selector_mode') != 'RELEASED'
            or before.get('rebased_selector_choice') != 'parallel'
            or route.get('accepted') is not False or fixed.get('accepted') is not False
            or fixed_gap.get('policy') != 'guard_low_speed_body_nonreturn_r136'
            or fixed_gap.get('reason') != 'near_gap_neighbor'):
        return before

    # 원 RELEASED parallel 분기를 그대로 호출한다. 선택 경로 일치/횡이동 1cm,
    # 원 시각/epoch/유효기간, 현재 점유/전방 관측/새 측방 물체가 먼저 통과해야 한다.
    core = core_policy.rebase(output, submitted, current, scene, history, cfg)
    if (core is None or not finite(core['speed_mps']) or not 0 < core['speed_mps'] <= output[0]
            or core.get('reason') != 'rebase_current_released_parallel'
            or core.get('current_side_presence') != {'left': False, 'right': False}
            or not finite(core.get('maximum_body_corner_lateral_m'))
            or not 0 <= core['maximum_body_corner_lateral_m'] <= .01
            or core['speed_mps'] != fixed.get('target_mps')
            or any(core.get(k) != before.get(k) for k in (
                'steering_rad', 'reference_s', 'path_source_s', 'original_reference_s', 'path_points'))):
        return before

    context = {}
    region = ps.motion_region(scene, measured, core['speed_mps'], core['steering_rad'],
                              previous, dt, cfg, ps.lateral_config(cfg), context=context)
    # 같은 현재 입력/원 목표로 얻은 영역이어야 한다. unknown 수·마스크를 줄이지 않는다.
    if region != route.get('region') or not region['unobserved'] or not context:
        return before
    raw = MeasurementScan(**thaw(current['scan']['measurements']))
    queries = context['queries'][~context['free'] & ~context['masked']]
    proof = bounded_body_nonreturn(scene, raw, queries, context['padding'], cfg.length_m,
                                  cfg.width_m, measured, core['speed_mps'], relative_neighbor=True)
    blocked = ps.region_blocked(region, ps.SelectorConfig(), bounded_unknown_risk=proof['accepted'])
    accepted = region['queries'] > 0 and proof['accepted'] and blocked is None
    diagnostic = dict(applied=accepted, policy='released_parallel_current_relative_body_r375',
        proof=proof, region=region, blocked=blocked, original_reason=before['reason'],
        original_target_mps=output[0], target_mps=core['speed_mps'], measured_speed_mps=measured,
        steering_rad=core['steering_rad'], previous_steering_rad=previous, dt_s=dt,
        current_full_recheck=True, same_path=True, same_steering=True, state_transition=False,
        target_increased=False, unknown_filled=False, source_timestamp_renewed=False,
        old_occupancy_permission_reused=False, default_changed=False)
    if not accepted:
        return {**before, DIAGNOSTIC: diagnostic}
    checked_route = {**route, 'accepted': True, 'bounded_current_risk': diagnostic}
    return {**core, 'current_route_proof': checked_route, DIAGNOSTIC: diagnostic}
