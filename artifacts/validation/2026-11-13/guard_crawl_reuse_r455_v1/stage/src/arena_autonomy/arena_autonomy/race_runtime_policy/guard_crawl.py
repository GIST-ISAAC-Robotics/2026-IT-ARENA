"""명시 실행 정책에서 기존 GUARD 가림 재검사 뒤 같은 경로를 .10 m/s로 검사한다.

관측된 측방 물체·통과 이력·미관측 영역과 1 cm 가림 한계는 유지한다.
고정 물체 분류나 정체 시간에 따른 자동 출발이 아니다.
"""
import math

from arena_autonomy import pass_release_trial as ps
from arena_autonomy.observation_support import source_valid

CAP = .10
_NO_VIEW = object()
REASONS = ('guard_curve_rejected_large_motion_masked_toward_left',
           'guard_curve_rejected_large_motion_masked_toward_right')


def assess(owner, scene, baseline, selected, speed, previous, dt, raw_scan, *, measured, current_view=_NO_VIEW):
    """이미 갱신된 selector 상태를 그대로 읽고 전체 GUARD 검사를 한 번만 수행한다."""
    cfg = owner.follow_config
    normal, detail = baseline[2], selected[2]
    partial = normal.get('partial_observation') or {}
    original = detail.get('selector_guard_candidate') or {}
    retry = detail.get('selector_guard_speed_recheck') or {}
    attempts = retry.get('attempts') or []
    numbers = (measured, speed, previous, dt, baseline[0], baseline[1], selected[0], selected[1])
    if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in numbers)
            or not 0 <= measured <= .15 or measured != speed or not 0 < dt <= .1
            or not cfg.traffic_recovery_trial or not cfg.guard_curve_trial
            or owner.mode != 'GUARD' or detail.get('selector_mode') != 'GUARD'
            or selected[0] != 0 or baseline[0] <= .15 or raw_scan is None
            or detail.get('selector_choice') != 'parallel' or detail.get('selector_reason') not in REASONS
            or normal.get('reason') not in ps.GUARD_BASELINE_REASONS
            or normal.get('boundary_from_memory') is not False
            or normal.get('path_source_s') != scene.source_first_s
            or partial.get('allowed') is not True or partial.get('reason') != 'fully_observed'
            or abs(previous) > cfg.max_steering_rad or abs(baseline[1]) > cfg.max_steering_rad
            or not source_valid(scene) or not 0 <= scene.t_ref_s-scene.source_first_s < owner.config.max_scene_age_s
            or original.get('accepted') is not False or retry.get('applied') is not False
            or original.get('trigger') != 'parallel_hold_reference_not_straight'
            or [a.get('target_mps') for a in attempts] != [.25, .15]):
        return selected
    regions = [original.get('region') or {}] + [a.get('region') or {} for a in attempts]
    if (any(a.get('accepted') is not False or a.get('reason') not in REASONS for a in attempts)
            or any(r.get('unobserved') != 0 or r.get('masked_front_samples') != 0
                   or r.get('queries', 0) <= 0
                   or not r.get('masked_tail_extent_m', 0) > owner.config.masked_tail_allowance_m
                   for r in regions)):
        return selected

    # 명시 runtime의 같은 _select/_guard_curve 호출에서 받은 view만 재사용한다.
    # 직접 helper 호출에는 기존 계산을 유지하며, None도 계산된 결과로 구별한다.
    view = (ps.threat_view(scene, normal.get('path_points'), owner.lat)
            if current_view is _NO_VIEW else current_view)
    parallel = (selected[0], selected[1], {**detail, 'reason': original['trigger']})
    # 원 _select의 상태 메타데이터만 전달한다. 실패한 parallel의 HOLD/reason/path로
    # 양수 baseline 출력의 의미를 덮어쓰면 후속 현재 경로 검사가 후보를 받지 못한다.
    selector_detail = {key: detail[key] for key in (
        'selector_mode_before', 'pass_release_trial', 'selector_unknown_filled',
        'selector_hidden_rear_risk_unresolved', 'selector_mode', 'selector_evidence',
        'selector_sides', 'selector_guard_speed_recheck') if key in detail}
    checked = owner._guard_curve_at_cap(scene, view, baseline, parallel, speed, previous, dt,
        scene.t_ref_s-scene.source_first_s, selector_detail, raw_scan, target_cap=CAP)
    proof = {} if checked is None else checked[2].get('selector_guard_candidate') or {}
    accepted = checked is not None and checked[0] > 0 and proof.get('accepted') is True
    result = checked if accepted else selected
    return result[0], result[1], {**result[2], 'selector_guard_crawl_retry': dict(
        applied=accepted, cap_mps=CAP, measured_speed_mps=measured,
        previous_caps_mps=[original.get('target_speed_mps'), .25, .15],
        low_reason=None if checked is None else checked[2].get('selector_reason'),
        low_guard=proof, full_guard_rechecked=True, state_transition=False,
        source_timestamp_renewed=False, unknown_filled=False)}
