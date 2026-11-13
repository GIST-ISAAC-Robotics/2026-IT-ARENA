"""기본 OFF 실행 정책. RELEASED/GUARD 기존 가림 재검사 실패 뒤 .10m/s만 비교한다."""
from contextlib import contextmanager
from contextvars import ContextVar
import math

from arena_autonomy import pass_release_trial as ps
from arena_autonomy.observation_support import source_valid

CAP = .10
MASKED = ('large_motion_masked_toward_left', 'large_motion_masked_toward_right')
_MEASURED = ContextVar('r361_measured_speed', default=None)


def assess(owner, scene, baseline, selected, speed, previous, dt):
    """상태를 바꾸지 않는 같은 입력 비교. 실패이면 원 출력을 그대로 반환한다."""
    cfg = owner.follow_config
    normal, detail = baseline[2], selected[2]
    partial = normal.get('partial_observation') or {}
    measured = _MEASURED.get()
    measured = speed if measured is None else measured
    retry = detail.get('selector_masked_speed_recheck') or {}
    attempts = retry.get('attempts') or []
    evidence = dict(eligible=False, applied=False, reason='outside_released_crawl_scope',
                    cap_mps=CAP, unknown_filled=False, default_changed=False)
    numbers = (measured, speed, previous, dt, baseline[0], baseline[1], selected[0], selected[1])
    if (not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in numbers)
            or not 0 <= measured <= .15 or speed != measured or not 0 < dt <= .1
            or abs(previous) > cfg.max_steering_rad or abs(baseline[1]) > cfg.max_steering_rad
            or not cfg.traffic_recovery_trial or owner.mode != 'RELEASED'
            or detail.get('selector_mode') != 'RELEASED' or detail.get('retained_passage') is not False
            or any(side['present'] for side in owner.sides.values())
            or selected[0] != 0 or baseline[0] <= .15
            or detail.get('selector_reason') not in MASKED
            or detail.get('selector_choice') != 'parallel'
            or normal.get('reason') not in ps.GUARD_BASELINE_REASONS
            or normal.get('boundary_from_memory') is not False
            or normal.get('path_source_s') != scene.source_first_s
            or partial.get('allowed') is not True or partial.get('reason') != 'fully_observed'
            or not source_valid(scene) or not 0 <= scene.t_ref_s-scene.source_first_s < owner.config.max_scene_age_s
            or retry.get('applied') is not False
            or [a.get('target_mps') for a in attempts] != [.35, .25, .15]):
        return selected, evidence
    for attempt in attempts:
        region = attempt.get('region') or {}
        if (attempt.get('blocked') not in MASKED or region.get('unobserved') != 0
                or region.get('masked_front_samples') != 0 or region.get('queries', 0) <= 0
                or region.get('masked_tail_extent_m', 0) <= owner.config.masked_tail_allowance_m):
            return selected, evidence
    context = {}
    region = ps.motion_region(scene, speed, CAP, baseline[1], previous, dt,
                              cfg, owner.lat, context=context)
    blocked = ps.region_blocked(region, owner.config)
    evidence.update(eligible=True, applied=blocked is None,
                    reason='released_crawl_region_accepted' if blocked is None else blocked,
                    measured_speed_mps=speed, steering_rad=baseline[1], previous_steering_rad=previous,
                    dt_s=dt, reference_s=scene.t_ref_s, source_first_s=scene.source_first_s,
                    previous_caps_mps=[.35, .25, .15], region=region, full_region_check=True,
                    source_timestamp_renewed=False, state_transition=False)
    if blocked is not None:
        return selected, evidence
    result = {**detail, **normal, 'selector_choice': 'baseline', 'parallel_hold_selected': False,
              'selector_reason': 'released_masked_crawl_verified', 'selector_region': region,
              'selector_masked_tail_risk_accepted': bool(region['masked_left'] or region['masked_right']),
              'selector_excursion_m': ps.lateral_excursion(max(speed, CAP), scene.t_ref_s-scene.source_first_s,
                                                           baseline[1], previous, dt, cfg),
              'selector_released_crawl_retry': evidence}
    return (CAP, baseline[1], result), evidence


@contextmanager
def enabled(*, shadow=False, observer=None):
    """명시적 단일 Python 과정에서만 연결하고 종료 시 반드시 원 함수를 복구한다."""
    original = ps.PassReleaseSelector._select
    from arena_autonomy.bypass_handoff import BypassHandoffFollow
    original_command = BypassHandoffFollow.command

    def command(owner, scene, history, speed, side, previous, dt, *, raw_scan=None):
        # 기존 forward_drive가 음수 센서값의 절댓값을 계획에 쓸 수 있다.
        # 새 후보는 상위 원 측정값을 따로 보존하고 음수에는 적용하지 않는다.
        token = _MEASURED.set(speed)
        try:
            return original_command(owner, scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        finally:
            _MEASURED.reset(token)

    def selected(owner, scene, baseline, parallel, speed, previous, dt, *, raw_scan=None):
        before = original(owner, scene, baseline, parallel, speed, previous, dt, raw_scan=raw_scan)
        from .guard_crawl import assess as assess_guard
        measured = _MEASURED.get()
        guarded = assess_guard(owner, scene, baseline, before, speed, previous, dt, raw_scan,
                               measured=speed if measured is None else measured)
        after, evidence = assess(owner, scene, baseline, guarded, speed, previous, dt)
        if observer is not None:
            observer(owner, scene, baseline, before, after, evidence, speed, previous, dt)
        return before if shadow else after

    ps.PassReleaseSelector._select = selected
    BypassHandoffFollow.command = command
    try:
        yield
    finally:
        ps.PassReleaseSelector._select = original
        BypassHandoffFollow.command = original_command
