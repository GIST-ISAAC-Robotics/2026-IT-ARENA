"""통과 상태의 기존 저속 차체 옆 공백 검사. 현재 관측·측방 상태를 대체하지 않는다."""
import math
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.current_path_recheck._gaps import bounded_body_nonreturn, bounded_adjacent_structure
from arena_autonomy.known_structure import parse_sectors


def check(scene, scan, speed, target, steering, previous, dt, cfg, lateral, selector):
    """같은 실제 속도/현재 광선의 이동 영역 전체를 다시 검사한다."""
    if (scan is None or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
            for v in (speed, target, steering, previous, dt))
            or not 0 <= speed <= .35 or not 0 < target <= .35 or not 0 < dt <= .1):
        return dict(accepted=False, reason='invalid_motion_or_scan', unknown_filled=False)
    context = {}
    region = ps.motion_region(scene, speed, target, steering, previous, dt, cfg, lateral, context=context)
    proof = dict(accepted=False, reason='no_unknown_context', unknown_filled=False)
    if region['unobserved'] and context:
        queries = context['queries'][~context['free'] & ~context['masked']]
        if cfg.known_structure_policy == 'structure_aware_v1':
            proof = bounded_adjacent_structure(scene, scan, queries, context['padding'],
                cfg.length_m, cfg.width_m, speed, target, parse_sectors(cfg.known_structure_sectors_json))
        else:
            proof = bounded_body_nonreturn(scene, scan, queries, context['padding'],
                                          cfg.length_m, cfg.width_m, speed, target)
    blocked = ps.region_blocked(region, selector, bounded_unknown_risk=proof['accepted'])
    return dict(accepted=(not region['unobserved'] or proof['accepted']) and blocked is None, proof=proof, region=region,
                blocked=blocked, measured_speed_mps=speed, target_mps=target, unknown_filled=False)


def propose(owner, scene, scan, baseline, speed, previous, dt, limit):
    """선택기 상태를 바꾸지 않는다. 현재 기준 경로·낮은 실제 속도에만 적용한다."""
    cfg = owner.follow_config
    normal = baseline[2]
    partial = normal.get('partial_observation') or {}
    values = (speed, previous, dt, baseline[0], baseline[1], limit)
    if (not cfg.released_body_gap_trial or owner.mode != 'RELEASED'
            or not all(isinstance(v, (float, int)) and not isinstance(v, bool) and math.isfinite(v) for v in values)
            or not 0 <= speed <= .35 or not 0 < dt <= .1 or limit <= 0
            or any(side['present'] for side in owner.sides.values())
            or normal.get('reason') not in ps.NORMAL_BASELINE
            or normal.get('boundary_from_memory') is not False
            or normal.get('path_source_s') != scene.source_first_s
            or partial.get('allowed') is not True or partial.get('reason') != 'fully_observed'):
        return None
    attempts = []
    for target in (.35, .25, .15):
        if target > limit or target >= baseline[0] or speed > target or target > owner.config.guard_speed_cap_mps:
            continue
        checked = check(scene, scan, speed, target, baseline[1], previous, dt, cfg, owner.lat, owner.config)
        attempts.append(checked)
        if checked['accepted']:
            return dict(accepted=True, target_mps=target, steering_rad=baseline[1],
                        attempts=attempts, state_transition=False, unknown_filled=False)
    return None
