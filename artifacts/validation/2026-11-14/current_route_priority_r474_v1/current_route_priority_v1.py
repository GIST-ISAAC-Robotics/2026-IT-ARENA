"""현재 경로와 실제 점유를 우선하고 중복 관측 정지 조건은 적용하지 않는다.

과거 측방 이력이나 자기 가림을 지우거나 자유 공간으로 바꾸지 않는다.
현재 경로가 성립한 경우 그 이력만으로 별도 전역 가시성 증명을 요구하지 않는다.
현재 실제 점유 swept 검사와 독립 보호층은 유지한다.
"""
import math

POLICY = 'current_route_priority_v1'


def enabled(cfg):
    return cfg.traffic_recovery_trial and cfg.guard_observation_policy == 'selected_r181'


def select(cfg, scene, baseline, view, detail):
    speed, steering, normal = baseline
    if (not enabled(cfg) or speed <= 0 or normal.get('reason') != 'local_follow'
            or normal.get('boundary_from_memory') is not False
            or normal.get('path_source_s') != scene.source_first_s
            or (normal.get('partial_observation') or {}).get('allowed') is not True
            or view is None or any(view[name]['present'] for name in ('left', 'right'))):
        return None
    # baseline이 현재 경로/점유 검사를 이미 끝냈다. 이전 측방 상태는 진단용으로
    # 유지하되 retained .35 상한, 차체 전 이동 외곽의 완전 관측을 다시 요구하지 않는다.
    return speed, steering, {**normal, **detail, 'selector_choice': 'baseline',
        'selector_reason': POLICY, 'selector_observation_policy': POLICY,
        'selector_unknown_filled': False}


def supported(output):
    speed, _, detail = output
    normal = detail.get('normal') or {}
    return (type(speed) in (int, float) and math.isfinite(speed) and speed > 0
            and detail.get('handoff_state') == 'FOLLOW'
            and normal.get('reason') == 'local_follow'
            and normal.get('selector_choice') == 'baseline'
            and normal.get('selector_observation_policy') == POLICY)


def rebase(output, submitted, current, scene, history, cfg):
    if not enabled(cfg) or not supported(output):
        return None
    from .retained_recheck import _geometry
    normal = output[2]['normal']
    # 이 순수 기하 함수의 분류만 맞춘다. 실제 선택기 상태/측방 이력은 변경하지 않는다.
    geometry = (output[0], output[1], {**output[2], 'normal': {**normal, 'selector_mode': 'SOLO'}})
    result = _geometry.rebase(geometry, submitted, current, scene, history, cfg)
    if result is None:
        raise RuntimeError('current route geometry unexpectedly unsupported')
    result = {**result, 'current_observation_policy': POLICY,
        'rebased_selector_mode': normal.get('selector_mode'), 'state_transition': False,
        'unknown_filled': False, 'source_permission_reused': False,
        'observation_policy_scope': 'current path and occupied sweep; no full-sweep visibility veto'}
    # 단일 결측/자기 가림마다 정지하지 않는다. 현재 전방 관측이 전부 소실되면
    # 오래된 경로만으로 주행하지 않으며, 복구는 다음 정상 입력에서 다시 평가한다.
    if result['speed_mps'] > 0 and not any(
            ray.status == 'return'
            and abs(math.atan2(ray.direction_xy[1], ray.direction_xy[0])) <= math.radians(100)
            for ray in scene.rays):
        result.update(speed_mps=0., reason='current_route_front_observation_lost')
    return result
