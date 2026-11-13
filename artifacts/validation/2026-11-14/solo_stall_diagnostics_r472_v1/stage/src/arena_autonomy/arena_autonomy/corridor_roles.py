"""운영 미연결: 선호 경계와 코너 안쪽의 역할을 분리한 기하 후보.

선호 경계를 바꾸지 않고 기존 후보부터 평가한다. 반대 역할의 추가 후보도
현재 선호 경계에서 만든 경로와 대조한다. 경로 기억·시간 확인·점유·관측·보호
검사는 별개이며, 이 결과만으로 명령을 만들거나 경계 기억을 갱신하지 않는다.
"""
from arena_autonomy.corner_corridor import corridor_evidence
from arena_autonomy.local_follow import path_agrees
from arena_autonomy.local_path import local_path


def role_aware_evidence(points, preferred_side, config):
    """기존 역할의 성공은 유지하고, 실패 시에만 선호 경계를 바깥 후보로 평가한다.

입력 점은 필터링하지 않는다. 새 후보는 이전의 경로 기억과 일치한다거나
분기 의미까지 확인했다는 주장이 아니다. 반환 preferred_side는 호출자가 준
선호 경계, inner_side/outer_side는 해당 기하 가설의 역할이다.
"""
    baseline = corridor_evidence(points, preferred_side, config)
    metadata = dict(preferred_side=preferred_side, inner_side=preferred_side,
                    role_selection='original_inner_role', baseline_reason=baseline['reason'],
                    preferred_observed_path_agrees=None, temporal_confirmation_evaluated=False,
                    memory_refresh_authorized=False, driving_authorized=False,
                    runtime_connected=False, branch_semantics_certified=False)
    if baseline['supported'] or baseline['reason'] == 'invalid_input':
        return {**baseline, **metadata}

    alternative_inner = 'right' if preferred_side == 'left' else 'left'
    alternative = corridor_evidence(points, alternative_inner, config)
    attempt = dict(inner_side=alternative_inner, outer_side=preferred_side,
                   supported=alternative['supported'], reason=alternative['reason'])
    if not alternative['supported']:
        return {**baseline, **metadata, 'alternative_role': attempt}

    preferred_path, preferred_info = local_path(points, preferred_side, config.wall_offset_m)
    agrees = bool(preferred_path is not None and
                  preferred_info['boundary_observed_ahead_m'] >= config.boundary_min_ahead_m and
                  abs(preferred_info['wall_distance_m']-config.wall_offset_m) <= config.boundary_anchor_error_m and
                  path_agrees(alternative['path'], preferred_path, config.boundary_agreement_m))
    if not agrees:
        return {**baseline, **metadata, 'alternative_role': attempt,
                'preferred_observed_path_agrees': False,
                'role_rejection': 'preferred_observed_path_disagreement'}
    return {**alternative, **metadata, 'inner_side': alternative_inner,
            'role_selection': 'preferred_outer_role', 'preferred_observed_path_agrees': True,
            'preferred_info': preferred_info, 'alternative_role': attempt}
