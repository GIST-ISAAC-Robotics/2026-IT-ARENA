"""동일 원 scan의 crop 계획을 현재 검사 후보로만 분류한다. 허가/거리 생성 없음."""


SCOPE = 'bounded_simulation_edge_component_not_free_space'


def classified_crop(output):
    normal = output[2].get('normal') or {}
    partial = normal.get('partial_observation') or {}
    component = partial.get('crop_edge_component')
    if (output[2].get('handoff_state') != 'FOLLOW' or normal.get('reason') != 'local_follow'
            or normal.get('selector_mode') != 'SOLO' or normal.get('selector_choice') != 'baseline'
            or normal.get('boundary_from_memory') is not False
            or partial.get('allowed') is not True or partial.get('reason') != 'bounded_side_missing'
            or partial.get('unknown_filled') is not False or partial.get('latch_reason') is not None
            or type(component) is not dict or component.get('scope') != SCOPE):
        return False
    # 표식의 모양만 확인한다. 아래 배열/폭을 현재 이동 영역의 proof로 사용하지 않는다.
    members = component.get('source_indices')
    inside = component.get('inside_source_indices')
    flanks = component.get('flank_source_indices')
    if any(type(ids) is not list or not ids or any(type(i) is not int or i < 0 for i in ids)
           or len(ids) != len(set(ids)) for ids in (members, inside, flanks)):
        return False
    return (len(flanks) == 2 and set(inside).issubset(members)
            and not set(flanks).intersection(members)
            and all(abs(a-b) == 1 for a, b in zip([flanks[0], *members], [*members, flanks[1]])))


def same_scan_scope(data, cfg, *, registered_environment=False):
    """부모 소유 scan 비교는 후보 분류뿐이다. 외부 ROS 입력/점유 허가가 아니다."""
    if (registered_environment is not True or getattr(cfg, 'crop_edge_component_trial', False) is not True
            or not classified_crop(data['output'])):
        return False
    current, submitted = data['current'], data['submitted']
    scan = current.get('scan') or {}; raw = scan.get('measurements') or {}
    evidence = data.get('crop_current_input') or {}
    return (evidence.get('schema') == 'parent_owned_frozen_scan_equality_v1'
            and evidence.get('same_scan') is True
            and evidence.get('submitted_reference_s') == submitted.get('reference_s')
            and evidence.get('current_reference_s') == current.get('reference_s')
            and evidence.get('submitted_timing_verified') is True
            and evidence.get('submitted_timing_policy') == 'verified_only'
            and evidence.get('submitted_time_basis') == 'simulation_acquisition'
            and current.get('timing_verified') is True
            and current.get('timing_policy', 'verified_only') == 'verified_only'
            and scan.get('representation') == 'native_measurements_v1'
            and raw.get('time_basis') == 'simulation_acquisition')


def partial_retry_scope(partial, *, crop_current_scope=False):
    """fully 경로는 원 계약 그대로. crop bool은 현재 입력에서 새로 만든 분류다."""
    return (partial.get('allowed') is True and (partial.get('reason') == 'fully_observed'
            or (crop_current_scope is True and partial.get('reason') == 'bounded_side_missing')))
