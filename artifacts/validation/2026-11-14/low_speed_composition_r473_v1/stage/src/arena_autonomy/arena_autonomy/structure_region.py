"""알려진 차체 구조 가림의 이동 영역 판단. 관측/측방 상태를 변경하지 않는다."""
from arena_autonomy.known_structure import current_mask_shadow, parse_sectors
from arena_autonomy.measurement_scan import MeasurementScan


def released_structure_region(scene, raw_scan, region, context, cfg, partial):
    """RELEASED의 기존 목표에 같은 구조 그림자 폭을 적용한다.

    실제 측방 통과 기록과 목표 상한은 호출자가 그대로 유지한다. 현재 점유와
    숨은 기억은 이미 baseline의 structure_aware_v1 및 독립 보호에서 검사한다.
    일반 미관측/앞면 가림을 면제하거나 과거 허가를 현재 자유 공간으로 쓰지 않는다.
    """
    invalid = dict(accepted=False, reason='outside_released_structure_scope', unknown_filled=False)
    structure = partial.get('known_structure') or {}
    if (cfg.known_structure_policy != 'structure_aware_v1'
            or not cfg.traffic_recovery_trial or not context
            or region.get('queries', 0) <= 0 or region.get('unobserved') != 0
            or region.get('masked_front_samples') != 0
            or not (region.get('masked_left') or region.get('masked_right'))
            or partial.get('allowed') is not True
            or structure.get('policy') != 'structure_aware_v1'
            or structure.get('evaluated') is not True
            or type(raw_scan) is not MeasurementScan):
        return invalid
    # 같은 호출에서 만든 입력을 대조한다. 원 시각과 frame을 갱신하지 않는다.
    if (scene.source_frame_id != raw_scan.source_frame_id
            or scene.source_first_s != raw_scan.acquired_at_s[0]
            or scene.source_last_s != raw_scan.acquired_at_s[-1]
            or scene.source_angles_rad != raw_scan.angles_rad
            or len(scene.rays) != len(raw_scan.ranges_m)):
        return {**invalid, 'reason': 'released_structure_source_mismatch'}
    try:
        proof = current_mask_shadow(scene, raw_scan,
            context['queries'][context['masked']], context['padding'],
            parse_sectors(cfg.known_structure_sectors_json))
    except (ValueError, TypeError, AttributeError, KeyError, OverflowError) as error:
        return {**invalid, 'reason': 'released_structure_input_invalid', 'error': str(error)}
    return {**proof, 'basis': 'same_current_geometry_as_solo_structure',
            'selector_state_changed': False, 'source_timestamp_renewed': False}
