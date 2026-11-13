"""빠른 추월 전용: 알려진 지주 가림 뒤의 충분히 긴 현재 외벽 조각 여유.

정적 신원을 확인하는 기능은 아니다. 같은 현재 직선·작은 구조 그림자·단일 상대의
규격면보다 긴 별도 관측이라는 조건부 외벽 가정이다. 빈 공간이나 반환을 만들지 않는다.
"""
import math
import numpy as np

from arena_autonomy.bypass_entry_trial import current_outer_wall_padding
from arena_autonomy.known_structure import classify
from arena_autonomy.wall_reference_support import observed_rear_wall_face


def current_outer_and_rear_padding(scene, config, side, wall, padding_growth_s=0., *, raw_scan, sectors):
    padding, evidence = current_outer_wall_padding(scene, config, side, wall, padding_growth_s)
    if (raw_scan is None or not sectors or len(raw_scan.ranges) != len(scene.rays)
            or wall.get('reason') != 'current_connected_straight_wall_fit'):
        return padding, evidence
    _, _, rows = classify(raw_scan, sectors)
    explained = {frozenset(row['rays'].tolist()) for row in rows if row['explained']}
    fit = (wall['slope'], wall['intercept_m'], *wall['observed_x_m'])
    # 평면 대각선 + 두 끝점의 거리 오차 + 두 끝점의 직선 일치 허용치.
    # 하나의 규격 상대면을 긴 외벽의 연속으로 취급하지 않기 위한 제한이다.
    minimum_span = math.hypot(config.length_m, config.width_m)+2*config.measurement_padding_m+2*.015
    observed = observed_rear_wall_face(scene, config, side, fit, wall, explained,
        minimum_rear_span_m=minimum_span, rear_limit_m=-1.5)
    if observed is None:
        return padding, evidence
    indices = np.asarray([r.source_index for r in scene.rays if r.status == 'return'])
    accepted = np.isin(indices, observed['source_ray_indices'])
    padding[accepted] = .025+config.measurement_padding_m
    proof = {**observed['proof'], 'minimum_rear_span_m': minimum_span,
             'current_rear_wall_assumption': True, 'static_identity_verified': False}
    evidence = {**evidence, 'rear_wall_padding': proof,
        'rear_static_return_count': int(np.sum(accepted)),
        'all_returns_retained': True, 'unknown_filled': False}
    return padding, evidence
