"""진단 전용 초기 선택 대안. 운영 노드/기본 옵션에는 연결하지 않는다."""
import math
import time
import numpy as np
from analyze import predict
from arena_autonomy.bypass_entry_trial import current_outer_wall_padding


def rerank(scene, comparison, config):
    eligible = [row for row in comparison['candidates'] if row.get('eligible')]
    if not eligible:
        return comparison
    points = np.asarray(scene.occupied_endpoints_xy_m)
    ranked = []
    began = time.perf_counter()
    for row in eligible:
        side = row['current_outer_wall']
        padding, _ = current_outer_wall_padding(scene, config, side, comparison['reference']['walls'][side])
        p = predict(np.asarray(row['path_points']), points, padding, config,
                    scene.t_ref_s-scene.source_first_s)
        # 서로 다른 마지막 적분 간격으로 순위를 바꾸지 않는다.
        progress = min(p['horizon_m'], p['first_padded_interference_progress_m'])
        row['prototype_tracking_progress_m'] = progress
        row['prototype_tracking_gap_m'] = p['minimum_padded_point_gap_m']
        ranked.append((row, p, progress))
    row, prediction, progress = max(ranked, key=lambda item:(
        math.floor(item[2]/.02), math.floor(item[1]['minimum_padded_point_gap_m']/.005),
        -abs(item[0]['offset_m']), item[0]['transition_m']))
    chosen = dict(offset_m=row['offset_m'], transition_m=row['transition_m'],
                  speed_mps=row['target_speed_mps'], steering_rad=row['steering_rad'],
                  ranking='offline_initial_pursuit_prediction_diagnostic')
    return {**comparison, 'selected':chosen,
            'prototype_ranking':dict(duration_ms=(time.perf_counter()-began)*1000,
                candidate_count=len(ranked), predicted_progress_m=progress,
                predicted_point_gap_m=prediction['minimum_padded_point_gap_m'],
                future_occupancy_or_observation_permission=False)}


def choose_initial_intent(intent, scene, comparison, original_select):
    """순위와 우회 시작 이득을 같은 예측량으로 대조한다. 기존4 cm 차이는 유지."""
    if intent.sign is not None or 'prototype_ranking' not in comparison:
        return original_select(intent, scene, comparison)
    # 입력/epoch/시각 검사는 원래 함수에서 그대로 수행한다.
    result = original_select(intent, scene, comparison)
    if result['reason'] == 'entry_intent_input_reset':
        return result
    chosen = comparison['selected']
    if chosen is None or chosen['offset_m'] == 0.:
        return result
    rows = [r for r in comparison['candidates'] if r.get('eligible')]
    candidate = next(r for r in rows if r['offset_m']==chosen['offset_m'] and r['transition_m']==chosen['transition_m'])
    straight = next((r for r in rows if r['offset_m']==0.), None)
    gain = None if straight is None else candidate['prototype_tracking_progress_m']-straight['prototype_tracking_progress_m']
    if gain is not None and gain < .04:
        intent.sign = None
        selected = dict(offset_m=0., transition_m=straight['transition_m'],
                        speed_mps=straight['target_speed_mps'], steering_rad=straight['steering_rad'],
                        ranking='no_meaningful_tracking_preview_gain')
    else:
        intent.sign = math.copysign(1.,chosen['offset_m'])
        selected = chosen
    return {**result, 'selected':selected, 'intent_direction':intent.sign,
            'prototype_tracking_gain_m':gain}
