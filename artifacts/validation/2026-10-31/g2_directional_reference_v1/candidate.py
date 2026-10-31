"""오프라인 비교: 양벽의 방향 합의와 선택한 외벽의 경로 길이를 분리한다.

공통 전방 관측은 25 cm 이상 유지한다. 반대쪽의 짧은 관측 때문에 관측이
충분한 쪽까지 버리지 않는다. 각 후보 경로의 뒤 차축 포함 여부와 현재 이동
영역 검사는 기존 entry_path/entry_motion_region이 그대로 담당한다.
"""
from arena_autonomy.bypass_initial_reference import CurrentPairReference


class DirectionalPairReference(CurrentPairReference):
    def resolve(self, scene, config, history):
        reference, detail = super().resolve(scene, config, history)
        if (reference is not None or detail.get('reason') != 'reference_does_not_cover_entry'
                or detail.get('all_returns_remain_occupied') is not True):
            return reference, detail
        walls = detail['walls']
        a, b = (walls[side] for side in ('left', 'right'))
        lower = max(a['observed_x_m'][0], b['observed_x_m'][0])
        upper = min(a['observed_x_m'][1], b['observed_x_m'][1])
        if upper-max(lower, 0.) < .25:
            return reference, detail
        rear = -config.wheelbase_m/2
        covering = [side for side, wall in walls.items()
                    if wall['observed_x_m'][0] <= rear and wall['observed_x_m'][1]-rear >= .30]
        if not covering:
            return reference, detail
        disagreement = max(abs((a['slope']-b['slope'])*x+a['intercept_m']-b['intercept_m'])
                           for x in (lower, upper))
        if disagreement > config.boundary_agreement_m:
            return None, dict(detail, reason='observed_wall_reference_disagrees',
                              disagreement_m=float(disagreement))
        # 이 공통 구간을 뒤로 늘리지 않는다. 실행 경로는 선택한 외벽의 실제 구간을 쓴다.
        result = ((a['slope']+b['slope'])/2, (a['intercept_m']+b['intercept_m'])/2, lower, upper)
        return result, dict(detail, reason='current_pair_directional_coverage_candidate',
            disagreement_m=float(disagreement), entry_covering_sides=covering,
            common_observed_x_m=[lower, upper],
            path_requires_selected_wall_coverage=True, unknown_filled=False,
            wall_offset_assumption_m=config.wall_offset_m)
