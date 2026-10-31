"""검증된 전체 각본의 동일한45초 접두 구간만 관찰한다. 평가 전용이다."""
from copy import deepcopy
import json
from pathlib import Path

from g1_course_eval import check_full_course
from g1_traffic_eval import actor_travel_m


def prefix_case(config, case):
    reference = json.loads(Path(__file__).with_name('route_reference_config.json').read_text(encoding='utf-8'))
    original = next(c for c in reference['cases'] if c['name'] == case['name'])
    if (case.get('route_kind') != 'full_course' or case.get('duration_s') != 45.
            or original['duration_s'] != 155.
            or config['actor'] != reference['actor']
            or config['acceptance'][case['name']] != reference['acceptance'][case['name']]
            or case.get('actor_lateral_offset_m', 0.) != 0.):
        raise ValueError('registered full-course geometry and 45s prefix required')
    expected = [dict(r, to_s=min(r['to_s'], 45.))
                for r in original['schedule'] if r['from_s'] < 45.]
    if case['schedule'] != expected:
        raise ValueError('limited actor schedule must be the exact registered prefix')
    extended = deepcopy(case)
    extended.update(duration_s=155., schedule=original['schedule'])
    return extended


def check_registered_prefix(config, line, case, start_s, road, obstacles):
    extended = prefix_case(config, case)
    # 전체155초의 모든1cm 표본을 검사하므로45초 안의 경로도 포함한다.
    full = check_full_course(config, line, extended, start_s, road, obstacles)
    travel = actor_travel_m(case['schedule'], case['duration_s'])
    return dict(kind='full_course', limited_observation_s=45.,
                actor_start_s_m=start_s, actor_end_s_m=start_s+travel,
                verified_reference=full, exact_schedule_prefix=True,
                full_lap_driving_claimed=False)
