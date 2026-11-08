"""명시 1/5랩 평가 전용 경로 검사안. 현재 주행 코드에는 아직 연결하지 않았다."""
import math

import numpy as np
from shapely.strtree import STRtree

import g1_traffic_eval as ev


def check_full_course(config, line, case, start_s, road, obstacles):
    rules = config['acceptance'][case['name']]
    laps = rules.get('minimum_laps')
    travel = ev.actor_travel_m(case['schedule'], case['duration_s'])
    limit = rules.get('maximum_actor_laps')
    if (type(laps) is not int or laps not in (1, 5) or not ev._finite(limit)
            or not laps < limit <= laps + .5
            or not laps*line.lap < travel <= limit*line.lap or road is None
            or case.get('actor_lateral_offset_m', 0.) != 0.
            or rules.get('surface_policy') != 'two_wheel_centers'
            or rules.get('wheelbase_m') != .135 or rules.get('track_m') != .135):
        raise ValueError('registered one/five-lap center route and bounded actor extent required')
    actor = config['actor']
    samples = np.linspace(start_s, start_s+travel, math.ceil(travel/.01)+1)
    tree = STRtree(obstacles)
    for s in samples:
        footprint = ev.rectangle(*line.pose(s), actor['length_m'], actor['width_m'])
        if (footprint.difference(road).area > 1e-6 or
                any(footprint.intersection(obstacles[int(i)]).area > 1e-6
                    for i in tree.query(footprint))):
            raise ValueError(f'full-course actor footprint outside road or into wall at s={s:.4f}')
    return dict(kind='full_course', actor_start_s_m=start_s, actor_end_s_m=start_s+travel,
        lap_length_m=line.lap, minimum_laps=laps, maximum_actor_laps=limit,
        actor_road_wall_samples=len(samples), maximum_sample_step_m=.01)
