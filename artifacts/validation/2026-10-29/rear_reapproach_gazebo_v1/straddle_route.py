"""평가 전용 도로 가장자리 걸침 경로. 자차 보호/노면 판정에는 사용하지 않는다."""
import math

import numpy as np
from shapely.strtree import STRtree


def check_straddle(config, centerline, case, s0, road, obstacles, straight_check, pose_at, rectangle):
    if case.get('route_kind') != 'side_straddle_straight' or road is None:
        raise ValueError('registered straddle route and road required')
    lateral = case.get('actor_lateral_offset_m', config['actor'].get('lateral_offset_m', 0.))
    if not math.isfinite(lateral) or lateral == 0 or abs(lateral) > .45:
        raise ValueError('invalid straddle offset')
    straight = straight_check(config, centerline, case, s0)
    end = straight['actor_end_s_m']
    if end < s0:
        end += centerline.lap
    samples = np.linspace(s0, end, max(1, math.ceil((end-s0)/.01))+1)
    corridor = road.buffer(.20)  # 공식 직선 양측 20 cm 잔디; 상대차 배치 검사 전용.
    tree = STRtree(obstacles)
    actor = config['actor']
    outside = []
    for along in samples:
        footprint = rectangle(*pose_at(centerline, along, [], 0., lateral),
                              actor['length_m'], actor['width_m'])
        off = footprint.difference(road).area
        if (off <= 1e-6 or footprint.intersection(road).area <= 1e-6
                or footprint.difference(corridor).area > 1e-6
                or any(footprint.intersection(obstacles[int(i)]).area > 1e-6
                       for i in tree.query(footprint))):
            raise ValueError(f'actor must straddle road within wall corridor at s={along:.4f}')
        outside.append(off)
    return dict(kind='side_straddle_straight', **straight, lateral_offset_m=lateral,
                sampled_footprints=len(samples), maximum_sample_step_m=.01,
                outside_road_area_range_m2=[min(outside), max(outside)],
                scope='sampled actor placement only; not ego road permission or continuous clearance')


def wrap_route_check(validator):
    original = validator.check_route

    def check(config, centerline, case, s0, road=None, obstacles=()):
        if case.get('route_kind') != 'side_straddle_straight':
            return original(config, centerline, case, s0, road, obstacles)
        return check_straddle(config, centerline, case, s0, road, obstacles,
                              validator.check_straight, validator.actor_pose_at, validator.rectangle)
    return check
