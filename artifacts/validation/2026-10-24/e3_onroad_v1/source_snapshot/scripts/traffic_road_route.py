"""평가용 도로 안 평행 actor 경로. 도로 밖을 허용하는 변경이 아니다."""
import math
import numpy as np
from shapely.strtree import STRtree
from build_experimental_track import rectangle


def check_onroad_route(config, centerline, case, s0, road, obstacles, straight_check, pose_at):
    if case.get('route_kind') != 'side_road_straight' or road is None:
        raise ValueError('explicit on-road side route and official road required')
    lateral = case.get('actor_lateral_offset_m', config['actor'].get('lateral_offset_m', 0.))
    if not math.isfinite(lateral) or lateral == 0 or abs(lateral) > .45:
        raise ValueError('finite nonzero on-road side offset required')
    straight = straight_check(config, centerline, case, s0)
    end = straight['actor_end_s_m']
    # s는 이 시작 직선에서 lap 경계를 건너므로 연속 s로 검사한다.
    if end < s0:
        end += centerline.lap
    samples = np.linspace(s0, end, max(1, math.ceil((end-s0)/.01))+1)
    tree = STRtree(obstacles)
    actor = config['actor']
    minimum_edge = math.inf
    for along in samples:
        footprint = rectangle(*pose_at(centerline, along, [], 0., lateral), actor['length_m'], actor['width_m'])
        if (footprint.difference(road).area > 1e-6
                or any(footprint.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(footprint))):
            raise ValueError(f'on-road actor leaves road or touches wall at s={along:.4f}')
        minimum_edge = min(minimum_edge, footprint.distance(road.boundary))
    return {'kind': 'side_road_straight', **straight, 'lateral_offset_m': lateral,
            'actor_onroad_samples': len(samples), 'maximum_sample_step_m': .01,
            'minimum_sampled_road_edge_distance_m': minimum_edge,
            'scope': 'fixed actor sampled route only; not ego passing clearance or continuous motion proof'}
