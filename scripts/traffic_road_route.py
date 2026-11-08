"""평가용 평행 actor 경로. 도로 안 기본값과 명시 두 바퀴 노면 모델을 구별한다."""
import math
import numpy as np
from shapely.geometry import Point
from shapely.strtree import STRtree
from build_experimental_track import rectangle


def check_onroad_route(config, centerline, case, s0, road, obstacles, straight_check, pose_at):
    kind = case.get('route_kind')
    if kind not in ('side_road_straight', 'side_two_wheel_straight') or road is None:
        raise ValueError('explicit on-road side route and official road required')
    two_wheel = kind == 'side_two_wheel_straight'
    actor = config['actor']
    if two_wheel:
        wheelbase, track = actor.get('wheelbase_m'), actor.get('track_m')
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0
               for v in (wheelbase, track)) or wheelbase > actor['length_m'] or track > actor['width_m']:
            raise ValueError('explicit finite wheelbase/track inside actor footprint required')
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
    minimum_edge = math.inf
    maximum_off = 0
    maximum_outside = 0.
    for along in samples:
        pose = pose_at(centerline, along, [], 0., lateral)
        footprint = rectangle(*pose, actor['length_m'], actor['width_m'])
        outside = footprint.difference(road).area
        if two_wheel:
            x, y, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            off = sum(not road.covers(Point(x+c*a-s*b, y+s*a+c*b))
                      for a in (-wheelbase/2, wheelbase/2) for b in (-track/2, track/2))
            maximum_off = max(maximum_off, off)
        else:
            off = 0
        maximum_outside = max(maximum_outside, outside)
        if ((not two_wheel and outside > 1e-6) or off > 2
                or any(footprint.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(footprint))):
            raise ValueError(f'on-road actor leaves road or touches wall at s={along:.4f}')
        minimum_edge = min(minimum_edge, footprint.distance(road.boundary))
    result = {'kind': kind, **straight, 'lateral_offset_m': lateral,
            'actor_onroad_samples': len(samples), 'maximum_sample_step_m': .01,
            'minimum_sampled_road_edge_distance_m': minimum_edge,
            'scope': 'fixed actor sampled route only; not ego passing clearance or continuous motion proof'}
    if two_wheel:
        result.update(actor_max_off_wheel_centers=maximum_off, actor_max_outside_body_area_m2=maximum_outside,
                      actor_wheelbase_m=wheelbase, actor_track_m=track,
                      scope='explicit sampled two-wheel-center actor proxy; not tire contact or referee judgement')
    return result
