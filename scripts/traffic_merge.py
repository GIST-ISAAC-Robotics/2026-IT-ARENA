"""평가 전용 앞차의 완만한 횡방향 합류. 상대차 정답은 대회 제어기에 전달하지 않는다.

종방향 일정에 대한 공간 경로이므로 정지 중 횡이동/제자리 회전이 생기지 않는다.
후륜 기준 경로에서 차체 중심을 계산하지만 가감속·조향 액추에이터·타이어 동역학 모델은 아니다.
"""
from dataclasses import dataclass
import math

import numpy as np

import g1_traffic_eval as ev


def _finite_number(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


@dataclass(frozen=True)
class MergeProfile:
    start_distance_m: float
    transition_m: float
    final_offset_m: float
    wheelbase_m: float = .135
    max_steering_rad: float = .37

    def __post_init__(self):
        values = (self.start_distance_m, self.transition_m, self.final_offset_m,
                  self.wheelbase_m, self.max_steering_rad)
        if (not all(_finite_number(v) for v in values)
                or self.start_distance_m < 0 or min(self.transition_m, self.wheelbase_m, self.max_steering_rad) <= 0
                or self.max_steering_rad >= math.pi/2 or abs(self.final_offset_m) > .15):
            raise ValueError('invalid merge profile')


def merge_profile_for_case(case):
    """미등록 프로파일이나 복수 actor 결합을 조용히 무시하지 않는다."""
    if case.get('route_kind') != 'merge_straight':
        if 'merge_profile' in case:
            raise ValueError('merge profile requires explicit merge_straight route')
        return None
    if case.get('additional_actors') or case.get('behavior_kind', case.get('name')) != 'stop_restart':
        raise ValueError('merge trial supports one stop_restart primary actor')
    try:
        profile = MergeProfile(**case['merge_profile'])
    except (TypeError, KeyError) as error:
        raise ValueError('invalid or missing merge profile') from error
    merge_geometry(0., case.get('actor_lateral_offset_m'), profile)
    if case['actor_lateral_offset_m'] == profile.final_offset_m:
        raise ValueError('merge requires a lateral change')
    return profile


def merge_geometry(distance, initial_offset, profile):
    """후륜 y(x), 기울기·곡률과 등가 조향. 경로 양 끝에서 1·2차 도함수가 0인 quintic 전이."""
    if (not isinstance(profile, MergeProfile) or any(not _finite_number(v)
            for v in (distance, initial_offset)) or distance < 0 or abs(initial_offset) > .15):
        raise ValueError('invalid merge coordinate')
    u = np.clip((distance-profile.start_distance_m)/profile.transition_m, 0., 1.)
    delta = profile.final_offset_m-initial_offset
    h = 10*u**3-15*u**4+6*u**5
    hp = (30*u**2-60*u**3+30*u**4)/profile.transition_m
    hpp = (60*u-180*u**2+120*u**3)/profile.transition_m**2
    y = initial_offset+delta*h
    slope = delta*hp
    curvature = delta*hpp/(1+slope*slope)**1.5
    return {'rear_lateral_m': float(y), 'slope': float(slope), 'yaw_offset_rad': math.atan(slope),
            'curvature_m_inv': float(curvature), 'steering_rad': math.atan(profile.wheelbase_m*curvature)}


def steering_bound(initial_offset, profile):
    merge_geometry(0., initial_offset, profile)
    # max |h''(u)| = 10/sqrt(3). (1+y'^2)^(3/2)>=1 이므로 연속 구간 전체의 보수적 상한이다.
    curvature = abs(profile.final_offset_m-initial_offset)*(10/math.sqrt(3))/profile.transition_m**2
    return math.atan(profile.wheelbase_m*curvature)


def merge_pose(origin, distance, initial_offset, profile):
    """고정 직선 기저의 후륜 경로에서 중심 자세를 구한다. initial pose는 원래 횡배치와 같다."""
    x, y, heading = origin
    if not all(_finite_number(v) for v in origin):
        raise ValueError('invalid merge origin')
    g = merge_geometry(distance, initial_offset, profile)
    alpha = g['yaw_offset_rad']; lr = profile.wheelbase_m/2
    forward = distance-lr+lr*math.cos(alpha)
    lateral = g['rear_lateral_m']+lr*math.sin(alpha)
    return (x+forward*math.cos(heading)-lateral*math.sin(heading),
            y+forward*math.sin(heading)+lateral*math.cos(heading), heading+alpha)


def scheduled_merge_pose(centerline, start_s, schedule, elapsed, initial_offset, profile):
    return merge_pose(centerline.pose(start_s), ev.actor_travel_m(schedule, elapsed), initial_offset, profile)


def validate_merge_route(config, centerline, case, start_s, road, obstacles, straight_check):
    """도로·벽은 5 mm 표본, 등가 조향은 해석 상한으로 사전 검사한다. 연속 충돌 증명은 아니다."""
    from shapely.strtree import STRtree
    from build_experimental_track import rectangle
    if case.get('route_kind') != 'merge_straight' or road is None:
        raise ValueError('explicit merge straight and official road required')
    profile = MergeProfile(**case['merge_profile'])
    initial = float(case['actor_lateral_offset_m'])
    if initial == profile.final_offset_m:
        raise ValueError('merge requires a lateral change')
    bound = steering_bound(initial, profile)
    if bound > profile.max_steering_rad:
        raise ValueError('merge steering bound exceeds registered limit')
    straight = straight_check(config, centerline, case, start_s)
    travel = ev.actor_travel_m(case['schedule'], case['duration_s'])
    if travel < profile.start_distance_m+profile.transition_m:
        raise ValueError('merge is not completed in registered schedule')
    origin = centerline.pose(start_s)
    actor = config['actor']; tree = STRtree(obstacles)
    distances = np.linspace(0., travel, max(1, math.ceil(travel/.005))+1)
    minimum_edge = math.inf
    for distance in distances:
        footprint = rectangle(*merge_pose(origin, float(distance), initial, profile), actor['length_m'], actor['width_m'])
        if (footprint.difference(road).area > 1e-6 or
                any(footprint.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(footprint))):
            raise ValueError(f'merge footprint leaves road or touches wall at travel={distance:.5f}')
        minimum_edge = min(minimum_edge, footprint.distance(road.boundary))
    return {'kind': 'merge_straight', **straight, 'sample_count': len(distances),
            'maximum_sample_step_m': .005, 'minimum_sampled_road_edge_m': minimum_edge,
            'continuous_steering_bound_rad': bound, 'max_steering_rad': profile.max_steering_rad,
            'origin': list(origin), 'profile': vars(profile),
            'scope': 'evaluation rear-path kinematics and sampled road/wall checks; not vehicle dynamics or continuous collision proof'}


def audit_merge_motion(config, case, centerline, start_s, green_s, actor, road, obstacles):
    """실제 합류·방향·노면은 별도 시험기 검사로 확인하며 제어 입력에는 쓰지 않는다."""
    from build_experimental_track import rectangle
    from shapely.strtree import STRtree
    profile = merge_profile_for_case(case)
    rules = config['merge_acceptance']
    keys = ('minimum_lateral_change_m', 'final_offset_error_max_m',
            'moving_heading_error_max_rad', 'static_heading_error_max_rad')
    if (profile is None or not _finite_number(green_s) or road is None
            or any(not _finite_number(rules.get(k)) or rules[k] <= 0 for k in keys)):
        raise ValueError('registered merge evidence required')
    if not ev._pose_rows_valid(actor):
        return {'valid': False, 'reason': 'invalid_actor_poses'}
    end = green_s+case['duration_s']
    coverage = ev.stream_coverage([p[0] for p in actor], green_s, end, ev.POSE_MAX_GAP_S)
    rows = [p for p in actor if green_s <= p[0] <= end]
    if not rows:
        return {'valid': False, 'reason': 'missing_actor_poses'}
    ox, oy, heading = centerline.pose(start_s)
    c, s = math.cos(heading), math.sin(heading)
    offsets, moving_errors, static_errors = [], [], []
    outside = wall = 0
    tree = STRtree(obstacles)
    for t, x, y, yaw in rows:
        elapsed = t-green_s
        expected = scheduled_merge_pose(centerline, start_s, case['schedule'], elapsed,
                                        case['actor_lateral_offset_m'], profile)
        error = abs(math.atan2(math.sin(yaw-expected[2]), math.cos(yaw-expected[2])))
        recent = max(ev.actor_speed_mps(case['schedule'], elapsed-dt) for dt in (0., .1, .2))
        (moving_errors if recent > 0 else static_errors).append(error)
        offsets.append(-(x-ox)*s+(y-oy)*c)
        body = rectangle(x, y, yaw, config['actor']['length_m'], config['actor']['width_m'])
        outside += body.difference(road).area > 1e-6
        wall += any(body.intersection(obstacles[int(i)]).area > 1e-6 for i in tree.query(body))
    checks = {
        'pose_coverage': coverage['valid'],
        'starts_at_registered_offset': abs(offsets[0]-case['actor_lateral_offset_m']) <= rules['final_offset_error_max_m'],
        'lateral_motion_observed': abs(offsets[-1]-offsets[0]) >= rules['minimum_lateral_change_m'],
        'finishes_at_registered_offset': abs(offsets[-1]-profile.final_offset_m) <= rules['final_offset_error_max_m'],
        'moving_heading_matches': bool(moving_errors) and max(moving_errors) <= rules['moving_heading_error_max_rad'],
        'static_heading_matches': bool(static_errors) and max(static_errors) <= rules['static_heading_error_max_rad'],
        'actor_stays_onroad': outside == 0,
        'actor_avoids_walls': wall == 0,
    }
    return {'valid': all(checks.values()), 'checks': checks, 'coverage': coverage,
            'pose_samples': len(rows), 'first_offset_m': offsets[0], 'last_offset_m': offsets[-1],
            'max_moving_heading_error_rad': max(moving_errors, default=None),
            'max_static_heading_error_rad': max(static_errors, default=None),
            'actor_outside_road_samples': int(outside), 'actor_wall_overlap_samples': int(wall),
            'scope': 'evaluation actor partial lateral merge; sampled geometry, not full lane change or vehicle dynamics'}
