"""평가 전용 일정한 속도의 직선→원호→직선 상대차. 대회 제어기에 전달하지 않는다."""
import math

import numpy as np
from shapely.geometry import Point
from shapely.strtree import STRtree

from build_experimental_track import rectangle
from g1_traffic_eval import actor_travel_m


def corner_geometry(config, centerline, case, s0):
    if case.get('route_kind') != 'side_first_curve':
        raise ValueError('explicit side first curve required')
    spec = case['corner_route']
    keys = ('x_m', 'y_m', 'incoming_yaw_rad', 'turn_rad', 'center_radius_m', 'exit_extent_m')
    if any(isinstance(spec.get(k), bool) or not isinstance(spec.get(k), (int, float))
           or not math.isfinite(spec[k]) for k in keys):
        raise ValueError('finite registered corner geometry required')
    turn, radius = spec['turn_rad'], spec['center_radius_m']
    lateral = case['actor_lateral_offset_m']
    if (not math.isfinite(lateral) or abs(lateral) > .45 or lateral == 0.
            or not .35 <= radius <= .8 or not .3 <= spec['exit_extent_m'] <= 1.2
            or not math.isclose(turn, math.pi/2, abs_tol=1e-9)):
        raise ValueError('bounded left ninety-degree side route required')
    heading = spec['incoming_yaw_rad']
    t = np.array([math.cos(heading), math.sin(heading)])
    n = np.array([-t[1], t[0]])
    corner = np.array([spec['x_m'], spec['y_m']])
    center = corner-radius*t+radius*n
    arc_radius = radius-lateral
    if arc_radius < .355:
        raise ValueError('actor center turn radius below this test envelope')
    x, y, yaw = centerline.pose(s0)
    if abs(math.atan2(math.sin(yaw-heading), math.cos(yaw-heading))) > .02:
        raise ValueError('actor does not start on incoming straight')
    start = np.array([x, y])+lateral*n
    first = center-arc_radius*n
    straight = float((first-start) @ t)
    if straight <= 0. or abs(float((first-start) @ n)) > .01:
        raise ValueError('registered corner does not join incoming actor line')
    return dict(start=start, center=center, t=t, n=n, heading=heading, radius=arc_radius,
                straight=straight, arc=arc_radius*turn, exit=spec['exit_extent_m'])


def pose_on_corner(geometry, distance):
    g = geometry
    d = max(0., float(distance))
    if d > g['straight']+g['arc']+g['exit']+1e-8:
        raise ValueError('actor beyond registered corner exit')
    if d <= g['straight']:
        p = g['start']+d*g['t']
        yaw = g['heading']
    elif d <= g['straight']+g['arc']:
        theta = (d-g['straight'])/g['radius']
        p = g['center']+g['radius']*(math.sin(theta)*g['t']-math.cos(theta)*g['n'])
        yaw = g['heading']+theta
    else:
        p = g['center']+g['radius']*g['t']+(d-g['straight']-g['arc'])*g['n']
        yaw = g['heading']+math.pi/2
    return float(p[0]), float(p[1]), yaw


def corner_pose(config, centerline, s0, case, elapsed):
    return pose_on_corner(corner_geometry(config, centerline, case, s0),
                          actor_travel_m(case['schedule'], elapsed))


def check_corner_route(config, centerline, case, s0, road, obstacles):
    if road is None:
        raise ValueError('road required for corner route')
    g = corner_geometry(config, centerline, case, s0)
    travel = actor_travel_m(case['schedule'], case['duration_s'])
    if not g['straight']+g['arc']+.3 <= travel <= g['straight']+g['arc']+g['exit']:
        raise ValueError('schedule must complete arc and at least 0.3m exit')
    actor = config['actor']
    wb, track = actor.get('wheelbase_m'), actor.get('track_m')
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
           or not 0 < v <= .2 for v in (wb, track)) or wb > actor['length_m'] or track > actor['width_m']:
        raise ValueError('explicit actor wheel geometry required')
    tree = STRtree(obstacles)
    samples = np.linspace(0., travel, math.ceil(travel/.005)+1)
    off_max, outside_max = 0, 0.
    for d in samples:
        x, y, yaw = pose_on_corner(g, d)
        footprint = rectangle(x, y, yaw, actor['length_m'], actor['width_m'])
        c, s = math.cos(yaw), math.sin(yaw)
        off = sum(not road.covers(Point(x+c*a-s*b, y+s*a+c*b))
                  for a in (-wb/2, wb/2) for b in (-track/2, track/2))
        off_max = max(off_max, off)
        outside_max = max(outside_max, footprint.difference(road).area)
        if off > 2 or any(footprint.intersection(obstacles[int(i)]).area > 1e-6
                          for i in tree.query(footprint)):
            raise ValueError(f'actor route exceeds two wheels or contacts wall at distance={d:.4f}')
    peak = max(r['speed_mps'] for r in case['schedule'])
    return dict(kind=case['route_kind'], actor_start_s_m=s0, actor_travel_m=travel,
                actor_turn_radius_m=g['radius'], actor_arc_begin_distance_m=g['straight'],
                actor_arc_end_distance_m=g['straight']+g['arc'],
                maximum_scheduled_yaw_rate_rad_s=peak/g['radius'],
                actor_road_wall_samples=len(samples), maximum_sample_step_m=.005,
                actor_max_off_wheel_centers=off_max, actor_max_outside_body_area_m2=outside_max,
                scope='evaluation-only constant physical-speed arc; sampled footprint, not vehicle dynamics')
