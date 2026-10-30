"""평가-only: 앞차1.4 m 이동 뒤 정지, 자차1랩 목표를 서로 구분한다."""
import math
import numpy as np
from shapely.strtree import STRtree
import g1_traffic_eval as ev

SCHEDULE=[dict(from_s=0.,to_s=4.,speed_mps=.35),dict(from_s=4.,to_s=155.,speed_mps=0.)]


def blocked_route(config,line,case,start_s,road,obstacles):
    rules=config['acceptance'][case['name']]
    travel=ev.actor_travel_m(case['schedule'],case['duration_s']);limit=rules.get('maximum_actor_laps')
    if (case.get('name')!='blocked_full_course' or case.get('route_kind')!='full_course'
        or case.get('duration_s')!=155. or case.get('schedule')!=SCHEDULE
        or type(rules.get('minimum_laps')) is not int or rules['minimum_laps']!=1
        or not ev._finite(limit) or not 1<limit<=1.5 or abs(travel-1.4)>1e-9 or not 0<=travel<=limit*line.lap
        or road is None or case.get('actor_lateral_offset_m',0.)!=0.
        or rules.get('surface_policy')!='two_wheel_centers'
        or rules.get('wheelbase_m')!=.135 or rules.get('track_m')!=.135):
        raise ValueError('registered blocked route, 1.4 m actor travel and one-lap ego goal required')
    actor=config['actor'];samples=np.linspace(start_s,start_s+travel,math.ceil(travel/.01)+1);tree=STRtree(obstacles)
    for s in samples:
        footprint=ev.rectangle(*line.pose(s),actor['length_m'],actor['width_m'])
        if footprint.difference(road).area>1e-6 or any(footprint.intersection(obstacles[int(i)]).area>1e-6 for i in tree.query(footprint)):
            raise ValueError('blocked actor route outside road or into wall')
    return dict(kind='full_course_blocked_lead',actor_start_s_m=start_s,actor_end_s_m=start_s+travel,
        lap_length_m=line.lap,minimum_ego_laps=1,actor_travel_m=travel,
        actor_road_wall_samples=len(samples),maximum_sample_step_m=.01)


def install_route_check(validator):
    original=validator.check_route
    def check(config,line,case,start_s,road,obstacles):
        if case.get('name')=='blocked_full_course' and case.get('route_kind')=='full_course':
            return blocked_route(config,line,case,start_s,road,obstacles)
        return original(config,line,case,start_s,road,obstacles)
    validator.check_route=check
