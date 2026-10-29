"""교통 전체 랩의 평가 전용 기하. 제어기로 지도/정답을 전달하지 않는다."""
import math
import numpy as np
from shapely.geometry import Point
from shapely.prepared import prep
from shapely.strtree import STRtree
import g1_traffic_eval as ev


def check_full_course(config, line, case, start_s, road, obstacles):
    rules = config['acceptance'][case['name']]
    laps = rules.get('minimum_laps')
    travel = ev.actor_travel_m(case['schedule'], case['duration_s'])
    limit = rules.get('maximum_actor_laps')
    if (type(laps) is not int or laps != 1 or not ev._finite(limit) or not 1 < limit <= 1.5
            or not laps*line.lap < travel <= limit*line.lap or road is None
            or case.get('actor_lateral_offset_m',0.) != 0.
            or rules.get('surface_policy') != 'two_wheel_centers'
            or rules.get('wheelbase_m') != .135 or rules.get('track_m') != .135):
        raise ValueError('registered one-lap center route and bounded actor extent required')
    actor=config['actor']
    samples=np.linspace(start_s,start_s+travel,math.ceil(travel/.01)+1)
    tree=STRtree(obstacles)
    for s in samples:
        footprint=ev.rectangle(*line.pose(s),actor['length_m'],actor['width_m'])
        if (footprint.difference(road).area>1e-6 or
                any(footprint.intersection(obstacles[int(i)]).area>1e-6 for i in tree.query(footprint))):
            raise ValueError(f'full-course actor footprint outside road or into wall at s={s:.4f}')
    return dict(kind='full_course',actor_start_s_m=start_s,actor_end_s_m=start_s+travel,
        lap_length_m=line.lap,minimum_laps=laps,maximum_actor_laps=limit,
        actor_road_wall_samples=len(samples),maximum_sample_step_m=.01)


def continuous_course_positions(line, poses, reference):
    """고정 기준에 매번 modulo하지 않고 작은 연속 이동을 누적한다. 한 표본>1m 도약은 실패다."""
    if not poses or not ev._pose_rows_valid(poses):
        return [],1
    first_s=line.unwrap(line.nearest_s(*poses[0][1:3])[0],reference)
    values,jumps=ev.progress_series(line,poses)
    return [(p['t'],first_s+p['progress_m']) for p in values],jumps


def full_course_progress(line, ego, actor, reference, actor_length):
    from bisect import bisect_right
    own,j1=continuous_course_positions(line,ego,reference)
    lead,j2=continuous_course_positions(line,actor,reference)
    if not own or not lead:
        return dict(valid=False,ego_stays_behind=False,progress_m=None,projection_jumps=j1+j2)
    times=[r[0] for r in lead]
    behind=True
    pairs=0
    for t,s in own:
        i=bisect_right(times,t)-1
        if i<0 or t-times[i]>ev.POSE_MAX_GAP_S+1e-9:
            behind=False
            continue
        pairs+=1
        behind &= s <= lead[i][1]-(ev.EGO_LENGTH_M+actor_length)/2
    return dict(valid=j1+j2==0 and pairs==len(own),ego_stays_behind=bool(behind),
        progress_m=own[-1][1]-own[0][1],projection_jumps=j1+j2,
        ego_start_s_m=own[0][1],ego_end_s_m=own[-1][1],actor_end_s_m=lead[-1][1],paired_samples=pairs)


def wheel_center_surface(road, poses, wheelbase=.135, track=.135):
    """네 바퀴 중심의 평면 표본. 타이어 접지·공식 심판 판정이 아니며 차체 road 지표도 별도로 남긴다."""
    result=dict(valid=False,samples=len(poses),max_off_wheels=None,three_plus_samples=None,
                scope='sampled planar wheel centers; not tire contact or referee judgement',
                wheelbase_m=wheelbase,track_m=track)
    if (road is None or not poses or not ev._pose_rows_valid(poses) or
            not all(ev._finite(v) and v>0 for v in (wheelbase,track))):
        return result
    surface=prep(road)
    counts=[]
    for _,x,y,yaw in poses:
        c,s=math.cos(yaw),math.sin(yaw)
        counts.append(sum(not surface.covers(Point(x+c*a-s*b,y+s*a+c*b)) for a,b in
            ((wheelbase/2,track/2),(wheelbase/2,-track/2),(-wheelbase/2,track/2),(-wheelbase/2,-track/2))))
    return {**result,'valid':True,'max_off_wheels':max(counts),'three_plus_samples':sum(c>=3 for c in counts),
            'histogram':{str(i):counts.count(i) for i in range(5)}}
