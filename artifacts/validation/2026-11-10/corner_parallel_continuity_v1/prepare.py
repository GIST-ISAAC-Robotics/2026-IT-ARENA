"""평가 전용: 바깥 차의 큰 회전 반경을 고려한 고정 동행 각본을 준비한다."""
import copy
import hashlib
import json
import math
from pathlib import Path

from validate_g1_traffic import load_config, load_centerline, actor_start_s, check_route
from traffic_corner_route import corner_geometry, corner_pose
from g1_traffic_eval import actor_travel_m
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons, rectangle

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
OLD = ROOT/'artifacts/validation/2026-11-08/observed_corner_continuity_r48_v1'


def make_schedule(geometry, approach_speed, arc_speed=.50):
    """0.1초 속도 구간으로 약0.5m/s² 전이를 만든다. 실제 상대 동역학은 아니다."""
    schedule = []
    clock = 0.
    def stage(duration, speed):
        nonlocal clock
        assert duration > 0 and speed > 0
        schedule.append(dict(from_s=clock, to_s=clock+duration, speed_mps=speed))
        clock += duration
    count = math.ceil(abs(arc_speed-approach_speed)/.05)
    transition = [approach_speed+(arc_speed-approach_speed)*(i+.5)/count for i in range(count)]
    ramp_distance = .1*sum(transition)
    stage((geometry['straight']-ramp_distance)/approach_speed, approach_speed)
    for speed in transition:
        stage(.1, speed)
    arc_begin = clock
    stage(geometry['arc']/arc_speed, arc_speed)
    arc_end = clock
    for speed in reversed(transition):
        stage(.1, speed)
    stage((1.1-ramp_distance)/approach_speed, approach_speed)
    return schedule, clock, arc_begin, arc_end


def stored_encounter(config, line, s0, case, trace, green):
    """기존 ego 궤적과 새 각본의 기하적 만남만 검사. 새 제어기 반응 예측은 아니다."""
    previous = None
    longest = span = 0.
    min_gap = float('inf')
    count = 0
    first_t = last_t = None
    wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
    for row in trace:
        elapsed = row['t']-green
        if not 0 <= elapsed <= case['duration_s']:
            continue
        x, y, yaw = corner_pose(config, line, s0, case, elapsed)
        c, s = math.cos(row['yaw']), math.sin(row['yaw'])
        dx, dy = x-row['x'], y-row['y']
        along, across = c*dx+s*dy, -s*dx+c*dy
        angle = math.degrees(wrap(row['yaw']-math.pi/2))
        difference = math.degrees(abs(wrap(yaw-row['yaw'])))
        match = 20 <= angle <= 70 and abs(along) <= .30 and .15 <= abs(across) <= .55 and difference <= 30
        if 10 <= angle <= 100 and row['y'] > 12:
            min_gap = min(min_gap, rectangle(x,y,yaw,.2,.15).distance(rectangle(row['x'],row['y'],row['yaw'],.2,.15)))
        if match:
            count += 1
            first_t = row['t'] if first_t is None else first_t
            last_t = row['t']
        if previous is not None:
            dt = row['t']-previous[0]
            span = span+dt if 0 < dt <= .20+1e-9 and match and previous[1] else 0.
            longest = max(longest,span)
        previous = row['t'], match
    return dict(longest_parallel_s=longest, matching_trace_samples=count,
                first_t=first_t, last_t=last_t, stored_corner_minimum_body_gap_m=min_gap)


def main():
    for name in ('new_track_course.json','geometry.json'):
        assert not (A/name).exists(), 'refused to overwrite preparation'
    config = load_config(OLD/'new_track_course.json')
    original_case = config['cases'][0]
    world = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
    line, scene = load_centerline(world)
    s0 = actor_start_s(config,scene,0,original_case)
    g = corner_geometry(config,line,original_case,s0)
    report = json.loads((OLD/'run1/report.json').read_bytes())
    trace = json.loads((OLD/'run1/trace.json').read_bytes())
    checks = []
    # 속도 상한 탐색이 아니라, 원래 짧았던 만남을 지속 조건으로 배치하는 사전 기하 비교다.
    for speed in (.30,.33,.35):
        case = copy.deepcopy(original_case)
        case['schedule'],case['duration_s'],begin,end = make_schedule(g,speed)
        checks.append(dict(approach_speed_mps=speed, arc_speed_mps=.5,
            arc_begin_relative_s=begin, arc_end_relative_s=end,
            **stored_encounter(config,line,s0,case,trace,report['case_window']['green_sim_s'])))
    case = copy.deepcopy(original_case)
    case['schedule'],case['duration_s'],begin,end = make_schedule(g,.33)
    case['expected'] = '오른쪽 바깥 차는 직선0.33m/s·코너0.50m/s 고정 각본. 실제 지속 병렬·출구와 비접촉을 별도로 평가한다.'
    config['cases']=[case]
    config['scope']='현재 코드의 지속 코너 병렬 대표. 상대 각본만 변경; 저장 ego 자세는 사전 만남 분석에만 사용.'
    route=check_route(config,line,case,s0,road_polygon(world/'world.sdf'),obstacle_polygons(world/'world.sdf'))
    assert math.isclose(actor_travel_m(case['schedule'],case['duration_s']),g['straight']+g['arc']+1.1,abs_tol=1e-9)
    slot=next(q for q in scene['starting_grid']['slots'] if q['index']==config['launch']['grid_slot'])
    initial=rectangle(*corner_pose(config,line,s0,case,0),.2,.15).distance(rectangle(slot['x'],slot['y'],slot['yaw_rad'],.2,.15))
    assert initial >= .08
    joins=[]
    for time in [s['to_s'] for s in case['schedule'][:-1]]:
        a,b=corner_pose(config,line,s0,case,time-1e-7),corner_pose(config,line,s0,case,time+1e-7)
        joins.append(dict(t=time,position_step_m=math.dist(a[:2],b[:2]),yaw_step_rad=abs(a[2]-b[2])))
    assert all(x['position_step_m']<1e-6 and x['yaw_step_rad']<1e-6 for x in joins)
    evidence=dict(scope='saved ego trajectory versus fixed scripted actor, no new closed-loop claim',
        initial_ego_actor_gap_m=initial, route=route, candidates=checks,
        selected_approach_mps=.33, selected_arc_mps=.5, scheduled_duration_s=case['duration_s'],
        arc_begin_s=begin,arc_end_s=end,joins=joins,
        original_trace_sha256=hashlib.sha256((OLD/'run1/trace.json').read_bytes()).hexdigest(),
        production_code_modified=False)
    (A/'new_track_course.json').write_text(json.dumps(config,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    (A/'geometry.json').write_text(json.dumps(evidence,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(evidence))


if __name__=='__main__':main()
