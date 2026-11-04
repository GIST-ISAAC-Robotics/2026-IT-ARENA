"""장착 후보와 화면 명령의 사후 평가. 정답 입력은 이 평가기에만 사용한다."""
from bisect import bisect_right
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from race_ops_eval import Course
from g1_traffic_eval import stop_pose_evidence
from surface_contact_eval import SurfaceMap, evaluate_pose, analyze_series


def lines(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def main():
    run = HERE/'run1'
    report = json.loads((run/'report.json').read_bytes())
    audit = json.loads((HERE/'run1_source_audit.json').read_bytes())
    history = json.loads((run/'ops_history.json').read_bytes())
    steps = {r['step']: r for r in report['steps']}
    poses = [tuple(r[:4]) for r in lines(run/'ego_poses.jsonl')]
    start = steps.get('browser_visual_started')
    stop = steps.get('browser_stop_observed')
    route = steps.get('camera_route_reached')
    begin = start['truth'][0] if start else poses[0][0]
    end = stop['truth'][0] if stop else poses[-1][0]
    driving = [p for p in poses if begin <= p[0] <= end]
    calls = []
    for row in lines(run/'follow_decisions.jsonl'):
        if row.get('kind') == 'call':
            data = row['data']
            t = data['input'].get('reference_s')
            if t is not None:
                calls.append((t, data['output'][1]))
    times = [c[0] for c in calls]
    surface = SurfaceMap.from_world(ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf')
    evaluated, steering_missing = [], 0
    for t,x,y,yaw in driving:
        i = bisect_right(times,t)-1
        steering_missing += i < 0 or t-times[i] > .2
        evaluated.append(evaluate_pose(surface,x,y,yaw,calls[i][1] if i>=0 else 0.))
    surface_result = analyze_series([p[0] for p in driving], evaluated)
    surface_result.update(steering_source='제어기 목표 조향. 실제 바퀴 관절각/접지력 아님.', steering_missing_samples=steering_missing)
    (HERE/'run1_surface.json').write_text(json.dumps(surface_result,ensure_ascii=False,indent=2),encoding='utf-8')
    course = Course()
    markers, sides = {}, []
    for row in history:
        data = row['driving']; t = data.get('sim_s')
        if t is None:
            continue
        for ident in data.get('marker_ids') or []:
            markers.setdefault(str(ident),dict(first_sim_s=t, first_wall_s=row['wall']))
        if not sides or sides[-1]['side'] != data.get('side'):
            sides.append(dict(sim_s=t,side=data.get('side')))
    commands = stop.get('commands',[]) if stop else []
    browser_stops = [c for c in commands if c.get('op')=='stop' and not c.get('request','').startswith('eval-')]
    automatic_stops = [c for c in commands if c.get('op')=='stop' and c.get('request','').startswith('eval-')]
    stop_evidence = stop_pose_evidence(poses,stop['truth'][0],.2) if stop else None
    model = ET.parse(run/'runtime_vehicle.sdf').getroot().find('model')
    camera_pose = list(map(float,model.findtext("link[@name='b0183_link']/pose").split()))
    speed = [math.dist(a[1:3],b[1:3])/(b[0]-a[0]) for a,b in zip(driving,driving[1:]) if b[0]>a[0]]
    branch_samples = sum(course.in_branch(p[1],p[2]) for p in driving)
    gates = dict(completed=report.get('completed') is True, runtime_exit=audit['runtime_exit']==0,
                 visual_start=bool(start and start.get('source')=='visual'),
                 reached_38m=bool(route and route['progress_s_m']>=38.),
                 markers_20_30={'20','30'}<=markers.keys(), main_course=branch_samples==0,
                 camera_pitch=abs(camera_pose[4]+math.radians(20))<1e-10,
                 selected_profile=audit.get('applied_profile_matches') is True,
                 browser_stop_before_fallback=bool(browser_stops) and not automatic_stops,
                 browser_policy_accepted=any(c.get('stage')=='policy_accepted' for c in browser_stops),
                 stopped=bool(stop_evidence and stop_evidence['passed']),
                 stop_mcu=bool(stop and stop.get('mcu')=='ESTOP_LATCHED'),
                 wheels_within_wall=surface_result['wall_wheel_samples']==0,
                 at_most_two_grass_wheels=all(r['max_off_wheels']<=2 for r in surface_result['interpretations'].values()),
                 sample_coverage=surface_result['sample_gaps_over_limit']==0 and steering_missing==0,
                 source_unchanged=report.get('source_unchanged') is True and not audit['source_mismatches'],
                 cleanup=not report.get('remaining_test_pids') and (report.get('gazebo_stop_service') or {}).get('verified') is True
                      and all(not c['forced'] for c in report.get('auxiliary_cleanup',[])))
    result = dict(passed=all(gates.values()), gates=gates, failed=[k for k,v in gates.items() if not v],
                  raw_report_passed=report.get('passed'), raw_error=report.get('error'),
                  scope='상향20도 후보·선두index2·단독 대표구간·가상MCU. 실물/다른선두칸/높은앞차 가림 미검증. 접촉 센서는 없으며 노면·벽 표본과 구별한다.',
                  start=start, route=route, camera_pose=camera_pose, first_markers=markers, side_changes=sides,
                  body_peak_mps=max(speed,default=None), branch_samples=branch_samples,
                  browser_stops=browser_stops, stop=stop_evidence, source_audit=audit,
                  exit_codes=report.get('exit_codes'), surface=surface_result)
    (HERE/'run1_analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('passed','failed','raw_error','body_peak_mps','branch_samples','first_markers','side_changes')},ensure_ascii=False))


if __name__ == '__main__':
    main()
