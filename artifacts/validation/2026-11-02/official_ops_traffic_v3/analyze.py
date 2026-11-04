"""운영·교통 결합 사후 평가. 정답 자세는 이 파일에서만 사용한다."""
from bisect import bisect_right
from collections import Counter
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
from race_ops_eval import Course
from g1_traffic_eval import sampled_gaps, swept_gap_lower_bounds, rectangle, stop_pose_evidence
from surface_contact_eval import SurfaceMap, evaluate_pose, analyze_series


def lines(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def actor_rows(path):
    rows = []
    for data in lines(path):
        for pose in data.get('pose', []):
            if pose.get('name') != 'eval_g1b_actor':
                continue
            stamp = pose.get('header', data.get('header', {})).get('stamp', {})
            t = float(stamp.get('sec', 0))+float(stamp.get('nsec', 0))*1e-9
            p, q = pose['position'], pose['orientation']
            yaw = math.atan2(2*(q.get('w', 1)*q.get('z', 0)+q.get('x', 0)*q.get('y', 0)),
                             1-2*(q.get('y', 0)**2+q.get('z', 0)**2))
            if not rows or t > rows[-1][0]:
                rows.append((t, p.get('x', 0), p.get('y', 0), yaw))
    return rows


def main():
    run = Path(sys.argv[1])
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    steps = report['steps']
    result = dict(completed=report.get('completed', False), error=report.get('error'),
                  exit_codes=report.get('exit_codes'), source_unchanged=report.get('source_unchanged'),
                  remaining_pids=report.get('remaining_test_pids'),
                  scope='공식 월드의 단일 정적 상대·재배치 대역·가상 MCU. 전체 본선/공용 Wi-Fi/실차 검증 아님.')
    resumes = [s for s in steps if s['step'] == 'resume_closed']
    if not resumes:
        result.update(passed=False, failed=['resume_not_closed'])
    else:
        start = resumes[-1]
        result['resume'] = start
        begins = start['truth'][0]
        ends = [s for s in steps if s['step'] == 'parked_progress']
        last_drive = ends[-1]['truth'][0] if ends else math.inf
        all_poses = [tuple(row[:4]) for row in lines(run/'ego_poses.jsonl')]
        poses = [p for p in all_poses if begins <= p[0] <= last_drive]
        actor = actor_rows(run/'actor_poses.jsonl')
        calls, all_calls = [], []
        boundary_checks = boundary_bad = 0
        for e in lines(run/'follow_decisions.jsonl'):
            if e.get('kind') != 'call':
                continue
            data = e['data']; inp, output = data['input'], data['output']
            t = inp.get('reference_s')
            if t is None:
                continue
            all_calls.append((t, output[0], output[1], output[2]))
            if start['last']['boundary_s'] <= t <= begins+2:
                boundary_checks += 1
                boundary_bad += inp['scan']['sec']+inp['scan']['nanosec']*1e-9 < start['last']['boundary_s']
            if begins <= t <= last_drive:
                calls.append((t, output[0], output[1], output[2]))
        config = json.loads((ROOT/'config/tests/official_traffic_v1.json').read_text(encoding='utf-8'))
        gaps = sampled_gaps(poses, actor, .2, .15, .05)
        swept = swept_gap_lower_bounds(poses, actor, .2, .15, .01,
                                      config['acceptance']['swept_max_curvature_per_m'], .05)
        result['gap'] = dict(min_sampled_m=min((r['gap_m'] for r in gaps if r['gap_m'] is not None), default=None),
                            min_swept_m=min((r['lower_bound_m'] for r in swept if r['lower_bound_m'] is not None), default=None),
                            overlap_samples=sum((r['overlap_area_m2'] or 0)>1e-9 for r in gaps),
                            missing_sampled=sum(r['gap_m'] is None for r in gaps),
                            missing_swept=sum(r['lower_bound_m'] is None for r in swept))
        contacts = [c for data in lines(run/'contacts.jsonl') for c in data.get('contact', [])
                    if 'arena_car' in str(c.get('collision1', {}))+str(c.get('collision2', {}))]
        surface = SurfaceMap.from_world(ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf')
        times = [c[0] for c in all_calls]; evaluated = []; steering_missing = 0
        # 같은 제어기 기록의 목표 조향을 사용한다. 관절각/접지력 측정으로 부르지 않는다.
        for t,x,y,yaw in poses:
            index = bisect_right(times,t)-1
            if index < 0 or t-times[index] > .2:
                steering_missing += 1
            steering = all_calls[index][2] if index >= 0 else 0.
            evaluated.append(evaluate_pose(surface,x,y,yaw,steering))
        surface_result = analyze_series([p[0] for p in poses],evaluated)
        surface_result.update(steering_source='판단 기록의 목표 조향, 실제 관절각 아님',steering_missing_samples=steering_missing)
        (run.parent/(run.name+'_surface.json')).write_text(json.dumps(surface_result,ensure_ascii=False,indent=1),encoding='utf-8')
        course = Course()
        final_s = course.nearest(*poses[-1][1:3])[0] if poses else None
        states = Counter(c[3].get('handoff_state') for c in calls)
        finals = [p for p in poses if p[0] >= poses[-1][0]-2.]
        outside_final = sum(rectangle(x,y,yaw,.20,.15).difference(surface.road).area > 1e-6 for _,x,y,yaw in finals)
        stops = [s for s in steps if s['step'] == 'browser_stop_observed']
        stop = stop_pose_evidence(all_poses,stops[-1]['truth'][0],.2) if stops else None
        result.update(calls=len(calls), states=dict(states), final_s=final_s, contact_messages_with_ego=len(contacts),
                      contact_topic=report.get('contact_topic_info'), final_body_outside_samples=outside_final,
                      surface=surface_result, stop=stop, boundary_checks=boundary_checks,boundary_bad=boundary_bad)
        g=result['gap']
        gates=dict(completed=result['completed'], permitted=start['last']['outcome']=='permitted',
                   reached_exit=bool(ends and final_s is not None and final_s>=6.5),
                   executed_bypass=states['BYPASS']>0, final_follow=bool(calls and calls[-1][3].get('handoff_state')=='FOLLOW'),
                   body_returned=bool(finals) and outside_final==0, no_overlap=g['overlap_samples']==0,
                   gap_evidence=bool(poses and actor and gaps and swept) and g['missing_sampled']==g['missing_swept']==0,
                   minimum_gap=g['min_sampled_m'] is not None and g['min_sampled_m']>=.08,
                   minimum_swept_gap=g['min_swept_m'] is not None and g['min_swept_m']>=.08,
                   no_observed_contact=not contacts and 'gz.msgs.Contacts' in report.get('contact_topic_info',''),
                   wheel_wall=surface_result['wall_wheel_samples']==0,
                   at_most_two_wheels=all(r['max_off_wheels']<=2 for r in surface_result['interpretations'].values()),
                   samples_complete=surface_result['sample_gaps_over_limit']==0 and steering_missing==0,
                   boundary=boundary_checks>0 and boundary_bad==0,
                   browser_stop=bool(stops and stop and stop['passed']), source_unchanged=result['source_unchanged'] is True,
                   shutdown=(not result['remaining_pids'] and (report.get('gazebo_stop_service') or {}).get('verified') is True
                             and all(not row['forced'] for row in report.get('auxiliary_cleanup',[]))))
        result.update(gates=gates,passed=all(gates.values()),failed=[k for k,v in gates.items() if not v])
    (run.parent/(run.name+'_analysis.json')).write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('resume','surface','contact_topic')},ensure_ascii=False))


if __name__=='__main__':
    main()
