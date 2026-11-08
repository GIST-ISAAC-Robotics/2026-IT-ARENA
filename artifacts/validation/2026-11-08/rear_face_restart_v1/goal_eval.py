"""경기 목표 평가. 원본 report.json의 판정을 고치지 않는 별도 사후 평가다. 인자: 실행 폴더."""
import json
import math
from pathlib import Path
import sys

import numpy as np

from audit_local_pursuit_road import road_polygon
from g1_traffic_eval import rectangle

MINIMUM_GAP_M = .08
MAXIMUM_STALL_S = 2.5
ROAD_AREA_TOLERANCE_M2 = 1e-6  # 기존 도로 이탈 평가와 같은 면적 허용치
FINAL_WINDOW_S = 2.  # 복귀는 창 끝 이 시간 전체에서 유지돼야 한다


def poses(path):
    rows = [json.loads(line) for line in path.read_bytes().splitlines()]
    return rows


def actor_rows(run):
    return [(row[0], row[1], row[2]) for row in poses(run/'actor_poses.jsonl')]


def decisions(run, green):
    out = []
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        e = json.loads(line)
        if e['kind'] != 'call':
            continue
        i, o = e['data']['input'], e['data']['output']
        out.append((i['reference_s']-green, o[0], o[2].get('handoff_state'), o[2].get('behavior'), o[2].get('reason')))
    return out


def centerline(world):
    data = np.genfromtxt(world/'centerline.csv', delimiter=',', names=True)
    xy = np.column_stack((data['x_m'], data['y_m']))
    s = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    return xy, s


def project(xy, s, x, y):
    index = int(np.argmin(np.hypot(xy[:, 0]-x, xy[:, 1]-y)))
    a, b = xy[max(index-1, 0)], xy[min(index+1, len(xy)-1)]
    tangent = (b-a)/np.linalg.norm(b-a)
    delta = np.array([x, y])-xy[index]
    return float(s[index]+delta@tangent), float(-delta[0]*tangent[1]+delta[1]*tangent[0])


def unwrap(values, lap):
    out = [values[0]]
    for value in values[1:]:
        while value-out[-1] < -lap/2:
            value += lap
        while value-out[-1] > lap/2:
            value -= lap
        out.append(value)
    return np.array(out)


def final_body_on_road(trace, t, road):
    """창 끝 표본의 0.20×0.15 m 회전 차체를 도로 다각형과 대조한다. 10 Hz 표본이며 연속 증명은 아니다."""
    rows = [row for row, value in zip(trace, t) if value >= t[-1]-FINAL_WINDOW_S]
    outside = sum(rectangle(row['x'], row['y'], row['yaw'], .20, .15).difference(road).area > ROAD_AREA_TOLERANCE_M2 for row in rows)
    return len(rows), int(outside)


def evaluate(run, world):
    report = json.loads((run/'report.json').read_bytes())
    surface = json.loads((run.parent/'surface_run1.json').read_bytes())
    evaluation = report['evaluation']
    green = evaluation['t_green']
    name = report['case']
    trace = json.loads((run/'trace.json').read_bytes())
    calls = decisions(run, green)
    xy, s = centerline(world)
    lap = float(s[-1]+np.linalg.norm(xy[0]-xy[-1]))
    t = np.array([r['t'] for r in trace])-green
    ego = [project(xy, s, r['x'], r['y']) for r in trace]
    ego_s, ego_l = unwrap([v[0] for v in ego], lap), np.array([v[1] for v in ego])
    speed = np.array([r['ground_speed_mps'] for r in trace])
    actor = actor_rows(run)
    at = np.array([a[0] for a in actor])-green
    projected = [project(xy, s, a[1], a[2]) for a in actor]
    actor_s, actor_l = unwrap([v[0] for v in projected], lap), np.array([v[1] for v in projected])
    while actor_s[0]-ego_s[0] > lap/2:
        actor_s -= lap
    while actor_s[0]-ego_s[0] < -lap/2:
        actor_s += lap
    keep = (t >= 0)
    a_s = np.interp(t, at, actor_s)
    a_l = np.interp(t, at, actor_l)
    body = a_s-ego_s-.20
    final = np.array([c[2] for c in calls])
    call_t = np.array([c[0] for c in calls])
    bypass_times = call_t[final == 'BYPASS']
    bypass_s = float(len(bypass_times))*.02
    gap = evaluation['gap']
    wheels = surface['interpretations']
    stop = report.get('stop_checks', {})
    samples, outside = final_body_on_road(trace, t, road_polygon(world/'world.sdf'))
    on_road = samples > 0 and outside == 0
    final_rows = t >= t[-1]-FINAL_WINDOW_S
    sampled, swept = gap['min_sampled_gap_m'], gap.get('min_swept_lower_bound_m')
    gates = dict(run_valid=report.get('run_valid') is True, harness_valid=report.get('harness_valid') is True,
                 stop_and_shutdown=bool(stop) and all(stop.values()) and report.get('shutdown_clean') is True,
                 source_unchanged=report.get('source_unchanged') is True, no_contact=gap['actor_contact_messages_with_ego'] == 0,
                 no_overlap=gap['sampled_overlap_samples'] == 0, no_wall_wheel=surface.get('wall_wheel_samples') == 0,
                 surface_evidence=(surface.get('samples', 0) > 0 and surface.get('sample_gaps_over_limit') == 0
                                   and set(wheels) == {'any', 'center', 'full'}),
                 at_most_two_wheels=bool(wheels) and all(v.get('max_off_wheels', 5) <= 2 for v in wheels.values()),
                 minimum_gap=sampled is not None and sampled >= MINIMUM_GAP_M,
                 minimum_swept_gap=(swept is not None and gap.get('swept_missing_actor_intervals') == 0
                                    and swept >= MINIMUM_GAP_M))
    moving = t >= 0
    stall, span = 0., 0.
    for a, b, v in zip(t[moving], t[moving][1:], speed[moving]):
        span = span+(b-a) if v < .05 else 0.
        stall = max(stall, span)
    metrics = dict(minimum_gap_m=gap['min_sampled_gap_m'], final_progress_m=float(ego_s[-1]-ego_s[0]),
                   bypass_output_s=bypass_s, first_bypass_s=float(bypass_times[0]) if len(bypass_times) else None,
                   last_bypass_s=float(bypass_times[-1]) if len(bypass_times) else None,
                   longest_stationary_s=stall, peak_speed_mps=float(speed.max()), final_state=calls[-1][2],
                   final_lateral_m=float(ego_l[-1]), final_window_max_centerline_offset_m=float(np.max(np.abs(ego_l[final_rows]))),
                   final_window_body_samples=samples, final_window_body_outside_samples=outside,
                   final_window_progress_m=float(ego_s[final_rows][-1]-ego_s[final_rows][0]),
                   final_window_min_speed_mps=float(np.min(speed[final_rows])), last_command_mps=float(calls[-1][1]),
                   minimum_swept_gap_m=swept, original_passed=report.get('passed'), original_gates=evaluation.get('gates'))
    actor_speed = np.gradient(actor_s, at)
    passed = np.flatnonzero(keep & (ego_s-.10 > a_s+.10))
    metrics['ego_passed_actor_s'] = float(t[passed[0]]) if len(passed) else None
    scene = {}
    if name == 'persistent_block':
        hold = (at > 3.5) & (at <= report['case_window']['end_sim_s']-green-.1)
        scene = dict(actor_stopped=bool(np.max(np.abs(actor_speed[hold])) < .02), executed_bypass=bypass_s >= 1.)
        rejoined = len(passed) > 0 and on_road
        gates.update(passed_actor=bool(len(passed)), rejoined_main=bool(rejoined), final_follow=calls[-1][2] == 'FOLLOW',
                     no_long_stall=bool(stall <= MAXIMUM_STALL_S),
                     post_return_progress=metrics['final_window_progress_m'] >= .50)
    elif name == 'moving_stop_restart':
        restart = float(at[np.flatnonzero((at > 7.5) & (actor_speed > .1))[0]])
        after = t >= restart
        active = np.flatnonzero(after & (speed > .1))
        delay = float(t[active[0]]-restart) if len(active) else None
        after_progress = float(ego_s[-1]-np.interp(restart,t,ego_s))
        tt=t[after]; vv=speed[after]; span=longest=0.
        for a,b,v in zip(tt,tt[1:],vv):
            span=span+(b-a) if v<.05 else 0.
            longest=max(longest,span)
        stopped=(at>=3.5)&(at<=7.5)
        advancing=((at>=.5)&(at<=2.5))|((at>=8.5)&(at<=17.5))
        windows=[float(np.interp(start+1.,at,actor_s)-np.interp(start,at,actor_s)) for start in [.5,1.,1.5]+[8.5+.5*k for k in range(17)]]
        sample_gap=float(max(np.diff(at)))
        metrics.update(actor_one_second_speeds_mps=windows,actor_maximum_pose_gap_s=sample_gap)
        scene=dict(actor_stopped=bool(stopped.any() and np.max(np.abs(actor_speed[stopped]))<.02),
                   actor_advanced=bool(sample_gap<=.12 and windows and all(v>.45 for v in windows)),
                   actor_restart_s=restart, observed_restart_on_schedule=abs(restart-8.)<=.12)
        metrics.update(restart_delay_s=delay,progress_after_restart_m=after_progress,
                       longest_stationary_after_restart_s=longest)
        gates.update(rejoined_main=on_road,final_follow=calls[-1][2]=='FOLLOW',
                     restart=delay is not None and delay<=2.5,progress_after_restart=after_progress>=1.,
                     no_long_stall_after_restart=longest<=2.5,
                     post_return_progress=metrics['final_window_progress_m']>=.50)
    elif name == 'restart_during_bypass':
        start = float(at[np.flatnonzero((at > 4.) & (actor_speed > .1))[0]])
        state = [c[2] for c in calls if abs(c[0]-start) <= .03]
        scene = dict(actor_restart_s=start, bypass_at_restart=bool(state) and all(v == 'BYPASS' for v in state),
                     ego_not_yet_past=bool(np.interp(start, t, ego_s)-.10 <= np.interp(start, at, actor_s)+.10))
        after = t > start
        exit_s = float(ego_s[0])+11.79  # 그리드 3에서 첫 U자 출구 s=9.8까지
        leave = call_t[(call_t > start) & (final != 'BYPASS')]
        metrics.update(rejoin_after_restart_s=float(leave[0]-start) if len(leave) else None)
        span, longest = 0., 0.
        for a, b, v in zip(t[after], t[after][1:], speed[after]):
            span = span+(b-a) if v < .05 else 0.
            longest = max(longest, span)
        gates.update(left_bypass=len(leave) > 0, reached_corner_exit=bool(ego_s[-1] >= exit_s),
                     rejoined_main=on_road, final_follow=calls[-1][2] == 'FOLLOW', no_long_stall=bool(longest <= MAXIMUM_STALL_S))
    elif name == 'close_cut_in':
        inside = np.flatnonzero((t > 0) & (np.abs(a_l-ego_l) < .15))
        first = int(inside[0]) if len(inside) else None
        scene = dict(body_gap_at_entry_m=float(body[first]) if first is not None else None,
                     ego_speed_at_entry_mps=float(speed[first]) if first is not None else None, entry_s=float(t[first]) if first is not None else None)
        scene['established'] = first is not None and scene['body_gap_at_entry_m'] <= .8 and scene['ego_speed_at_entry_mps'] >= .5
        after = t[first:] if first is not None else t
        decel = np.gradient(speed, t)
        window = (t >= (t[first] if first is not None else 0))
        metrics.update(minimum_speed_after_entry_mps=float(speed[window].min()), peak_decel_mps2=float(-np.min(decel[window])))
        span, longest = 0., 0.
        for a, b, v in zip(after, after[1:], speed[window]):
            span = span+(b-a) if v < .05 else 0.
            longest = max(longest, span)
        gates.update(stays_behind=not len(passed), no_long_stop=bool(longest <= 1.))
        metrics['actor_surface_checked'] = '도로+20 cm 잔디 폭 안(잔디 시작 장면). 원본 report의 actor_stays_onroad는 이 뜻이다.'
    conditions = [v for v in scene.values() if isinstance(v, bool)]
    established = bool(conditions) and all(conditions)
    result = dict(scope='경기 목표 평가. 원본 판정을 대신하지 않는다.', scene=name, scene_conditions=scene,
                  established=established, gates=gates, passed=established and all(gates.values()), metrics=metrics)
    output = run.parent/f'{run.name}_goal.json'
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2, default=float), encoding='utf-8', newline='\n')
    print(json.dumps({k: result[k] for k in ('scene', 'established', 'passed', 'scene_conditions', 'gates')}, ensure_ascii=False, default=float))
    print(json.dumps({k: v for k, v in metrics.items() if k != 'original_gates'}, ensure_ascii=False, default=float))


if __name__ == '__main__':
    evaluate(Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve())
