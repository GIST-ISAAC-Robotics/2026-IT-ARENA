"""새 직선 대표 장면 사후 평가. 원 검사기 판정(raw)을 바꾸지 않고 옆에 현행 보충 판정을 기록한다.

- raw: report.json의 passed/gates(옛 15 cm 간격·차체 도로 밖 0 포함)를 그대로 옮긴다.
- 보충: 현행 사용자 정책(최대 두 바퀴 잔디 허용)의 바퀴 노면 판정(surface.json), 접촉·간격 하계,
  B0390·단면 실제 적용, 실제 초록 뒤 진행, 속도/정지, 추월·차선 복귀·후속 전진, STOP·종료·해시·프로세스.
정답 자세는 평가에만 쓴다.
"""
from bisect import bisect_left
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SENSOR = ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def rows(path):
    with Path(path).open(encoding='utf-8') as stream:
        yield from (json.loads(line) for line in stream if line.strip())


def interpolate(data, times, t):
    i = max(1, min(bisect_left(times, t), len(times)-1))
    a, b = data[i-1:i+1]
    if not a[0] <= t <= b[0] or not 0 < b[0]-a[0] <= .12:
        raise ValueError(f'missing pose coverage at {t:.3f}')
    k = (t-a[0])/(b[0]-a[0])
    return [a[j]+k*(b[j]-a[j]) for j in (1, 2)]


def find(data, key):
    if isinstance(data, dict):
        for k, v in data.items():
            if k == key:
                yield v
            yield from find(v, key)
    elif isinstance(data, list):
        for v in data:
            yield from find(v, key)


def longest(samples, predicate):
    best = start = None
    best = 0.
    for t, value in samples:
        if predicate(value):
            start = t if start is None else start
            best = max(best, t-start)
        else:
            start = None
    return best


def main(case):
    folder = HERE/case
    run = folder/'run1'
    report, registered = read(run/'report.json'), read(folder/'registered.json')
    course = read(HERE/'new_track_course.json')
    spec = next(c for c in course['cases'] if c['name'] == case)
    offset = spec.get('actor_lateral_offset_m', 0.)
    tg, te = (report['case_window'][k] for k in ('green_sim_s', 'end_sim_s'))
    ego = list(rows(run/'ego_poses.jsonl'))
    actor = list(rows(run/'actor_poses.jsonl'))
    et, at = [e[0] for e in ego], [a[0] for a in actor]
    a0, a1 = interpolate(actor, at, tg), interpolate(actor, at, te)
    dx, dy = a1[0]-a0[0], a1[1]-a0[1]
    d = math.hypot(dx, dy)
    c, s = dx/d, dy/d

    def relative(t):
        """(t, 상대 기준 종방향[+면 앞], 중심선 기준 횡방향[+왼쪽], 출발 직선 진행)."""
        e, a = interpolate(ego, et, tg+t), interpolate(actor, at, tg+t)
        x, y = e[0]-a[0], e[1]-a[1]
        return t, c*x+s*y, -s*x+c*y+offset, c*(e[0]-a0[0])+s*(e[1]-a0[1])
    track = [relative(i*.05) for i in range(round((te-tg)*20)+1)]
    rates = [math.dist(interpolate(actor, at, tg+k), interpolate(actor, at, tg+k+1)) for k in range(round(te-tg))]

    trace = list(rows(run/'follow_decisions.jsonl'))
    header = trace[0]['data']
    reasons, entry, withdraw, complete = Counter(), None, [], []
    for event in trace[1:]:
        if event.get('kind') != 'call':
            continue
        q = event['data']
        t = q['input']['reference_s']-tg
        if not 0 <= t <= te-tg:
            continue
        detail = q['output'][2]
        moving = detail.get('moving_pass', {}) or {}
        reasons[moving.get('reason') or detail.get('reason')] += 1
        if detail.get('reason') == 'moving_pass_entry' and entry is None:
            entry = t
        if moving.get('reason') == 'aborted_to_observed_follow':
            withdraw.append(t)
        if moving.get('completed'):
            complete.append(t)

    samples = [r for r in read(run/'trace.json') if tg <= r['t'] <= te]
    moved = [r['t'] for r in samples if r['ground_speed_mps'] > .10]
    speed = [(r['t'], r['ground_speed_mps']) for r in samples if moved and r['t'] >= moved[0]]
    motion = dict(trace_rate_samples=len(samples), first_motion_after_green_s=moved[0]-tg if moved else None,
                  max_ground_speed_mps=max((r['ground_speed_mps'] for r in samples), default=None),
                  mean_ground_speed_after_motion_mps=sum(v for _, v in speed)/len(speed) if speed else None,
                  longest_below_005_after_motion_s=longest(speed, lambda v: v < .05),
                  zero_safe_command_samples_after_motion=sum(1 for r in samples if moved and r['t'] >= moved[0]
                                                             and r['safe_speed_mps'] == 0),
                  max_drive_command_mps=max((r['drive_speed_mps'] for r in samples), default=None),
                  progress_window_end_m=report['evaluation']['progress']['at_window_end_m'])

    runtime = yaml.safe_load((run/'runtime_local_sensors.yaml').read_text(encoding='utf-8'))
    expected = yaml.safe_load(SENSOR.read_text(encoding='utf-8'))
    offsets, halves = sorted(set(find(header, 'wall_offset_m'))), sorted(set(find(header, 'road_half_m')))
    gap = report['evaluation']['gap']
    surface = read(folder/'surface_run1.json')
    post = read(folder/'postflight_integrity_run1.json')
    leftover = (folder/'postflight_processes_run1.txt').read_text().strip()
    signal = report.get('official_signal_evidence', {})
    common = dict(
        run_valid=bool(report.get('run_valid')), harness_valid=bool(report.get('harness_valid')),
        piecewise_requested=report.get('piecewise_wall_requested') is True,
        piecewise_applied=set(find(header,'piecewise_wall_trial'))=={True},
        sources_unchanged=not post['changed'] and bool(report.get('source_unchanged')),
        no_leftover_processes=not leftover,
        stop_checks=all(report['stop_checks'].values()), shutdown_clean=bool(report.get('shutdown_clean')),
        b0390_runtime=runtime['camera'] == expected['camera']
            and report['local_sensor_config']['sha256'] == hashlib.sha256(SENSOR.read_bytes()).hexdigest(),
        section_505_applied=offsets == [.505], section_305_applied=halves == [.305],
        actual_green_then_motion=bool(signal.get('passed') and signal.get('start_after_actual_green')),
        no_contact=gap['actor_contact_messages_with_ego'] == 0 and gap['sampled_overlap_samples'] == 0
            and gap['sampled_finite_complete'] and gap['swept_missing_actor_intervals'] == 0
            and gap['min_swept_lower_bound_m'] > 0,
        wheel_surface_two_wheel_policy=surface['interpretations']['any']['max_off_wheels'] <= 2
            and surface['wall_wheel_samples'] == 0)
    behind = all(x < 0 for _, x, _, _ in track)
    if case == 'slow_lead':
        metrics = report['evaluation']['case_metrics']
        behavior = dict(
            actor_followed_schedule=all(.25 <= v <= .35 for v in rates),
            raw_slow_lead_progress=report['gates'].get('progress'), raw_no_permanent_stall=report['gates'].get('no_permanent_stall'),
            stays_behind_lead=behind, moving_pass_off=header['execution_config'].get('moving_pass_trial') is False)
        extra = dict(case_metrics=metrics, max_abs_lateral_m=max(abs(y) for _, _, y, _ in track),
                     final_relative=track[-1])
    else:
        first = None
        returned = 0.
        for t, x, y, p in track:
            if complete and t >= complete[0] and x >= .20 and abs(y) <= .06:
                first = p if first is None else first
                returned = max(returned,p-first)
            else:
                first = None
        behavior = dict(
            moving_pass_on=header['execution_config'].get('moving_pass_trial') is True,
            handoff_execution=bool(report.get('handoff_execution', {}).get('passed')),
            actor_followed_schedule=all(.25 <= v <= .35 for v in rates),
            entered=entry is not None,
            started_behind=track[0][1] < -.20,
            observed_beside=any(abs(x) <= .20 for _,x,_,_ in track),
            finished_ahead=track[-1][1] >= .20,
            completed_return_and_progress=bool(complete) and returned >= .50,
            no_permanent_stop=motion['longest_below_005_after_motion_s'] <= .50)
        extra = dict(entry_after_green_s=entry,pass_completed_times_s=complete,
            post_return_progress_m=returned,final_relative=track[-1],
            max_abs_lateral_m=max(abs(y) for _,_,y,_ in track))
    result = dict(
        scope='single registered scene on the new start straight, evaluation only; not full-course or real-car evidence',
        raw=dict(passed=report['passed'], gates=report['gates'], behavior_gates_passed=report.get('behavior_gates_passed'),
                 note='옛 등록 판정(15 cm/차체 도로 밖 0 포함)을 수정 없이 기록'),
        common=common, behavior=behavior, motion=motion, extra=extra, actor_1s_mps=rates, reasons=dict(reasons),
        gap=dict(min_sampled_m=gap['min_sampled_gap_m'], min_swept_lower_bound_m=gap['min_swept_lower_bound_m'],
                 contact_messages=gap['actor_contact_messages_with_ego']),
        surface=dict(max_off_wheels={k: v['max_off_wheels'] for k, v in surface['interpretations'].items()},
                     wall_wheel_samples=surface['wall_wheel_samples']),
        section=dict(wall_offset_m=offsets, road_half_m=halves), stop=report['stop_checks'],
        signal=signal)
    result['supplementary_all_true'] = all(common.values()) and all(v for v in behavior.values() if isinstance(v, bool))
    (folder/'assessment_run1.json').write_text(json.dumps(result, ensure_ascii=False, indent=1, default=str)+'\n',
                                          encoding='utf-8')
    print(json.dumps(dict(raw_passed=report['passed'], supplementary=result['supplementary_all_true'],
                          common=common, behavior=behavior), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
