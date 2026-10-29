"""기록된 다랩/간격/정지 판정과 랩별 표본 요약. 원시 판정은 수정하지 않는다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def lap_windows(rows, first, last, lap, required):
    """평가 시간 안의 순진행으로 요약한다. 밀집 자세 기반 공식 판정은 별도다."""
    rows = [row for row in rows if first <= row['t'] <= last]
    if not rows:
        return dict(sampled_origin_m=None, sampled_origin_time_s=None, windows=[])
    origin = rows[0]['progress_m']
    windows = []
    began = first
    for i in range(1, required+1):
        crossed = next((r for r in rows if r['progress_m']-origin >= i*lap), None)
        if crossed is None:
            break
        section = [r for r in rows if began <= r['t'] <= crossed['t']]
        windows.append(dict(lap=i, begin_s=began, reached_s=crossed['t'],
            duration_s=crossed['t']-began, progress_boundary_m=i*lap,
            maximum_sampled_speed_mps=max(r['ground_speed_mps'] for r in section),
            zero_safe_command_samples=sum(r['safe_speed_mps']==0 for r in section),
            controller_reasons=dict(Counter(str(r['autonomy'].get('reason')) for r in section))))
        began = crossed['t']
    return dict(sampled_origin_m=origin, sampled_origin_time_s=rows[0]['t'], windows=windows)


def sampled_side_entries(rows, first, last):
    """반복 조우의 관측 표본을 남긴다. 정답 물체 신원이나 누락된 전이는 추정하지 않는다."""
    previous = {}
    entries = []
    for row in rows:
        if not first <= row['t'] <= last:
            continue
        selector = row.get('pass_release_trial') or {}
        sides = selector.get('selector_sides') or {}
        for side in ('left', 'right'):
            state = sides.get(side)
            if not isinstance(state, dict) or type(state.get('present')) is not bool:
                previous.pop(side, None)
                continue
            old = previous.get(side)
            contiguous = old is not None and 0 <= row['t']-old['t'] <= .15
            if state['present'] and (not contiguous or not old['present']):
                entries.append(dict(t=row['t'], progress_m=row['progress_m'], side=side,
                    preceded_by_observed_absence=bool(contiguous and not old['present']),
                    passed=state.get('passed'), clear=state.get('clear'),
                    selector_mode=selector.get('selector_mode'),
                    previous_pass_not_reused=state.get('passed') is False and state.get('clear') == 0))
            previous[side] = dict(t=row['t'], present=state['present'])
    return entries


def summarize(run):
    paths = [run/name for name in ('report.json','trial_overlay.json','trajectory.jsonl','follow_trace_replay.json')]
    before = {p.name:sha(p) for p in paths}
    report = json.loads(paths[0].read_text(encoding='utf-8'))
    overlay = json.loads(paths[1].read_text(encoding='utf-8'))
    replay = json.loads(paths[3].read_text(encoding='utf-8'))
    first = report['case_window']['green_sim_s']; last = report['case_window']['end_sim_s']
    lap = report['route_check']['lap_length_m']
    required = report['route_check']['minimum_laps']
    with paths[2].open(encoding='utf-8') as stream:
        rows = list(map(json.loads, stream))
    diagnostic = lap_windows(rows, first, last, lap, required)
    failed = [f'primary.{key}' for key,ok in report.get('gates',{}).items() if not ok]
    actors = report.get('additional_actors',{}).get('actors',{})
    for name,actor in actors.items():
        failed.extend(f'{name}.{key}' for key,ok in actor.get('gates',{}).items() if not ok)
    return dict(scope='evaluation-only net course progress and sampled lap windows; not onboard lap counting',
        original_report_passed=report['passed'], original_run_valid=report['run_valid'],
        failed_gates=failed, requested_route=overlay['route'], route_check=report['route_check'],
        progress=report['evaluation'].get('full_course_progress'), lap_windows=diagnostic.pop('windows'),
        sampled_lap_basis=dict(**diagnostic, first_s=first, last_s=last,
            authoritative=False, note='sampled window diagnostics; original dense-pose report decides completion'),
        sampled_side_entries=sampled_side_entries(rows, first, last),
        side_entry_scope='observed state transitions only; no object identity or complete event coverage',
        stop=report.get('stop'), shutdown=report.get('shutdown_audit'),
        actor_tail_drain=report.get('actor_tail_drain'),
        runtime_contract=overlay.get('runtime_contract'), source_unchanged=overlay.get('source_unchanged'),
        replay={k:v for k,v in replay.items() if k!='outcomes'},
        input_sha256=before, input_unchanged=all(sha(p)==before[p.name] for p in paths))


if __name__ == '__main__':
    run = Path(sys.argv[1])
    result = summarize(run)
    output = run.parent/(run.name+'_summary.json')
    with output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps({k:result[k] for k in ('original_report_passed','original_run_valid','failed_gates','progress')},ensure_ascii=True))
