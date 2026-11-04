"""장착 비교 실행의 공통 평가 요약. 평가 정답 자세는 집계에만 쓰며 제어 입력이 아니다.

사용: evaluate.py solo|traffic 이름 → 이름_eval.json. 원본 report.json 판정은 바꾸지 않는다.
"""
from collections import Counter
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))


def jsonl(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def decisions(run, begin, end):
    """follow 판단 0명령 시간(50 Hz 호출 1회=0.02 s)과 사유."""
    zero, calls, speed = Counter(), 0, []
    path = run/'follow_decisions.jsonl'
    if not path.exists():
        return None
    with path.open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']
        for line in stream:
            event = json.loads(line)
            if event['kind'] != 'call':
                continue
            t = event['data']['input']['reference_s']
            if not begin <= t <= end:
                continue
            v, _, detail = event['data']['output']
            calls += 1
            speed.append(v)
            if v <= .001:
                zero[detail.get('reason')] += .02
    mask = header['motion_config']
    return dict(calls=calls, zero_command_s=round(sum(zero.values()), 2),
                zero_by_reason={k: round(v, 2) for k, v in zero.most_common()},
                mean_command_mps=float(np.mean(speed)) if speed else None,
                lidar_x_m=mask.get('lidar_x_m'), lidar_y_m=mask.get('lidar_y_m'))


def weighted(times, labels, begin, end):
    out = Counter()
    for (a, la), (b, _) in zip(zip(times, labels), list(zip(times, labels))[1:]):
        if begin <= a < end:
            out[la] += min(b, end)-a
    return {str(k): round(v, 3) for k, v in out.most_common()}


def common(run):
    # 11/4 첫 두 raw 실행은 검사기 옵션의 Path 직렬화 결함으로 report.json이 없다(실행 기록은 있음).
    report = json.loads((run/'report.json').read_bytes()) if (run/'report.json').exists() else {
        'error': 'report.json missing (validator serialization defect)'}
    sensors = run/'runtime_local_sensors.yaml'
    sectors = run/'lidar_blind_sectors.json'
    return report, dict(
        sensor_profile_sha256=__import__('hashlib').sha256(sensors.read_bytes()).hexdigest() if sensors.exists() else None,
        blind_sectors_deg=json.loads(sectors.read_text()) if sectors.exists() else None,
        runtime_posts=(run/'runtime_vehicle.sdf').read_text(encoding='utf-8').count('c1_mount_post_')
            if (run/'runtime_vehicle.sdf').exists() else None,
        error=report.get('error'), passed=report.get('passed'))


def solo(name):
    run = HERE/name
    report, out = common(run)
    metrics_path = HERE/f'{name}_metrics.json'
    metrics = json.loads(metrics_path.read_bytes()) if metrics_path.exists() and metrics_path.stat().st_size else {}
    rows = jsonl(run/'trajectory.jsonl')
    green = [r for r in rows if r['light'] == 'green']
    begin = green[0]['time'] if green else math.inf
    lap = metrics.get('lap_s_from_first_green_sample')
    end = begin+lap if lap else (green[-1]['time'] if green else -math.inf)
    t = [r['time'] for r in green]
    out.update({k: report.get(k) for k in ('route_complete', 'completed_laps', 'progress_m', 'collision_samples',
                                           'stop_stable', 'false_start', 'peak_ground_speed_mps', 'marker_ids')})
    out['lap_s'] = lap
    out['metrics'] = {k: metrics.get(k) for k in ('max_ground_speed_mps', 'max_1s_mean_speed_mps', 'stopped_s', 'error')
                      if k in metrics}
    out['safety_reason_s'] = weighted(t, [r['safety']['reason'] for r in green], begin, end)
    out['front_observation_s'] = weighted(t, [((r['safety'].get('front_observation') or {}).get('reason')) for r in green], begin, end)
    limited = [r for r in green if begin <= r['time'] < end
               and r.get('safe_drive_speed_mps', 0) < r['autonomy'].get('speed_command_mps', 0)-.05]
    out['safety_limited_samples'] = len(limited)
    out['decisions'] = decisions(run, begin, end)
    # 출발 반응: 첫 초록 뒤 차체가 0.1 m/s를 넘는 시각.
    moving = next((r['time'] for r in green if math.hypot(r['dynamics']['truth_longitudinal_mps'],
                                                          r['dynamics']['truth_lateral_mps']) >= .1), None)
    out['first_move_after_green_s'] = None if moving is None else moving-begin
    first = green[:25]
    out['first_green_safety'] = Counter(r['safety']['reason'] for r in first).most_common()
    return out


def traffic(name):
    run = HERE/name
    report, out = common(run)
    evaluation = report.get('evaluation') or {}
    window = report.get('case_window') or {}
    green, end = evaluation.get('t_green'), window.get('end_sim_s')
    gap = evaluation.get('gap') or {}
    out.update(run_valid=report.get('run_valid'), shutdown_clean=report.get('shutdown_clean'),
               stop_checks=report.get('stop_checks'), wall_elapsed_s=report.get('wall_elapsed_s'),
               gates=evaluation.get('gates'),
               progress_m=(evaluation.get('progress') or {}).get('at_window_end_m'),
               min_gap_m=gap.get('min_sampled_gap_m'), min_swept_gap_m=gap.get('min_swept_lower_bound_m'),
               contacts=gap.get('actor_contact_messages_with_ego'), overlap_samples=gap.get('sampled_overlap_samples'))
    if green is None or end is None:
        return out
    trace = [r for r in json.loads((run/'trace.json').read_bytes()) if green <= r['t'] <= end]
    first = next((r['t'] for r in trace if r['ground_speed_mps'] >= .1), None)
    span = stall = 0.
    for a, b in zip(trace, trace[1:]):
        if first is not None and a['t'] >= first:
            span = span+b['t']-a['t'] if a['ground_speed_mps'] < .05 else 0.
            stall = max(stall, span)
    out.update(peak_speed_mps=max((r['ground_speed_mps'] for r in trace), default=None),
               first_move_after_green_s=None if first is None else first-green,
               longest_stationary_after_move_s=stall,
               safety_reason_s=weighted([r['t'] for r in trace], [r['safety_reason'] for r in trace], green, end))
    out['decisions'] = decisions(run, green, end)
    if (run/'actor_poses.jsonl').exists():
        spec = importlib.util.spec_from_file_location('reapproach_eval', ROOT/'artifacts/validation/2026-11-03/right_reapproach_v3/evaluate.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rows = module.scan_evidence(run, green, end)
        near = [r for r in rows if math.hypot(r['relative_x_m'], r['relative_y_m']) <= 1.0]
        out['actor_scans'] = dict(total=len(rows), observed_3plus=sum(r['hits'] >= 3 for r in rows),
            within_1m=len(near), within_1m_under_3_hits=sum(r['hits'] < 3 for r in near),
            longest_within_1m_unseen_run=max_run([r['hits'] < 3 for r in near]),
            note='hits=기록 스캔(마스크 뒤 NaN 포함)의 상대 외곽 20mm 띠 반환 수. 상대 중심 1m 이내 표본.')
        out['actor_scan_rows'] = [dict(t=round(r['elapsed_s'], 3), hits=r['hits'], x=round(r['relative_x_m'], 3),
                                       y=round(r['relative_y_m'], 3)) for r in rows]
    return out


def max_run(flags):
    best = cur = 0
    for flag in flags:
        cur = cur+1 if flag else 0
        best = max(best, cur)
    return best


if __name__ == '__main__':
    kind, name = sys.argv[1:3]
    result = solo(name) if kind == 'solo' else traffic(name)
    (HERE/f'{name}_eval.json').write_text(json.dumps(result, ensure_ascii=False, indent=1, default=str),
                                          encoding='utf-8', newline='\n')
    print(json.dumps({k: v for k, v in result.items() if k != 'actor_scan_rows'}, ensure_ascii=False, default=str)[:3000])
