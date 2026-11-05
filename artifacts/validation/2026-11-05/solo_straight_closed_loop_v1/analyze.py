"""단독 1랩 기준 사후 집계(평가 전용). 정답 자세·신호는 집계에만 쓰며 제어 입력이 아니다.

사용: analyze.py run1 → run1_analysis.json. 같은 함수로 11/4 Ø2 mm JR60P20 기록을 '기존값'으로 따로 계산한다.
원 report.json 판정을 바꾸지 않는다.
"""
from collections import Counter
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PRIOR = ROOT/'artifacts/validation/2026-11-05/solo_jr60p20_baseline_v1'
TARGET = 10/3.6
THRESHOLDS = {'2.0_mps': 2.0, 'target95_2.639_mps': .95*TARGET, 'target_2.778_mps': TARGET}


def jsonl(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def above(t, v, th):
    """임계 이상 총 시간과 최장 연속 시간(선형 보간 교차)."""
    total = best = cur = 0.
    first = None
    for a, b, va, vb in zip(t, t[1:], v, v[1:]):
        dt = b-a
        if min(va, vb) >= th:
            frac, start_in = 1., True
        elif max(va, vb) <= th:
            best, cur = max(best, cur), 0.
            continue
        else:
            frac, start_in = (max(va, vb)-th)/abs(vb-va), va >= th
        seg = dt*frac
        if first is None:
            first = a if start_in else b-seg
        total += seg
        if start_in:
            cur += seg
            if frac < 1.:
                best, cur = max(best, cur), 0.
        else:
            cur = seg
    return dict(total_s=round(total, 3), longest_continuous_s=round(max(best, cur), 3),
                first_reached_sim_s=None if first is None else round(first, 3))


def decisions(run, begin, end, s_of_t, first_move):
    path = run/'follow_decisions.jsonl'
    if not path.exists():
        return None
    total, pre, post, spans, active, calls = Counter(), 0., 0., [], None, 0
    with path.open(encoding='utf-8') as stream:
        next(stream)
        for line in stream:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            t = e['data']['input']['reference_s']
            if not begin <= t <= end:
                continue
            calls += 1
            v, _, d = e['data']['output']
            if v > .001:
                continue
            reason = d.get('reason')
            total[reason] += .02
            if first_move is not None and t >= first_move:
                post += .02
            else:
                pre += .02
            if active and t-active['end'] <= .03:
                active['end'] = t
                active['reasons'][reason] += .02
            else:
                active = dict(start=t, end=t, reasons=Counter({reason: .02}))
                spans.append(active)
    rows = [dict(start_sim_s=round(a['start'], 3), seconds=round(a['end']-a['start']+.02, 2),
                 progress_m=round(float(s_of_t(a['start'])), 2), after_first_move=first_move is not None and a['start'] >= first_move,
                 reasons={k: round(x, 2) for k, x in a['reasons'].items()}) for a in spans]
    return dict(calls=calls, zero_command_s=round(sum(total.values()), 2), before_first_move_s=round(pre, 2),
                after_first_move_s=round(post, 2), by_reason={k: round(x, 2) for k, x in total.most_common()}, spans=rows,
                definition='follow_decisions 출력 속도 ≤0.001 m/s 호출 × 0.02 s(50 Hz). 계획기 명령이며 지면 정지와 다르다.')


def weighted(t, labels, begin, end):
    out = Counter()
    for a, b, la in zip(t, t[1:], labels):
        if begin <= a < end:
            out[str(la)] += min(b, end)-a
    return {k: round(x, 3) for k, x in out.most_common()}


def measure(run, lap_s):
    rows = jsonl(run/'trajectory.jsonl')
    green = [r for r in rows if r['light'] == 'green']
    t = np.array([r['time'] for r in green])
    v = np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'], r['dynamics']['truth_lateral_mps']) for r in green])
    s = np.array([r['progress_m'] for r in green])
    begin = float(t[0])
    end = begin+lap_s if lap_s else float(t[-1])
    keep = np.r_[True, np.diff(s) > 0]

    def s_of_t(x):
        return np.interp(x, t, s)

    def t_of_s(x):
        return float(np.interp(x, s[keep], t[keep]))

    lap = (t >= begin) & (t <= end)
    tl, vl = t[lap], v[lap]
    out = {}
    confirmed = next((r for r in rows if r['autonomy'].get('signal') == 'green_confirmed'), None)
    started = next((r for r in rows if r['autonomy'].get('started')), None)
    move = next((float(a) for a, b in zip(t, v) if b >= .1), None)
    out['start'] = dict(
        first_green_truth_sample_sim_s=round(begin, 3),
        first_autonomy_green_confirmed_sim_s=None if confirmed is None else confirmed['time'],
        first_autonomy_started_sim_s=None if started is None else started['time'],
        light_at_first_started=None if started is None else started['light'],
        first_ground_0p1_mps_sim_s=move, first_move_after_green_s=None if move is None else round(move-begin, 3),
        autonomy_signal_states_before_green=sorted({str(r['autonomy'].get('signal')) for r in rows if r['time'] < begin}),
        note='autonomy.signal은 RGB 처리 결과(센서 입력). light는 평가 정답.')
    out['speed'] = dict(peak_ground_mps=round(float(vl.max()), 4), peak_at_progress_m=round(float(s[lap][np.argmax(vl)]), 2),
                        thresholds={k: above(list(tl), list(vl), th) for k, th in THRESHOLDS.items()},
                        mean_lap_mps=None)
    stops, span = [], None
    for a, b, va in zip(tl, tl[1:], vl):
        if move is not None and a >= move and va < .05:
            span = [a, b] if span is None else [span[0], b]
        elif span is not None:
            stops.append(span)
            span = None
    if span:
        stops.append(span)
    out['ground_stops_after_first_move'] = [dict(start_sim_s=round(a, 3), seconds=round(b-a, 3), progress_m=round(float(s_of_t(a)), 2))
                                            for a, b in stops if b-a >= .1]
    out['zero_command'] = decisions(run, begin, end, s_of_t, move)
    t42, t44 = t_of_s(42.), t_of_s(44.)
    win = (t >= t42) & (t <= t44)
    zero = [z for z in (out['zero_command'] or {}).get('spans', []) if 42. <= z['progress_m'] <= 44.]
    out['loss_42_44_m'] = dict(enter_sim_s=round(t42, 3), exit_sim_s=round(t44, 3), traverse_s=round(t44-t42, 3),
                               mean_mps=round(2./(t44-t42), 3), min_ground_mps=round(float(v[win].min()), 3) if win.any() else None,
                               zero_command_spans=zero, zero_command_s=round(sum(z['seconds'] for z in zero), 2),
                               safety_reason_s=weighted(list(t), [g['safety']['reason'] for g in green], t42, t44))
    edges = np.arange(0., 47., 2.)
    times = [t_of_s(x) for x in edges]
    out['progress_bins_2m_s'] = [dict(s0=float(a), s1=float(b), seconds=round(y-x, 3))
                                 for a, b, x, y in zip(edges, edges[1:], times, times[1:])]
    out['safety'] = dict(reason_s=weighted(list(t), [g['safety']['reason'] for g in green], begin, end),
                         limited_samples=sum(1 for g in green if begin <= g['time'] < end
                                             and g.get('safe_drive_speed_mps', 0) < g['autonomy'].get('speed_command_mps', 0)-.05),
                         definition='/drive/safe가 /drive보다 0.05 m/s 이상 낮은 궤적 표본')
    return out


def budget(run, begin, lap):
    """직선 속도 후보의 판단 사유(follow 판단 기록, 센서 입력만). 첫 초록~결승 안에서 센다."""
    reasons, raised, gain, steps = Counter(), 0, [], []
    rows = jsonl(run/'trajectory.jsonl')
    t = np.array([r['time'] for r in rows]); s = np.array([r['progress_m'] for r in rows])
    with (run/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        next(stream)
        for line in stream:
            e = json.loads(line)
            if e['kind'] != 'call':
                continue
            x = e['data']['input']['reference_s']
            if not begin <= x <= begin+lap:
                continue
            v, _, d = e['data']['output']
            reasons[str(d.get('solo_straight_budget'))] += 1
            original = d.get('original_speed_mps')
            if original is not None and v > original+1e-9:
                raised += 1
                gain.append(v-original)
                steps.append(round(float(np.interp(x, t, s)), 2))
    return dict(reasons=dict(reasons.most_common()), raised_calls=raised,
                max_raise_mps=round(max(gain), 4) if gain else 0., raised_progress_m_min_max=[min(steps), max(steps)] if steps else None,
                raised_in_42_44_m=sum(42. <= x <= 44. for x in steps),
                note='폐루프 판단이므로 OFF 재생과 1:1 대응하지 않는다.')


def report_fields(run):
    r = json.loads((run/'report.json').read_bytes())
    act = r.get('actuation') or {}
    keys = ('passed', 'error', 'route_complete', 'completed_laps', 'progress_m', 'lap_length_m', 'collision_samples',
            'stop_collision_samples', 'false_start', 'official_signal_evidence', 'signal_states', 'marker_ids', 'stop_stable',
            'stop_wait_sim_s', 'stop_displacement_m', 'final_odom_speed_mps', 'dense_pose_stop_evidence', 'actuation_passed',
            'shutdown_clean', 'shutdown_audit', 'remaining_test_pids', 'process_exit_codes', 'process_errors', 'source_unchanged',
            'gazebo_stop_service', 'lidar', 'lidar_rate_passed', 'sensor_delivery', 'drive_delivery', 'sensor_only_inputs',
            'safety_sensor_only_inputs', 'autonomy_subscriptions', 'safety_subscriptions', 'lidar_safety_enabled', 'render_backend',
            'speed_profile', 'speed_bump_enabled', 'sequential_acquisition_verified', 'g1_runtime_verified', 'video')
    out = {k: r.get(k) for k in keys}
    out['final_mcu'] = {k: (act.get('final_mcu') or {}).get(k) for k in ('state', 'reason', 'feedback_mode', 'mean_wheel_mps')}
    out['stop_mode'] = act.get('stop_mode')
    targs = r.get('test_arguments') or {}
    out['test_arguments'] = {k: targs.get(k) for k in ('local_sensor_config', 'pilot_marker_z', 'render_backend', 'speed_profile',
                                                        'lidar_rate_hz', 'control_rate_hz', 'actuation_mode', 'grid_slot', 'laps')}
    sdf = run/'runtime_vehicle.sdf'
    if sdf.exists():
        root = ET.parse(sdf).getroot()
        wires = {}
        for link in root.iter('link'):
            for vis in link.iter('visual'):
                name = vis.get('name', '')
                if name.startswith('c1_mount_') or name == 'rear_marker_id49_board':
                    radius = vis.find('.//cylinder/radius')
                    wires[name] = dict(pose=' '.join((vis.findtext('pose') or '').split()),
                                       radius=None if radius is None else float(radius.text))
        out['runtime_vehicle_structures'] = wires
        plugin = root.find(".//plugin[@name='arena::SingleMotorDrive']")
        out['runtime_drive'] = None if plugin is None else {k: plugin.findtext(k) for k in ('gear_ratio', 'wheel_radius')}
    return out


def main(name):
    run = HERE/name
    result = dict(scope='평가 전용. 평가 정답은 제어 입력이 아니다. prior_reference는 오늘 OFF 기준 run1(같은 장착·월드)이며 1회끼리 비교다.')
    if not (run/'report.json').exists():
        result['error'] = 'report.json missing'
        (HERE/f'{name}_analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
        return 1
    metrics_path = HERE/f'{name}_metrics.json'
    metrics = json.loads(metrics_path.read_bytes()) if metrics_path.exists() and metrics_path.stat().st_size else {}
    result['report'] = report_fields(run)
    result['metrics'] = metrics
    lap = metrics.get('lap_s_from_first_green_sample')
    if (run/'trajectory.jsonl').exists():
        result['now'] = measure(run, lap)
        if lap:
            result['now']['speed']['mean_lap_mps'] = round(result['report']['lap_length_m']/lap, 4)
    if 'now' in result and lap:
        result['straight_budget'] = budget(run, result['metrics']['first_green_sample_s'], lap)
    for extra in ('surface', 'motor_audit'):
        p = HERE/f'{name}_{extra}.json'
        result[extra] = json.loads(p.read_bytes()) if p.exists() and p.stat().st_size else None
    video = result['report'].get('video') or {}
    result['video_presence'] = dict(file_exists=(ROOT/video['path']).exists() if video.get('path') else False,
                                    **{k: video.get(k) for k in ('frames_written', 'decoded_frames', 'decode_passed',
                                                                 'first_frame_sim_time_s', 'last_frame_sim_time_s')})
    prior_metrics = json.loads((PRIOR/'run1_metrics.json').read_bytes())
    result['prior_reference'] = dict(run='artifacts/validation/2026-11-05/solo_jr60p20_baseline_v1/run1',
                                     note='오늘 OFF 기준 run1(같은 장착·월드). 같은 함수로 계산. 1회끼리 비교.',
                                     lap_s=prior_metrics['lap_s_from_first_green_sample'],
                                     **measure(PRIOR/'run1', prior_metrics['lap_s_from_first_green_sample']))
    if 'now' in result:
        result['progress_bins_now_minus_prior_s'] = [
            dict(s0=a['s0'], s1=a['s1'], delta_s=round(a['seconds']-b['seconds'], 3))
            for a, b in zip(result['now']['progress_bins_2m_s'], result['prior_reference']['progress_bins_2m_s'])]
    (HERE/f'{name}_analysis.json').write_text(json.dumps(result, ensure_ascii=False, indent=1, default=str),
                                              encoding='utf-8', newline='\n')
    now = result.get('now', {})
    print(json.dumps(dict(lap=lap, passed=result['report'].get('passed'), peak=now.get('speed', {}).get('peak_ground_mps'),
                          zero=(now.get('zero_command') or {}).get('zero_command_s'),
                          loss_42_44=now.get('loss_42_44_m', {}).get('traverse_s')), ensure_ascii=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1] if len(sys.argv) > 1 else 'run1'))
