"""M3 제한 폐루프 결과의 읽기 전용 분해. 제어 재실행/원본 판정 변경 없음.

정답 자세(자차·옆차·앞차)는 사후 평가 축으로만 쓴다. 제어 입력이 아니다.
진행성·측방 간격·관측 거절·MCU 상태를 서로 다른 표로 분리한다.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

EGO_L, EGO_W = .20, .15


def load_jsonl(path):
    with open(path, encoding='utf-8') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def frame(h):
    c, s = math.cos(h), math.sin(h)
    return lambda x, y, x0, y0: ((x-x0)*c+(y-y0)*s, -(x-x0)*s+(y-y0)*c)


def latest(rows, t, start=0):
    i = start
    while i+1 < len(rows) and rows[i+1][0] <= t:
        i += 1
    return i


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--side-actor', default='eval_e3_side')
    args = p.parse_args()
    if args.output.exists() or args.output.resolve().is_relative_to(args.run.resolve()):
        raise ValueError('new output outside raw run required')
    report = json.loads((args.run/'report.json').read_text(encoding='utf-8'))
    raw_decisions = (args.run/'follow_decisions.jsonl').read_bytes()
    events = [json.loads(line) for line in raw_decisions.decode().splitlines()]
    ego = load_jsonl(args.run/'ego_poses.jsonl')
    lead = load_jsonl(args.run/'actor_poses.jsonl')
    extra = json.loads((args.run/'additional_actors.json').read_text(encoding='utf-8'))
    side = extra['poses'][args.side_actor]
    mcu = json.loads((args.run/'actuation_trace.json').read_text(encoding='utf-8'))
    ev = report['evaluation']
    t_green = ev['t_green']
    # 등록 직선 구간의 시작 자차 방향을 사후 좌표축으로 둔다(평가 전용).
    _, x0, y0, h0 = ego[0]
    to_local = frame(h0)

    # 1) 결정 단계: 이유·선택·실제 명령 거절
    calls = [e for e in events if e['kind'] == 'call']
    reasons, modes, selected, rejected, positive_by_candidate = Counter(), Counter(), Counter(), Counter(), Counter()
    fallback_positive, spans = 0, []
    for e in calls:
        speed, steering, d = e['data']['output']
        lat = d.get('m3_lateral', {})
        reasons[d['reason']] += 1
        modes[str(lat.get('mode'))] += 1
        selected[str(lat.get('selected'))] += 1
        rej = {k: v['command']['reason'] for k, v in lat.get('checks', {}).items()
               if 'command' in v and not v['command']['accepted']}
        for k, v in rej.items():
            rejected[k+':'+str(v)] += 1
        if speed > 0:
            positive_by_candidate[str(d.get('m3_candidate'))] += 1
            fallback_positive += bool(rej)
        t = e['data']['input']['reference_s']
        present = ','.join(k for k, v in lat.get('threats', {}).items() if v.get('present'))
        key = [d['reason'], lat.get('mode'), present]
        if spans and spans[-1]['key'] == key:
            spans[-1].update(last_s=t, calls=spans[-1]['calls']+1, positive=spans[-1]['positive']+int(speed > 0))
        else:
            spans.append({'key': key, 'first_s': t, 'last_s': t, 'calls': 1, 'positive': int(speed > 0)})

    # 2) 정답 기반 사후 기하: 자차 횡위치·진행·옆차 상대 위치(평가 전용)
    timeline, j_side, j_lead = [], 0, 0
    first_side_present = next((e['data']['input']['reference_s'] for e in calls
                               if any(v.get('present') for v in e['data']['output'][2].get('m3_lateral', {}).get('threats', {}).values())), None)
    for t, x, y, h in ego[::10]:
        if t < t_green-.2:
            continue
        j_side = latest(side, t, j_side)
        j_lead = latest(lead, t, j_lead)
        ex, ey = to_local(x, y, x0, y0)
        sx, sy = to_local(side[j_side][1], side[j_side][2], x0, y0)
        lx, ly = to_local(lead[j_lead][1], lead[j_lead][2], x0, y0)
        dyaw = math.remainder(h-h0, math.tau)
        half_span = (EGO_L*abs(math.sin(dyaw))+EGO_W*abs(math.cos(dyaw)))/2
        timeline.append({'t': round(t, 3), 'ego_long_m': round(ex, 4), 'ego_lat_m': round(ey, 4),
                         'ego_yaw_deg': round(math.degrees(dyaw), 2),
                         'side_rel_long_m': round(sx-ex, 4), 'side_lat_m': round(sy, 4),
                         'side_lateral_projection_gap_m': round(sy-ey-half_span-EGO_W/2, 4),
                         'lead_rel_long_m': round(lx-ex, 4)})
    overlap = [r for r in timeline if abs(r['side_rel_long_m']) < EGO_L]
    def phase(r):
        return 'behind' if r['side_rel_long_m'] <= -EGO_L else ('ahead' if r['side_rel_long_m'] >= EGO_L else 'alongside')
    phases = []
    for r in timeline:
        if not phases or phases[-1]['phase'] != phase(r):
            phases.append({'phase': phase(r), 'first_s': r['t'], 'last_s': r['t']})
        else:
            phases[-1]['last_s'] = r['t']

    # 3) MCU 상태 전이
    transitions, last = [], None
    for m in mcu:
        key = (m.get('state'), m.get('reason'))
        if key != last:
            transitions.append({'now_us': m.get('now_us'), 'state': key[0], 'reason': key[1]})
            last = key

    side_report = report['additional_actors']['actors'][args.side_actor]
    result = {
        'scope': 'read-only decomposition; truth poses used only for evaluation axes; no control replay or verdict change',
        'decision_sha256': hashlib.sha256(raw_decisions).hexdigest(),
        'verdict_original': {k: report.get(k) for k in ('passed', 'run_valid', 'behavior_gates_passed', 'harness_valid')},
        'gates': ev['gates'], 'integrity': report.get('integrity'), 'stop_checks': report.get('stop_checks'),
        'controller_mode': ev.get('controller_mode'),
        'progress': ev['progress'], 'case_metrics': ev['case_metrics'],
        'side_actor': {k: side_report.get(k) for k in ('gates', 'min_sampled_gap_m', 'min_swept_lower_bound_m')},
        'side_encounter': {k: v for k, v in side_report.get('encounter', {}).items() if k != 'coverage'},
        'decisions': {'calls': len(calls), 'positive': sum(e['data']['output'][0] > 0 for e in calls),
                      'positive_after_rejected_command': fallback_positive,
                      'positive_by_candidate': dict(positive_by_candidate), 'reasons': dict(reasons),
                      'modes': dict(modes), 'selected': dict(selected), 'rejected_commands': dict(rejected),
                      'first_side_threat_present_s': first_side_present},
        'phases_truth': phases,
        'alongside_truth_window': {'first_s': overlap[0]['t'], 'last_s': overlap[-1]['t'],
                                   'ego_lat_range_m': [min(r['ego_lat_m'] for r in overlap), max(r['ego_lat_m'] for r in overlap)],
                                   'min_projection_gap_m': min(r['side_lateral_projection_gap_m'] for r in overlap)} if overlap else None,
        'ego_lat_initial_m': timeline[0]['ego_lat_m'] if timeline else None,
        'ego_lat_min_max_m': [min(r['ego_lat_m'] for r in timeline), max(r['ego_lat_m'] for r in timeline)] if timeline else None,
        'mcu_transitions': transitions,
        'actuation_diagnostics': report.get('actuation_diagnostics'),
        'wall_elapsed_s': report.get('wall_elapsed_s'),
        'spans': spans,
        'timeline_every_10_samples': timeline,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: result[k] for k in ('verdict_original', 'progress', 'case_metrics', 'side_actor', 'decisions',
                                             'phases_truth', 'alongside_truth_window', 'ego_lat_initial_m',
                                             'ego_lat_min_max_m', 'mcu_transitions')}, ensure_ascii=False, indent=1))


if __name__ == '__main__':
    main()
