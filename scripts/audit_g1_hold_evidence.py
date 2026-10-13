#!/usr/bin/env python3
"""평가 전용: HOLD 요청, 0목표 제동, 밀집 자세의 안정 정지를 구분한다.

기존 report/감사 결과는 수정하지 않는다. 원래 사례의 정지 시간·속도 기준을
밀집 자세에 적용하는 추가 증거이며 제어기·MCU·실물 구동 허가가 아니다.
"""
import argparse
from bisect import bisect_right
import hashlib
import json
import math
from pathlib import Path

from g1_traffic_eval import POSE_MAX_GAP_S, TRACE_MAX_GAP_S, kinematic_speed_segments


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def merge(spans):
    result = []
    for a, b in spans:
        if result and abs(a-result[-1][1]) < 1e-9:
            result[-1][1] = b
        else:
            result.append([a, b])
    return result


def covered(spans, starts, a, b):
    i = bisect_right(starts, a+1e-9)-1
    return i >= 0 and spans[i][1] >= b-1e-9


def decision_holds(events):
    """상태 5 Hz 대신 전체 판단 기록을 사용. reset/누락/양수 요청은 구간을 끊는다."""
    if not events or events[0].get('kind') != 'header' or events[-1].get('kind') != 'footer':
        raise ValueError('complete decision capture required')
    if events[0].get('data', {}).get('schema') != 1:
        raise ValueError('unsupported decision capture')
    footer = events[-1]['data']
    if (any(footer.get(k) != 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
            or footer.get('worker_alive') is not False
            or any(footer.get(k) != len(events)-1 for k in ('accepted', 'written'))):
        raise ValueError('incomplete decision capture')
    spans, prior, last_time = [], None, -math.inf
    for index, event in enumerate(events):
        if event.get('seq') != index:
            raise ValueError('decision sequence gap')
        kind = event.get('kind')
        if kind != 'call':
            if kind not in ('header', 'reset', 'footer') or (kind == 'header' and index != 0) or (
                    kind == 'footer' and index != len(events)-1):
                raise ValueError('unexpected decision event')
            prior = None
            continue
        data = event['data']
        t, output = data['input']['reference_s'], data['output']
        if (not finite(t) or t <= last_time or not isinstance(output, list) or len(output) != 3
                or not finite(output[0]) or not isinstance(output[2], dict)):
            raise ValueError('invalid decision call')
        held = output[0] == 0 and output[2].get('behavior') == 'HOLD'
        if prior and held and prior[1] and t-prior[0] <= TRACE_MAX_GAP_S+1e-9:
            spans.append([prior[0], t])
        prior, last_time = (t, held), t
    return merge(spans)


def zero_target_spans(dynamics):
    times = [r.get('sim_time_s') for r in dynamics]
    if (len(times) < 2 or not all(finite(t) for t in times)
            or any(b <= a for a, b in zip(times, times[1:]))
            or any(not finite(r.get('applied_target_mps')) for r in dynamics)):
        raise ValueError('ordered finite applied-target samples required')
    return merge([[a['sim_time_s'], b['sim_time_s']] for a, b in zip(dynamics, dynamics[1:])
                  if a['applied_target_mps'] == b['applied_target_mps'] == 0
                  and b['sim_time_s']-a['sim_time_s'] <= POSE_MAX_GAP_S+1e-9])


def requirement(config, report):
    cases = [c for c in config['cases'] if c['name'] == report['case']]
    if len(cases) != 1 or report.get('mode') != 'g1':
        raise ValueError('one registered G1 case required')
    case, rules = cases[0], config['acceptance'][report['case']]
    first = report['green_observed_sim_s']
    if not finite(first) or not finite(case['duration_s']) or case['duration_s'] <= 0:
        raise ValueError('invalid case window')
    kind = case.get('behavior_kind', case['name'])
    end = first+case['duration_s']
    if kind == 'stopped_center':
        required = rules['minimum_continuous_hold_s']
    elif kind == 'stop_restart':
        lo, hi = rules['hold_window_s']
        first, end = first+lo, first+hi
        required = rules['minimum_hold_s']
    elif kind in ('side_occlusion', 'slow_lead'):
        required = None  # 0요청이 있다고 안정 정지 완료를 요구하는 사례로 바꾸지 않는다.
    else:
        raise ValueError('unsupported case kind')
    if end <= first or (required is not None and (not finite(required) or required <= 0)):
        raise ValueError('invalid stationary requirement')
    return dict(first_s=first, last_s=end, required_hold_s=required,
                kind=kind, speed_below_mps=.01)


def safe_zero_spans(trace):
    rows = []
    for row in trace:
        t = (row.get('braking_diagnostics') or {}).get('safe_command_stamp_s')
        speed = row.get('safe_speed_mps')
        if not finite(t) or not finite(speed):
            raise ValueError('safe command source time required; pose time is not a substitute')
        if rows and t == rows[-1][0]:
            if speed != rows[-1][1]:
                raise ValueError('same safe command timestamp has different values')
            continue
        if rows and t < rows[-1][0]:
            raise ValueError('safe command time reversed')
        rows.append((t, speed))
    return merge([[a[0], b[0]] for a, b in zip(rows, rows[1:])
                  if a[1] == b[1] == 0 and b[0]-a[0] <= TRACE_MAX_GAP_S+1e-9])


def audit(poses, dynamics, events, trace, config, report):
    req = requirement(config, report)
    segments = kinematic_speed_segments(poses)
    if not segments:
        raise ValueError('dense poses required')
    holds, zeros, safe = decision_holds(events), zero_target_spans(dynamics), safe_zero_spans(trace)
    hs, zs, ss = [a for a, _ in holds], [a for a, _ in zeros], [a for a, _ in safe]
    # 표본 구간 전체가 원래 시간창과 세 영목표 기록 범위 안에 있을 때만 사용한다.
    selected = [s for s in segments if s['t0'] >= req['first_s']-1e-9
                and s['t1'] <= req['last_s']+1e-9
                and s['t1']-s['t0'] <= POSE_MAX_GAP_S+1e-9
                and covered(holds, hs, s['t0'], s['t1'])
                and covered(zeros, zs, s['t0'], s['t1'])
                and covered(safe, ss, s['t0'], s['t1'])]
    stationary = merge([[s['t0'], s['t1']] for s in selected if s['average_speed_mps'] < .01])
    longest = max((b-a for a, b in stationary), default=0.)
    required = req['required_hold_s']
    # 안정 완료 뒤 0목표 유지 중 움직임 재발도 별도 기록. 짧은 제동 과도와 다르다.
    episodes = []
    stable_hold = config['acceptance']['stop']['stable_hold_sim_s']
    if not finite(stable_hold) or stable_hold <= 0:
        raise ValueError('invalid registered stable hold')
    for a, b in zeros:
        sub = [s for s in segments if a <= s['t0'] and s['t1'] <= b
               and s['t1']-s['t0'] <= POSE_MAX_GAP_S+1e-9]
        low = merge([[s['t0'], s['t1']] for s in sub if s['average_speed_mps'] < .01])
        settled = next((x+stable_hold for x, y in low if y-x >= stable_hold-1e-9), None)
        lost = [s for s in sub if settled is not None and s['t0'] >= settled
                and s['average_speed_mps'] >= .01]
        episodes.append(dict(first_s=a, last_s=b, settled_after_s=settled,
                             longest_low_speed_s=max((y-x for x, y in low), default=0.),
                             maximum_speed_mps=max((s['average_speed_mps'] for s in sub), default=None),
                             movement_after_settled=bool(lost), first_movement_after_settled=lost[:1]))
    return dict(schema='g1_hold_evidence_v1', requirement=req,
                hold_requirement_applicable=required is not None,
                dense_hold_requirement_met=longest >= required-1e-9 if required is not None else None,
                longest_dense_hold_s=longest, dense_hold_intervals=stationary,
                commanded_hold_intervals=holds, sampled_zero_target_episodes=episodes,
                original_verdict_changed=False, driving_authorized=False,
                limits=['HOLD is a zero-speed request, not an established stationary flag',
                        'only complete pose segments count; no gap fill or speed interpolation',
                        'sampled targets bracket intervals; unrecorded faster target toggles are not excluded',
                        'pose secants are sampled averages, not instantaneous or continuous velocity',
                        'no physical cause attribution or real vehicle braking claim'])


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists(): parser.error('output must be new')
    names = ('report.json', 'trace.json', 'ego_poses.jsonl', 'braking_diagnostics.jsonl', 'follow_decisions.jsonl')
    paths = {n: args.run/n for n in names}
    paths['config'] = args.config
    hashes = {n: digest(p) for n, p in paths.items()}
    report = json.loads(paths['report.json'].read_text())
    if report['config_sha256'] != hashes['config']:
        raise ValueError('config differs from original run')
    def rows(name):
        with paths[name].open(encoding='utf-8') as stream:
            return [json.loads(line) for line in stream]
    result = audit(rows('ego_poses.jsonl'), rows('braking_diagnostics.jsonl'),
                   rows('follow_decisions.jsonl'), json.loads(paths['trace.json'].read_text()),
                   json.loads(args.config.read_text()), report)
    result.update(input_sha256=hashes, source_sha256=digest(__file__),
                  inputs_unchanged=all(digest(paths[n]) == h for n, h in hashes.items()),
                  original_run_passed=report.get('passed'))
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps({k: result[k] for k in ('schema', 'hold_requirement_applicable',
                     'dense_hold_requirement_met', 'longest_dense_hold_s', 'inputs_unchanged')}))
    return 0 if result['inputs_unchanged'] and result['dense_hold_requirement_met'] is not False else 2


if __name__ == '__main__':
    raise SystemExit(main())
