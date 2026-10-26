"""작은 중앙 보정 생략(선택) 저장 입력 비교. 센서·속도는 기록 그대로인 개루프이며 폐루프 개선 증거가 아니다.

변형:
- recorded_off/on: 기록된 이전 조향을 그대로 넣는다(OFF는 원래 출력과 전체 동일해야 한다).
- self_off/on: 이전 조향만 이 제어기의 직전 출력으로 바꾼다(노드의 last_steering과 같은 연결). 차량 궤적·속도·
  장면은 원래 주행 그대로라 이후 판단은 실제로 그 명령을 따라간 결과가 아니다.
- self_naive: 반례 대조용. 현재 후륜축 횡오차만 22 mm 이내면 직진을 먼저 두는 단순 deadband(방향·곡률·예측 편차 무시).
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
import arena_autonomy.m3_execution_trial as mx

VARIANTS = (('recorded_off', 'recorded', False, False), ('recorded_on', 'recorded', True, False),
            ('self_off', 'self', False, False), ('self_on', 'self', True, False), ('self_naive', 'self', True, True))


def naive_check(straight, cfg):
    d = abs(float(straight['offset'][0]))
    return {'ok': bool(straight['valid'] and d <= cfg.budget.tracking_straight_m), 'max_abs_offset_m': d,
            'limit_m': cfg.budget.tracking_straight_m, 'curved': bool(straight['curved']),
            'valid': bool(straight['valid']), 'planned_length_m': float(straight['progress'][-1]), 'naive': True}


def run(events, steering_mode, enabled, naive):
    head = events[0]['data']
    motion, cfg = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    execution = replace(mx.ExecutionConfig(**head['execution_config']), small_center_straight=enabled)
    follow = mx.M3ExecutionFollow(cfg, execution, allow_simulation_trial=True)
    original = mx.small_center_check
    if naive:
        mx.small_center_check = naive_check
    rows, previous = [], 0.
    try:
        for event in events[1:-1]:
            if event['kind'] == 'reset':
                follow.reset()
                previous = 0.
                continue
            assert event['kind'] == 'call'
            data = dict(event['data']['input'])
            if steering_mode == 'self':
                data['previous_steering_rad'] = previous
            begin = time.perf_counter_ns()
            out = freeze(replay_call(follow, data, motion))
            ms = (time.perf_counter_ns()-begin)*1e-6
            previous = out[1]
            d = out[2]
            lateral = d.get('m3_lateral') or {}
            state = lateral.get('state') or {}
            small = lateral.get('small_center')
            rows.append({'seq': event['seq'], 't': data['reference_s'], 'output': list(out[:2]), 'reason': d['reason'],
                         'mode': lateral.get('mode'), 'selected': lateral.get('selected'),
                         'all_rejected': bool(d.get('m3_all_commands_rejected')),
                         'released': bool(lateral.get('released_to_baseline')), 'ms': ms,
                         'checks': list((lateral.get('checks') or {}).keys()),
                         'd0': state.get('d0'), 'd_rear0': state.get('d_rear0'), 'psi': state.get('psi'),
                         'small_center': small, 'gate': lateral.get('release_gate'),
                         'threats': lateral.get('threats'),
                         'matches_recorded': bool(equal_output(out, event['data']['output']))})
    finally:
        mx.small_center_check = original
    return rows


def summary(rows):
    active = [r for r in rows if r['mode'] not in (None, 'BASELINE')]
    small = [r for r in rows if r['small_center']]
    chosen = [r for r in small if r['small_center']['ok']]
    stop = [r for r in rows if r['all_rejected']]
    return {'calls': len(rows), 'matches_recorded': sum(r['matches_recorded'] for r in rows),
            'positive': sum(r['output'][0] > 0 for r in rows), 'active_calls': len(active),
            'active_positive': sum(r['output'][0] > 0 for r in active),
            'active_positive_nonzero_steering': sum(r['output'][0] > 0 and r['output'][1] != 0. for r in active),
            'all_commands_rejected_holds': len(stop), 'all_commands_rejected_s': [round(r['t'], 3) for r in stop],
            'modes': dict(Counter(r['mode'] for r in active)), 'reasons': dict(Counter(r['reason'] for r in rows)),
            'release_events_s': [round(r['t'], 3) for r in rows if r['released']],
            'small_center_calls': len(small), 'small_center_ok': len(chosen),
            'small_center_ok_selected_straight': sum(r['selected'] == 'straight' for r in chosen),
            'max_abs_d0_when_ok_m': max((abs(r['d0']) for r in chosen), default=None),
            'max_predicted_offset_when_ok_m': max((r['small_center']['max_abs_offset_m'] for r in chosen), default=None),
            'max_abs_psi_when_ok_rad': max((abs(r['psi']) for r in chosen), default=None),
            'active_p95_ms': float(np.percentile([r['ms'] for r in active], 95)) if active else None,
            'all_p95_ms': float(np.percentile([r['ms'] for r in rows], 95))}


def differences(a, b):
    diff = [(x, y) for x, y in zip(a, b) if not np.allclose(x['output'], y['output'], atol=1e-10, rtol=0)]
    gates = sum(x['gate'] != y['gate'] for x, y in zip(a, b))
    threats = sum(x['threats'] != y['threats'] for x, y in zip(a, b))
    return {'output_differences': len(diff), 'first_difference_s': round(diff[0][0]['t'], 3) if diff else None,
            'release_gate_differences': gates, 'threat_differences': threats}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('inputs', type=Path, nargs='+')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    result = {'scope': 'stored sensor input open-loop counterfactual; not candidate-driven trajectory or closed-loop result',
              'inputs': {}}
    for path in args.inputs:
        raw = path.read_bytes()
        events = [json.loads(line) for line in raw.decode('utf8').splitlines()]
        head, foot = events[0], events[-1]
        assert head['kind'] == 'header' and foot['kind'] == 'footer'
        assert head['data']['controller_mode'] == 'm3_execution_trial'
        assert [e['seq'] for e in events] == list(range(len(events)))
        assert all(foot['data'][k] == 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
        name = path.parent.parent.name
        entry = {'input_sha256': hashlib.sha256(raw).hexdigest(), 'variants': {}}
        rows = {}
        for key, steering, enabled, naive in VARIANTS:
            rows[key] = run(events, steering, enabled, naive)
            entry['variants'][key] = summary(rows[key])
            (args.output/f'{name}_{key}.json').write_text(json.dumps(rows[key], ensure_ascii=False), encoding='utf8')
        entry['recorded_on_vs_off'] = differences(rows['recorded_on'], rows['recorded_off'])
        entry['self_on_vs_off'] = differences(rows['self_on'], rows['self_off'])
        entry['self_naive_vs_on'] = differences(rows['self_naive'], rows['self_on'])
        entry['input_unchanged'] = hashlib.sha256(path.read_bytes()).hexdigest() == entry['input_sha256']
        result['inputs'][name] = entry
        print(json.dumps({name: {k: v for k, v in entry.items() if k != 'variants'}}, ensure_ascii=False), flush=True)
    (args.output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    for name, entry in result['inputs'].items():
        off = entry['variants']['recorded_off']
        assert entry['input_unchanged'] and off['matches_recorded'] == off['calls'], name


if __name__ == '__main__':
    main()
