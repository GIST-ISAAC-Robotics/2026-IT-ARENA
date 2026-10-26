"""M3 시험이 해제되지 않은 채(복귀 기록 만료) 첫 코너에 들어가면 어떻게 되는가 — run24 저장 입력 개루프 주입.

run24는 측방 미촉발 기본 G1 코너 추종 기록이다. 지정 시각부터 측방 시험을 활성·위협 소멸 뒤 중앙 단계로 두고
만료된 위협 기록을 넣어 run19/run27 끝과 같은 '복귀 불가' 상태를 만든다. 이전 조향은 이 제어기의 직전 출력이다.
차량은 원래 기본 G1 궤적으로 움직였으므로, 각 자세에서 M3가 움직일 수 있었는지를 보는 진단이지 폐루프 결과가 아니다.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np

from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
import arena_autonomy.m3_execution_trial as mx


def stale_record():
    scene = SimpleNamespace(t_ref_s=0., source_epoch=-1, source_first_s=0.)   # 다른 세대: 항상 만료/불일치
    return mx.ThreatRecord('left', scene, np.zeros((1, 2)), np.zeros(1), None, 0.)


def run(events, small, inject_s):
    head = events[0]['data']
    motion, cfg = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    execution = replace(mx.ExecutionConfig(**head['execution_config']), small_center_straight=small)
    follow = mx.M3ExecutionFollow(cfg, execution, allow_simulation_trial=True)
    rows, previous, injected, matched_before = [], 0., False, 0
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            follow.reset()
            previous = 0.
            continue
        data = dict(event['data']['input'])
        if inject_s is not None and not injected and data['reference_s'] >= inject_s:
            follow.lateral.active = True
            follow.lateral.clear_fresh = follow.lateral_config.release_fresh
            follow.lateral.records = {'left': stale_record()}
            injected = True
        if injected:
            data['previous_steering_rad'] = previous
        begin = time.perf_counter_ns()
        out = freeze(replay_call(follow, data, motion))
        ms = (time.perf_counter_ns()-begin)*1e-6
        previous = out[1]
        if not injected:
            matched_before += equal_output(out, event['data']['output'])
            continue
        d = out[2]
        lateral = d.get('m3_lateral') or {}
        checks = lateral.get('checks') or {}
        envelope = d.get('m3_envelope') or {}
        obs = envelope.get('observation') or {}
        rows.append({'t': data['reference_s'], 'output': list(out[:2]), 'recorded_output': event['data']['output'][:2],
                     'reason': d['reason'], 'mode': lateral.get('mode'), 'selected': lateral.get('selected'),
                     'all_rejected': bool(d.get('m3_all_commands_rejected')), 'ms': ms,
                     'gate': (lateral.get('release_gate') or {}).get('reason'), 'active': follow.lateral.active,
                     'state': lateral.get('state'), 'small_center': lateral.get('small_center'),
                     'plan': {k: v.get('reasons') for k, v in checks.items()},
                     'command': {k: (v.get('command') or {}).get('reason') for k, v in checks.items() if v.get('command')},
                     'envelope_steering': [g['steering_rad'] for g in envelope.get('geometry', [])],
                     'envelope_unsupported': obs.get('unsupported'), 'envelope_masked': obs.get('masked_candidates'),
                     'recorded_steering': event['data']['output'][1]})
    return rows, matched_before


def sections(rows, curve_s):
    out = {}
    for name, lo, hi in (('before_curve', -np.inf, curve_s[0]), ('in_curve', curve_s[0], curve_s[1]),
                         ('after_curve', curve_s[1], np.inf)):
        part = [r for r in rows if lo <= r['t'] < hi]
        if not part:
            continue
        positive = [r for r in part if r['output'][0] > 0]
        out[name] = {'calls': len(part), 'positive': len(positive),
                     'positive_nonzero_steering': sum(r['output'][1] != 0 for r in positive),
                     'max_abs_positive_steering_rad': max((abs(r['output'][1]) for r in positive), default=None),
                     'recorded_max_abs_steering_rad': max(abs(r['recorded_steering']) for r in part),
                     'all_rejected': sum(r['all_rejected'] for r in part),
                     'reasons': dict(Counter(r['reason'] for r in part)), 'modes': dict(Counter(r['mode'] for r in part)),
                     'plan_rejections': dict(Counter(f'{k}:{x}' for r in part for k, v in r['plan'].items()
                                                     for x in (v or []))),
                     'command_rejections': dict(Counter(f'{k}:{v}' for r in part for k, v in r['command'].items() if v)),
                     'gate': dict(Counter(r['gate'] for r in part)), 'still_active': all(r['active'] for r in part),
                     'p95_ms': float(np.percentile([r['ms'] for r in part], 95))}
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument('input', type=Path)
    p.add_argument('--inject-s', type=float, required=True)
    p.add_argument('--curve-s', type=float, nargs=2, required=True, help='평가 전용 코너 구간(기록 시각)')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    raw = args.input.read_bytes()
    events = [json.loads(line) for line in raw.decode('utf8').splitlines()]
    assert events[0]['kind'] == 'header' and events[-1]['kind'] == 'footer'
    result = {'scope': 'stored run24 corner input with injected unreleased M3 state; open-loop, not closed-loop driving',
              'input_sha256': hashlib.sha256(raw).hexdigest(), 'inject_s': args.inject_s, 'curve_s': args.curve_s,
              'variants': {}}
    for small in (False, True):
        rows, matched = run(events, small, args.inject_s)
        key = 'small_center_on' if small else 'small_center_off'
        first_stuck = next((r['t'] for k, r in enumerate(rows) if all(x['output'][0] == 0 for x in rows[k:])), None)
        result['variants'][key] = {'matched_before_injection': matched, 'calls_after_injection': len(rows),
                                   'first_time_never_positive_again_s': first_stuck, 'sections': sections(rows, args.curve_s)}
        (args.output/f'{key}.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf8')
    result['input_unchanged'] = hashlib.sha256(args.input.read_bytes()).hexdigest() == result['input_sha256']
    (args.output/'summary.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result, ensure_ascii=False, indent=2))
    assert result['input_unchanged']


if __name__ == '__main__':
    main()
