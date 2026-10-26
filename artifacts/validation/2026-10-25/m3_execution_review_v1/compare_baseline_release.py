"""저장 M3 판단 기록의 기본 G1 복귀 옵션 OFF/ON 개루프 비교. 새 주행/안전 판정 아님.

OFF는 기록 헤더의 실행 설정 그대로이며 원래 출력과 전부 같아야 한다. ON은 같은 센서·자세·측정 속도에
release_to_baseline만 켠 반사실이다. 후보가 낸 속도로 차량이 움직인 결과가 아니다.
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
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig


def run(events, head, enabled):
    motion, config = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    execution = replace(ExecutionConfig(**head['execution_config']), release_to_baseline=enabled)
    follow = M3ExecutionFollow(config, execution, allow_simulation_trial=True)
    rows, exact, counter, modes = [], 0, Counter(), Counter()
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            follow.reset()
            continue
        data = event['data']['input']
        begin = time.perf_counter_ns()
        out = freeze(replay_call(follow, data, motion))
        ms = (time.perf_counter_ns()-begin)*1e-6
        d = out[2]
        lateral = d.get('m3_lateral', {})
        rows.append({'seq': event['seq'], 't': data['reference_s'], 'output': out[:2], 'reason': d['reason'],
                     'mode': lateral.get('mode'), 'released': bool(lateral.get('released_to_baseline')),
                     'baseline_output': d.get('baseline_output'), 'ms': ms})
        exact += equal_output(out, event['data']['output'])
        counter[d['reason']] += 1
        modes[str(lateral.get('mode'))] += 1
    releases = [r['t'] for r in rows if r['released']]
    after = [r for r in rows if releases and r['t'] >= releases[0]]
    return rows, {'calls': len(rows), 'exact_output_matches': exact,
                  'positive': sum(r['output'][0] > 0 for r in rows),
                  'max_speed_mps': max((r['output'][0] for r in rows), default=None),
                  'release_events_s': releases,
                  'calls_after_first_release': len(after),
                  'baseline_mode_after_first_release': sum(r['mode'] == 'BASELINE' for r in after),
                  'positive_after_first_release': sum(r['output'][0] > 0 for r in after),
                  'output_equals_baseline_after_release': sum(r['mode'] == 'BASELINE' and r['baseline_output'] is not None
                                                              and r['output'] == r['baseline_output'] for r in after),
                  'reasons': dict(counter), 'modes': dict(modes),
                  'decision_ms_p95': float(np.percentile([r['ms'] for r in rows], 95)) if rows else None}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('inputs', type=Path, nargs='+')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise ValueError('new output folder required')
    args.output.mkdir(parents=True)
    summary = {'scope': 'open-loop counterfactual on stored sensor input; not a candidate-driven trajectory', 'inputs': {}}
    for path in args.inputs:
        raw = path.read_bytes()
        events = [json.loads(line) for line in raw.decode().splitlines()]
        head = events[0]['data']
        assert events[0]['kind'] == 'header' and head['controller_mode'] == 'm3_execution_trial'
        assert [e['seq'] for e in events] == list(range(len(events)))
        name = path.parent.parent.name
        entry = {'input_sha256': hashlib.sha256(raw).hexdigest(), 'header_execution_config': head['execution_config']}
        for enabled in (False, True):
            rows, stats = run(events, head, enabled)
            key = 'release_on' if enabled else 'release_off'
            (args.output/f'{name}_{key}.json').write_text(json.dumps(rows, ensure_ascii=False, indent=0),
                                                         encoding='utf-8', newline='\n')
            entry[key] = stats
        entry['input_unchanged'] = hashlib.sha256(path.read_bytes()).hexdigest() == entry['input_sha256']
        summary['inputs'][name] = entry
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8',
                                           newline='\n')
    print(json.dumps(summary, ensure_ascii=False, indent=1))
    for entry in summary['inputs'].values():
        assert entry['input_unchanged']
        off = entry['release_off']
        assert off['exact_output_matches'] == off['calls'] and not off['release_events_s']


if __name__ == '__main__':
    main()
