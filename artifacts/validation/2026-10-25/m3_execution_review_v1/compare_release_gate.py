"""관측 복귀 관문의 OFF 동일성 및 이전 ON/현재 ON 비교. 저장 센서 입력의 개루프다."""
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


def evaluate(events, enabled):
    head = events[0]['data']
    motion = MotionConfig(**head['motion_config'])
    cfg = FollowConfig(**head['follow_config'])
    execution = replace(ExecutionConfig(**head['execution_config']), release_to_baseline=enabled)
    controller = M3ExecutionFollow(cfg, execution, allow_simulation_trial=True)
    rows, matching = [], 0
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            controller.reset()
            continue
        data = event['data']['input']
        start = time.perf_counter_ns()
        out = freeze(replay_call(controller, data, motion))
        elapsed = (time.perf_counter_ns()-start)*1e-6
        matching += equal_output(out, event['data']['output'])
        d = out[2]
        lateral = d.get('m3_lateral', {})
        gate = lateral.get('release_gate')
        rows.append({'seq': event['seq'], 't': data['reference_s'], 'output': out[:2],
                     'reason': d['reason'], 'mode': lateral.get('mode'), 'ms': elapsed,
                     'released': bool(lateral.get('released_to_baseline')), 'gate': gate})
    gate_rows = [r for r in rows if r['gate'] and r['gate'].get('sides')]
    return rows, {'calls': len(rows), 'exact_output_matches': matching,
                 'positive': sum(r['output'][0] > 0 for r in rows),
                 'release_events_s': [r['t'] for r in rows if r['released']],
                 'reasons': dict(Counter(r['reason'] for r in rows)),
                 'gate_reasons': dict(Counter(r['gate'].get('reason') for r in rows if r['gate'])),
                 'p95_ms': float(np.percentile([r['ms'] for r in rows], 95)),
                 'gate_calls': len(gate_rows),
                 'gate_calls_p95_ms': float(np.percentile([r['ms'] for r in gate_rows], 95)) if gate_rows else None,
                 'maximum_ms': max(r['ms'] for r in rows)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('inputs', type=Path, nargs='+')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--old-summary', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(exist_ok=False)
    old = json.loads(args.old_summary.read_text(encoding='utf8'))
    summary = {'scope': 'fixed-input counterfactual; not closed-loop driving', 'inputs': {}}
    for path in args.inputs:
        raw = path.read_bytes()
        events = [json.loads(line) for line in raw.decode('utf8').splitlines()]
        assert events[0]['kind'] == 'header' and events[-1]['kind'] == 'footer'
        assert events[0]['data']['controller_mode'] == 'm3_execution_trial'
        assert [e['seq'] for e in events] == list(range(len(events)))
        name = path.parent.parent.name
        entry = {'input_sha256': hashlib.sha256(raw).hexdigest()}
        assert old['inputs'][name]['input_sha256'] == entry['input_sha256']
        entry['previous_on'] = old['inputs'][name]['release_on']
        for enabled in (False, True):
            rows, stats = evaluate(events, enabled)
            key = 'on' if enabled else 'off'
            entry[key] = stats
            (args.output/f'{name}_{key}.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf8')
        entry['input_unchanged'] = hashlib.sha256(path.read_bytes()).hexdigest() == entry['input_sha256']
        summary['inputs'][name] = entry
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    assert all(e['input_unchanged'] and e['off']['exact_output_matches'] == e['off']['calls']
               for e in summary['inputs'].values())


if __name__ == '__main__':
    main()
