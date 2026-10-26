"""run9 센서 기록의 후보 선택 순서 OFF/ON 개루프 비교. 새 주행/안전 판정 아님."""
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


def main():
    p = argparse.ArgumentParser()
    p.add_argument('input', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise ValueError('new output folder required')
    raw = args.input.read_bytes()
    events = [json.loads(line) for line in raw.decode().splitlines()]
    head, foot = events[0]['data'], events[-1]['data']
    assert events[0]['kind'] == 'header' and events[-1]['kind'] == 'footer'
    assert [e['seq'] for e in events] == list(range(len(events)))
    assert all(foot[k] == 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
    assert foot['worker_alive'] is False and foot['written'] == foot['accepted'] == len(events)-1
    motion, config = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    args.output.mkdir(parents=True)
    summary = {'scope': 'counterfactual stored sensor input, not candidate-driven trajectory',
        'input_sha256': hashlib.sha256(raw).hexdigest(), 'variants': {}}
    for mode in ('recorded', 'self_steering'):
        for enabled in (False, True):
            follow = M3ExecutionFollow(config, replace(ExecutionConfig(**head['execution_config']),
                command_fallback=enabled), allow_simulation_trial=True)
            previous, rows, exact, counter = 0., [], 0, Counter()
            for event in events[1:-1]:
                if event['kind'] == 'reset':
                    follow.reset()
                    previous = 0.
                    continue
                assert event['kind'] == 'call'
                data = dict(event['data']['input'])
                if mode == 'self_steering':
                    data['previous_steering_rad'] = previous
                begin = time.perf_counter_ns()
                out = freeze(replay_call(follow, data, motion))
                ms = (time.perf_counter_ns()-begin)*1e-6
                previous = out[1]
                d = out[2]
                lateral = d.get('m3_lateral', {})
                checks = lateral.get('checks', {})
                row = {'seq': event['seq'], 't': data['reference_s'], 'output': out[:2],
                    'reason': d['reason'], 'candidate': d.get('m3_candidate'),
                    'mode': lateral.get('mode'), 'ms_restore_scene_command': ms,
                    'rejected_commands': {k:v['command']['reason'] for k,v in checks.items()
                                         if 'command' in v and not v['command']['accepted']},
                    'selected': d.get('m3_selected', False),
                    'observation': (d.get('m3_envelope') or {}).get('observation_ok'),
                    'geometry': (d.get('m3_envelope') or {}).get('geometry_ok')}
                rows.append(row)
                exact += equal_output(out, event['data']['output'])
                counter[d['reason']] += 1
            key = mode+('_on' if enabled else '_off')
            (args.output/(key+'.json')).write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding='utf-8')
            summary['variants'][key] = {'calls': len(rows), 'exact_output_matches': exact,
                'positive': sum(r['output'][0] > 0 for r in rows), 'reasons': dict(counter),
                'positive_after_rejected_candidate': sum(r['output'][0] > 0 and bool(r['rejected_commands']) for r in rows),
                'positive_without_supported_envelope': sum(r['output'][0] > 0 and r['selected']
                    and not (r['observation'] and r['geometry']) for r in rows),
                'restore_scene_command_ms_p95': float(np.percentile([r['ms_restore_scene_command'] for r in rows], 95))}
    summary['input_unchanged'] = hashlib.sha256(args.input.read_bytes()).hexdigest() == summary['input_sha256']
    (args.output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False))
    assert summary['input_unchanged']
    assert summary['variants']['recorded_off']['exact_output_matches'] == summary['variants']['recorded_off']['calls']
    assert all(v['positive_without_supported_envelope'] == 0 for v in summary['variants'].values())


if __name__ == '__main__':
    main()
