"""확약 노면 옵션의 저장 입력 비교. 센서/운동은 고정한 개루프이며 실제 주행 결과가 아니다."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig


def run(events, enabled, self_steering):
    header = events[0]['data']
    execution = replace(ExecutionConfig(**header['execution_config']), straight_surface_commitment=enabled)
    ctl = M3ExecutionFollow(FollowConfig(**header['follow_config']), execution, allow_simulation_trial=True)
    motion = MotionConfig(**header['motion_config'])
    rows, previous = [], 0.
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            ctl.reset()
            previous = 0.
            continue
        assert event['kind'] == 'call'
        data = dict(event['data']['input'])
        if self_steering:
            data['previous_steering_rad'] = previous
        begin = time.perf_counter()
        result = freeze(replay_call(ctl, data, motion))
        elapsed = time.perf_counter()-begin
        previous = result[1]
        detail = result[2]
        trial = detail.get('m3_lateral') or {}
        checks = trial.get('checks') or {}
        rows.append({'seq': event['seq'], 't': data['reference_s'], 'output': result[:2],
                     'reason': detail['reason'], 'mode': trial.get('mode'), 'selected': trial.get('selected'),
                     'matches_recorded': equal_output(result, event['data']['output']),
                     'released': bool(trial.get('released_to_baseline')),
                     'gate': trial.get('release_gate'), 'threats': trial.get('threats'),
                     'plan_reasons': {key: value['reasons'] for key, value in checks.items()},
                     'straight_scope': (checks.get('straight') or {}).get('surface_scope'),
                     'all_rejected': bool(detail.get('m3_all_commands_rejected')), 'seconds': elapsed})
    return rows


def summarize(rows):
    active = [r for r in rows if r['mode'] not in (None, 'BASELINE')]
    return {'calls': len(rows), 'matches_recorded': sum(r['matches_recorded'] for r in rows),
            'positive': sum(r['output'][0] > 0 for r in rows),
            'active_positive': sum(r['output'][0] > 0 for r in active),
            'late_positive_after_32s': sum(r['output'][0] > 0 and r['t'] >= 32 for r in rows),
            'reasons': dict(Counter(r['reason'] for r in rows)),
            'release_times': [r['t'] for r in rows if r['released']],
            'all_rejected': sum(r['all_rejected'] for r in rows),
            'changed_surface_calls': sum(r['straight_scope'] is not None for r in rows)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('inputs', nargs='+', type=Path)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False)
    report = {'scope': 'stored sensor open-loop; self variant only substitutes previous steering; no closed-loop evidence',
              'inputs': {}}
    for path in args.inputs:
        raw = path.read_bytes()
        events = [json.loads(line) for line in raw.decode('utf8').splitlines()]
        assert events[0]['kind'] == 'header' and events[-1]['kind'] == 'footer'
        assert [e['seq'] for e in events] == list(range(len(events)))
        assert all(events[-1]['data'][k] == 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
        name = path.parent.parent.name
        variants = {'off': (False, False), 'on': (True, False)}
        if name.startswith('run32'):
            variants['self_on'] = (True, True)
        all_rows = {key: run(events, *settings) for key, settings in variants.items()}
        entry = {'sha256': hashlib.sha256(raw).hexdigest(),
                 'variants': {key: summarize(rows) for key, rows in all_rows.items()}}
        for key, rows in all_rows.items():
            (args.output/f'{name}_{key}.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf8')
        entry['on_vs_off'] = {
            'output_differences': sum(not equal_output(a['output'], b['output']) for a, b in zip(all_rows['off'], all_rows['on'])),
            'gate_differences': sum(a['gate'] != b['gate'] for a, b in zip(all_rows['off'], all_rows['on'])),
            'threat_differences': sum(a['threats'] != b['threats'] for a, b in zip(all_rows['off'], all_rows['on']))}
        entry['input_unchanged'] = hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256']
        report['inputs'][name] = entry
        (args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
        print(json.dumps({name: entry}, ensure_ascii=False), flush=True)
        assert entry['input_unchanged'] and all(r['matches_recorded'] for r in all_rows['off'])
        assert entry['on_vs_off']['gate_differences'] == entry['on_vs_off']['threat_differences'] == 0


if __name__ == '__main__':
    main()
