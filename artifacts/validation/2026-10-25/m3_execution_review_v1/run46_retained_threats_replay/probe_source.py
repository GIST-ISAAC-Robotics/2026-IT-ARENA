"""기록 보존 수정 전후의 같은 저장 입력 대조. 폐루프 성공 판정 아님."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
import zipfile
import numpy as np
from arena_autonomy.follow_trace import replay_call, freeze, equal_output
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
import arena_autonomy.m3_execution_trial as current


def run(events, module):
    header = events[0]['data']
    ctl = module.M3ExecutionFollow(FollowConfig(**header['follow_config']),
        module.ExecutionConfig(**header['execution_config']), allow_simulation_trial=True)
    motion, rows = MotionConfig(**header['motion_config']), []
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            ctl.reset()
            continue
        assert event['kind'] == 'call'
        start = time.perf_counter()
        out = freeze(replay_call(ctl, event['data']['input'], motion))
        cost = time.perf_counter()-start
        lat = out[2].get('m3_lateral') or {}
        gate = lat.get('release_gate') or {}
        rows.append({'seq': event['seq'], 't': event['data']['input']['reference_s'], 'output': out[:2],
            'reason': out[2]['reason'], 'mode': lat.get('mode'), 'released': bool(lat.get('released_to_baseline')),
            'matches_recorded': equal_output(out, event['data']['output']), 'gate_reason': gate.get('reason'),
            'records': gate.get('memory_records', 0), 'memory_blocked': gate.get('memory_blocked', {}),
            'seconds': cost})
    times = np.array([r['seconds'] for r in rows])
    return rows, {'calls': len(rows), 'positive': sum(r['output'][0] > 0 for r in rows),
        'matches_recorded': sum(r['matches_recorded'] for r in rows),
        'released_at': [r['t'] for r in rows if r['released']],
        'max_records': max(r['records'] for r in rows),
        'memory_blocked_first': next((r['t'] for r in rows if r['memory_blocked']), None),
        'reasons': dict(Counter(r['reason'] for r in rows)),
        'cost_ms': {'p50': float(np.quantile(times,.5)*1000), 'p95': float(np.quantile(times,.95)*1000),
                    'max': float(max(times)*1000)}}


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('inputs', nargs='+', type=Path)
    args = p.parse_args()
    args.output.mkdir(exist_ok=False)
    snapshot = Path('artifacts/validation/2026-10-25/m3_execution_review_v1/run35_committed_surface_corner/run/source_snapshot.zip')
    with zipfile.ZipFile(snapshot) as z:
        raw = z.read('src/arena_autonomy/arena_autonomy/m3_execution_trial.py')
    old_path = args.output/'m3_before.py'
    old_path.write_bytes(raw)
    spec = importlib.util.spec_from_file_location('m3_memory_before', old_path)
    before = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = before
    spec.loader.exec_module(before)
    digest = hashlib.sha256(Path(current.__file__).read_bytes()).hexdigest()
    summary = {'scope': 'stored input open-loop; old/new cost sequential, not real-time benchmark',
        'old_sha256': hashlib.sha256(raw).hexdigest(), 'new_sha256': digest, 'inputs': {}}
    for path in args.inputs:
        raw = path.read_bytes()
        events = [json.loads(line) for line in raw.decode('utf8').splitlines()]
        assert events[0]['kind'] == 'header' and events[-1]['kind'] == 'footer'
        assert [e['seq'] for e in events] == list(range(len(events)))
        assert all(events[-1]['data'][k] == 0 for k in ('capture_errors','failed','dropped','pending'))
        name = path.parent.parent.name
        old_rows, old_summary = run(events, before)
        new_rows, new_summary = run(events, current)
        entry = {'sha256': hashlib.sha256(raw).hexdigest(), 'before': old_summary, 'after': new_summary,
                 'output_differences': sum(not equal_output(a['output'],b['output']) for a,b in zip(old_rows,new_rows))}
        entry['inputs_unchanged'] = hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256']
        (args.output/f'{name}_before.json').write_text(json.dumps(old_rows,ensure_ascii=False),encoding='utf8')
        (args.output/f'{name}_after.json').write_text(json.dumps(new_rows,ensure_ascii=False),encoding='utf8')
        summary['inputs'][name] = entry
        summary['source_unchanged'] = hashlib.sha256(Path(current.__file__).read_bytes()).hexdigest() == digest
        (args.output/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        print(json.dumps({name:entry},ensure_ascii=False),flush=True)
        assert entry['inputs_unchanged'] and summary['source_unchanged']
        assert old_summary['matches_recorded'] == old_summary['calls']
