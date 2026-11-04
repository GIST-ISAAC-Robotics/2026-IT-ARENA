"""Existing policy counterfactuals on fixed inputs; not trajectory replay."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = ROOT/'build/side_departure_runtime_v3'
sys.path.insert(0, str(BASE))
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig


def main():
    path = ROOT/'artifacts/validation/2026-11-03/right_reapproach_v2/run1/follow_decisions.jsonl'
    variants = dict(original={}, no_static=dict(static_side_release_trial=False),
        keep=dict(rear_lane_keep_trial=True, rear_lane_keep_policy='observed_fresh_v2'),
        no_static_keep=dict(static_side_release_trial=False, rear_lane_keep_trial=True,
                            rear_lane_keep_policy='observed_fresh_v2'))
    results = {k: [] for k in variants}
    n = 0
    footer = None
    for line in path.open(encoding='utf-8'):
        event = json.loads(line); data = event['data']
        if event['seq'] != n or footer is not None:
            raise ValueError('trace ordering')
        n += 1
        if event['kind'] == 'header':
            cfg = FollowConfig(**data['follow_config']); motion = MotionConfig(**data['motion_config'])
            ex = HandoffExecutionConfig.from_mapping(data['execution_config'])
            engines = {k: BypassHandoffFollow(replace(cfg, **v), ex, allow_simulation_trial=True)
                       for k, v in variants.items()}
        elif event['kind'] == 'reset':
            for e in engines.values(): e.reset()
        elif event['kind'] == 'footer':
            footer = data
        elif event['kind'] == 'call':
            raw = data['input']
            for name, engine in engines.items():
                out = freeze(replay_call(engine, raw, motion))
                if name == 'original' and not equal_output(comparable_decision(out), comparable_decision(data['output'])):
                    (HERE/'original_mismatch.json').write_text(json.dumps(dict(seq=event['seq'], replay=out, recorded=data['output'])), encoding='utf-8')
                    raise ValueError(f'original mismatch seq={event["seq"]}')
                normal = out[2].get('normal') or out[2]
                results[name].append(dict(seq=event['seq'], t=raw['reference_s'], dt=raw['dt_s'],
                    command=out[:2], state=out[2].get('handoff_state'), reason=normal.get('reason'),
                    selector=normal.get('selector_reason'), mode=normal.get('selector_mode'),
                    sides=normal.get('selector_sides'), keep=normal.get('rear_lane_keep'),
                    parallel=(normal.get('parallel_hold_trial') or {}).get('reason')))
    if (footer is None or any(footer.get(k) != 0 for k in ('capture_errors','failed','dropped','pending'))
            or footer.get('worker_alive') is not False or footer.get('written') != n-1
            or footer.get('accepted') != n-1):
        raise ValueError('incomplete input trace')
    summary = {}
    original = results['original']
    for name, rows in results.items():
        changed = [r for r, a in zip(rows, original) if not equal_output(r['command'], a['command']) or r['state'] != a['state']]
        summary[name] = dict(calls=len(rows), changed=len(changed), first_changes=changed[:2],
            new_zero=sum(r['command'][0] <= 1e-9 < a['command'][0] for r,a in zip(rows,original)),
            released_zero=sum(a['command'][0] <= 1e-9 < r['command'][0] for r,a in zip(rows,original)),
            zero_sample_s=sum(r['dt'] for r in rows if r['command'][0] <= 1e-9),
            keep_applied=sum(bool((r['keep'] or {}).get('applied')) for r in rows))
        (HERE/f'{name}.json').write_text(json.dumps(rows, ensure_ascii=False), encoding='utf-8')
    imported = [Path(m.__file__).resolve() for k,m in sys.modules.items()
                if k.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(m,'__file__',None)]
    if not imported or not all(p.is_relative_to(BASE.resolve()) for p in imported):
        raise ValueError('mixed source imports')
    output = dict(input_sha256=hashlib.sha256(path.read_bytes()).hexdigest(), variants=variants, results=summary,
                  scope='fixed input decisions, not changed trajectory or collision proof')
    (HERE/'comparison.json').write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:{a:b for a,b in v.items() if a != 'first_changes'} for k,v in summary.items()}), flush=True)


if __name__ == '__main__': main()
